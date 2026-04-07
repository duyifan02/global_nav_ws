#!/usr/bin/env python3
"""
Multi-query amortization benchmark.

For each scene:
  1. Upload voxel map once (set_voxel_map)
  2. TopoNav: build_topo_map once, then run N queries
  3. JPS: run N queries (no precompute)
  4. Record: per-query Ts, amortized total time = (topo_ms + N*Ts) / N

This shows that TopoNav's one-time topo_ms cost is amortized over many queries,
making it faster than JPS on a per-query basis for large N.

Usage:
    python3 multi_query_benchmark.py \
        --csv benchmark_v4b_det_results.csv \
        --occmap-dir /path/to/matterport3d_occmaps \
        --n-queries 50 \
        --output multi_query_results.csv
"""
import argparse
import csv
import sys
import time
from pathlib import Path
from typing import List, Dict, Any

import numpy as np
import rclpy
from rclpy.node import Node

# Reuse node from auto_benchmark
sys.path.insert(0, str(Path(__file__).parent))
from auto_benchmark import AutoBenchmarkNode, _load_scene, _scene_key


def make_random_queries(voxel_centers: np.ndarray, seed: np.ndarray,
                        n: int, rng: np.random.Generator) -> List[Dict]:
    """Sample n random (start, goal) pairs from surface voxels near seed."""
    # Use all voxels as candidate positions
    n_vox = len(voxel_centers)
    idx = rng.integers(0, n_vox, size=(n * 2,))
    queries = []
    for i in range(n):
        queries.append({
            'start': voxel_centers[idx[i * 2]],
            'goal':  voxel_centers[idx[i * 2 + 1]],
        })
    return queries


def run_multi_query(node: AutoBenchmarkNode, args, rows: List[dict]) -> List[dict]:
    results = []
    rng = np.random.default_rng(42)

    # Group rows by scene
    scenes_done = set()
    for row in rows:
        sk = _scene_key(row)
        if sk in scenes_done:
            continue
        scenes_done.add(sk)

        print(f'\n{"="*60}')
        print(f'Scene: {row["scene"]}  res={row["resolution"]}')

        voxel_centers, tp_ms = _load_scene(row, args.occmap_dir)
        if voxel_centers is None:
            print(f'  SKIP: scene load failed')
            continue
        print(f'  Loaded {len(voxel_centers)} voxels (tp={tp_ms:.1f}ms)')

        seed = np.array([float(row['seed_x']), float(row['seed_y']), float(row['seed_z'])])
        resolution = float(row['resolution'])

        # Upload map once (shared by JPS and TopoNav)
        ok, te_ms, _, mem = node.set_voxel_map(voxel_centers, seed, resolution)
        if not ok:
            print(f'  SKIP: set_voxel_map failed')
            continue
        print(f'  Map uploaded: te={te_ms:.1f}ms, mem={mem//1024//1024}MB')

        # Build topo map once
        ok2, topo_ms = node.build_topo_map()
        if not ok2:
            print(f'  SKIP: build_topo_map failed')
            continue
        print(f'  Topo built: topo_ms={topo_ms:.1f}ms')

        # Generate random queries
        queries = make_random_queries(voxel_centers, seed, args.n_queries, rng)
        print(f'  Running {args.n_queries} queries for JPS and TopoNav...')

        # Run JPS queries
        jps_times, jps_nodes, jps_paths, jps_success = [], [], [], 0
        for q in queries:
            r = node.plan_jps(q['start'], q['goal'])
            if r['success']:
                jps_times.append(r['ts_ms'])
                jps_nodes.append(r['ns_visited_nodes'])
                jps_paths.append(r['path_length_m'])
                jps_success += 1

        # Run TopoNav queries
        topo_times, topo_nodes, topo_paths, topo_success = [], [], [], 0
        for q in queries:
            r = node.plan_topo(q['start'], q['goal'])
            if r['success']:
                topo_times.append(r['ts_ms'])
                topo_nodes.append(r['ns_visited_nodes'])
                topo_paths.append(r['path_length_m'])
                topo_success += 1

        n_jps = len(jps_times)
        n_topo = len(topo_times)

        jps_mean_ts  = float(np.mean(jps_times))  if jps_times  else 0
        topo_mean_ts = float(np.mean(topo_times)) if topo_times else 0

        # Amortized cost per query = (precompute + N*Ts) / N
        jps_amortized  = (te_ms + n_jps  * jps_mean_ts)  / max(n_jps, 1)
        topo_amortized = (topo_ms + n_topo * topo_mean_ts) / max(n_topo, 1)

        print(f'  JPS:   {n_jps}/{args.n_queries} ok | Ts={jps_mean_ts:.2f}ms | amortized={jps_amortized:.2f}ms')
        print(f'  Topo:  {n_topo}/{args.n_queries} ok | Ts={topo_mean_ts:.2f}ms | topo_precompute={topo_ms:.1f}ms | amortized={topo_amortized:.2f}ms')

        scan_id = None
        for part in Path(row.get('obj_path','')).parts:
            if part == 'scans':
                idx_part = list(Path(row['obj_path']).parts).index('scans')
                scan_id = list(Path(row['obj_path']).parts)[idx_part + 1]
                break

        results.append({
            'scene': row['scene'],
            'scan_id': scan_id or row['scene'],
            'resolution': resolution,
            'n_queries': args.n_queries,
            'surface_voxels': len(voxel_centers),
            'te_ms': round(te_ms, 3),
            'topo_ms': round(topo_ms, 3),
            'jps_success': n_jps,
            'jps_mean_ts_ms': round(jps_mean_ts, 3),
            'jps_mean_nodes': round(float(np.mean(jps_nodes)), 0) if jps_nodes else 0,
            'jps_mean_path_m': round(float(np.mean(jps_paths)), 3) if jps_paths else 0,
            'jps_amortized_ms': round(jps_amortized, 3),
            'topo_success': n_topo,
            'topo_mean_ts_ms': round(topo_mean_ts, 3),
            'topo_mean_nodes': round(float(np.mean(topo_nodes)), 0) if topo_nodes else 0,
            'topo_mean_path_m': round(float(np.mean(topo_paths)), 3) if topo_paths else 0,
            'topo_amortized_ms': round(topo_amortized, 3),
            'speedup_ts': round(jps_mean_ts / topo_mean_ts, 3) if topo_mean_ts > 0 else 0,
            'speedup_amortized': round(jps_amortized / topo_amortized, 3) if topo_amortized > 0 else 0,
        })

    return results


def main():
    parser = argparse.ArgumentParser(description='Multi-query amortization benchmark')
    parser.add_argument('--csv', required=True, help='Input CSV with scene/seed/obj_path info')
    parser.add_argument('--occmap-dir', default=None)
    parser.add_argument('--n-queries', type=int, default=50)
    parser.add_argument('--output', required=True, help='Output CSV path')
    parser.add_argument('--tau', type=float, default=8.0)
    parser.add_argument('--robot-diameter', type=float, default=0.4)
    parser.add_argument('--service-timeout', type=float, default=10.0)
    parser.add_argument('--map-load-timeout', type=float, default=120.0)
    args = parser.parse_args()

    rows = list(csv.DictReader(open(args.csv)))

    rclpy.init()
    node = AutoBenchmarkNode(
        service_timeout_sec=args.service_timeout,
        map_load_timeout_sec=args.map_load_timeout,
        persistence_tau=args.tau,
        robot_diameter_m=args.robot_diameter,
    )
    if not node.wait_for_services():
        print('ERROR: services not available')
        rclpy.shutdown()
        return

    results = run_multi_query(node, args, rows)

    if results:
        fieldnames = list(results[0].keys())
        with open(args.output, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=fieldnames)
            w.writeheader()
            w.writerows(results)
        print(f'\n写入 {len(results)} 场景 → {args.output}')

        print('\n=== 摊销加速比（N={} queries）==='.format(args.n_queries))
        for r in results:
            print(f"  {r['scan_id']:20s}  JPS_amort={r['jps_amortized_ms']:.2f}ms  "
                  f"Topo_amort={r['topo_amortized_ms']:.2f}ms  "
                  f"speedup={r['speedup_amortized']:.2f}x")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
