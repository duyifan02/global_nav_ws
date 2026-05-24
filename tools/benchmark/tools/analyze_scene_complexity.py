#!/usr/bin/env python3
"""P2.3: Per-scene complexity analysis and speedup correlation.

Reads ALT + TG result CSVs, computes per-scene metrics, and outputs
a summary CSV suitable for scatter plots (speedup vs complexity).

Usage:
    python3 analyze_scene_complexity.py \
      --alt experiments/results/hm3d_100/100_alt_01m_500q.csv \
      --tg experiments/results/hm3d_100/100_tg_auto_k5_01m_500q.csv \
      --out experiments/results/analysis/scene_complexity.csv
"""
import argparse
import csv
import statistics
from collections import defaultdict
from pathlib import Path
import math


def sf(v, default=0.0):
    try:
        return float(v)
    except (ValueError, TypeError):
        return default


def si(v, default=0):
    try:
        return int(float(v))
    except (ValueError, TypeError):
        return default


def gini(values):
    """Gini coefficient (0 = equal, 1 = maximal inequality)."""
    if not values or len(values) < 2:
        return 0.0
    s = sorted(values)
    n = len(s)
    total = sum(s)
    if total == 0:
        return 0.0
    cum = sum((2 * i - n + 1) * s[i] for i in range(n))
    return cum / (n * total)


def analyze(alt_path: str, tg_path: str, out_path: str):
    with open(alt_path, newline='') as f:
        alt_rows = list(csv.DictReader(f))
    with open(tg_path, newline='') as f:
        tg_rows = list(csv.DictReader(f))

    # Group by scene
    alt_by_scene = defaultdict(list)
    tg_by_scene = defaultdict(list)
    for r in alt_rows:
        alt_by_scene[r.get('scene', '')].append(r)
    for r in tg_rows:
        tg_by_scene[r.get('scene', '')].append(r)

    scenes = sorted(set(alt_by_scene.keys()) | set(tg_by_scene.keys()))
    results = []

    for scene in scenes:
        a_rows = alt_by_scene.get(scene, [])
        t_rows = tg_by_scene.get(scene, [])
        if not a_rows or not t_rows:
            continue

        # Scene-level info from first TG row
        first_tg = t_rows[0]

        # Successful queries
        a_succ = [r for r in a_rows if r.get('success') == 'True' and sf(r.get('path_length_m')) > 0]
        t_succ = [r for r in t_rows if r.get('success') == 'True' and sf(r.get('path_length_m')) > 0]

        # Query stats
        a_ts = [sf(r['planning_time_ms']) for r in a_succ] if a_succ else [0]
        t_ts = [sf(r['planning_time_ms']) for r in t_succ] if t_succ else [0]
        a_paths = [sf(r['path_length_m']) for r in a_succ] if a_succ else [0]
        t_paths = [sf(r['path_length_m']) for r in t_succ] if t_succ else [0]

        mean_alt_ts = statistics.mean(a_ts)
        mean_tg_ts = statistics.mean(t_ts)
        speedup = mean_alt_ts / mean_tg_ts if mean_tg_ts > 0 else 0

        # Paired overhead for this scene
        alt_map = {}
        for r in a_succ:
            key = f"{r.get('start_x')}_{r.get('start_y')}_{r.get('start_z')}_{r.get('goal_x')}_{r.get('goal_y')}_{r.get('goal_z')}"
            alt_map[key] = sf(r['path_length_m'])
        tg_map = {}
        for r in t_succ:
            key = f"{r.get('start_x')}_{r.get('start_y')}_{r.get('start_z')}_{r.get('goal_x')}_{r.get('goal_y')}_{r.get('goal_z')}"
            tg_map[key] = sf(r['path_length_m'])

        common = set(alt_map.keys()) & set(tg_map.keys())
        overheads = [(tg_map[k] / alt_map[k] - 1) * 100 for k in common if alt_map[k] > 0]
        mean_oh = statistics.mean(overheads) if overheads else 0

        # Euclidean and geodesic distances
        euclid_dists = []
        geodesic_dists = []
        for r in a_succ:
            sx, sy, sz = sf(r.get('start_x')), sf(r.get('start_y')), sf(r.get('start_z'))
            gx, gy, gz = sf(r.get('goal_x')), sf(r.get('goal_y')), sf(r.get('goal_z'))
            euclid = math.sqrt((gx-sx)**2 + (gy-sy)**2 + (gz-sz)**2)
            geodesic = sf(r.get('path_length_m'))
            euclid_dists.append(euclid)
            geodesic_dists.append(geodesic)

        mean_euclid = statistics.mean(euclid_dists) if euclid_dists else 0
        mean_geodesic = statistics.mean(geodesic_dists) if geodesic_dists else 0
        mean_tortuosity = (mean_geodesic / mean_euclid) if mean_euclid > 0 else 0

        # Room transitions
        room_hops = [sf(r.get('diag_room_hops', 0)) for r in t_succ]
        same_room = [1 for r in t_succ if r.get('diag_same_room') == 'True']

        # Scene complexity
        num_rooms = si(first_tg.get('num_rooms', 0))
        num_room_edges = si(first_tg.get('num_room_edges', 0))
        num_portals = si(first_tg.get('num_portals', 0))
        tg_nodes = si(first_tg.get('tg_nodes', 0))
        tg_edges = si(first_tg.get('tg_edges', 0))
        selected_tau = sf(first_tg.get('selected_tau', 0))
        k_effective = sf(first_tg.get('portal_k_effective', 0))
        memory_mb = sf(first_tg.get('memory_usage_bytes', 0)) / 1e6
        topo_ms = sf(first_tg.get('topo_ms', 0))

        # Fallback stats
        fb_nodes = [si(r.get('diag_fallback_nodes', 0)) for r in t_succ]

        results.append({
            'scene': scene,
            'resolution': first_tg.get('resolution', ''),
            'num_rooms': num_rooms,
            'num_room_edges': num_room_edges,
            'num_portals': num_portals,
            'tg_nodes': tg_nodes,
            'tg_edges': tg_edges,
            'selected_tau': selected_tau,
            'k_effective': k_effective,
            'memory_mb': f"{memory_mb:.2f}",
            'topo_ms': f"{topo_ms:.1f}",
            'alt_success': len(a_succ),
            'tg_success': len(t_succ),
            'alt_total': len(a_rows),
            'tg_total': len(t_rows),
            'mean_alt_ts_ms': f"{mean_alt_ts:.4f}",
            'mean_tg_ts_ms': f"{mean_tg_ts:.4f}",
            'speedup': f"{speedup:.2f}",
            'mean_path_overhead_pct': f"{mean_oh:.2f}",
            'n_paired': len(common),
            'mean_alt_path_m': f"{statistics.mean(a_paths):.2f}" if a_paths else '',
            'mean_tg_path_m': f"{statistics.mean(t_paths):.2f}" if t_paths else '',
            'mean_euclidean_m': f"{mean_euclid:.2f}",
            'mean_geodesic_m': f"{mean_geodesic:.2f}",
            'mean_tortuosity': f"{mean_tortuosity:.3f}",
            'mean_room_hops': f"{statistics.mean(room_hops):.1f}" if room_hops else '',
            'same_room_ratio': f"{len(same_room)/len(t_succ):.3f}" if t_succ else '',
            'mean_fallback_nodes': f"{statistics.mean(fb_nodes):.0f}" if fb_nodes else '',
            'max_fallback_nodes': max(fb_nodes) if fb_nodes else '',
        })

    # Write output
    Path(out_path).parent.mkdir(parents=True, exist_ok=True)
    if results:
        with open(out_path, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(results[0].keys()))
            w.writeheader()
            w.writerows(results)
        print(f"Written {len(results)} scenes to {out_path}")

    # Print summary
    speedups = [sf(r['speedup']) for r in results if sf(r['speedup']) > 0]
    ohs = [sf(r['mean_path_overhead_pct']) for r in results]
    print(f"\n{'='*70}")
    print(f"  Scene Complexity Summary ({len(results)} scenes)")
    print(f"{'='*70}")
    print(f"  Speedup: mean={statistics.mean(speedups):.2f}x, "
          f"median={statistics.median(speedups):.2f}x, "
          f"min={min(speedups):.2f}x, max={max(speedups):.2f}x")
    print(f"  Overhead: mean={statistics.mean(ohs):+.1f}%, "
          f"median={statistics.median(ohs):+.1f}%")
    rooms = [si(r['num_rooms']) for r in results]
    print(f"  Rooms: mean={statistics.mean(rooms):.1f}, "
          f"min={min(rooms)}, max={max(rooms)}")

    # Correlation analysis
    print(f"\n  Speedup correlations (Spearman-like, sign+magnitude):")
    for metric, key in [
        ('num_rooms', 'num_rooms'), ('num_portals', 'num_portals'),
        ('mean_geodesic', 'mean_geodesic_m'), ('tortuosity', 'mean_tortuosity'),
        ('mean_room_hops', 'mean_room_hops'),
    ]:
        pairs = [(sf(r[key]), sf(r['speedup'])) for r in results
                 if sf(r[key]) > 0 and sf(r['speedup']) > 0]
        if len(pairs) < 5:
            continue
        # Simple correlation via rank
        xs = [p[0] for p in pairs]
        ys = [p[1] for p in pairs]
        mx, my = statistics.mean(xs), statistics.mean(ys)
        cov = sum((x-mx)*(y-my) for x, y in pairs) / len(pairs)
        sx = statistics.pstdev(xs) or 1
        sy = statistics.pstdev(ys) or 1
        corr = cov / (sx * sy)
        print(f"    {metric:<20}: r={corr:+.3f} (n={len(pairs)})")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--alt', required=True)
    ap.add_argument('--tg', required=True)
    ap.add_argument('--out', default='experiments/results/analysis/scene_complexity.csv')
    args = ap.parse_args()
    analyze(args.alt, args.tg, args.out)


if __name__ == '__main__':
    main()
