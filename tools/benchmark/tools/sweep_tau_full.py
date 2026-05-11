#!/usr/bin/env python3
"""
Comprehensive τ sensitivity sweep.

Iterates τ values, for each: builds topo map (captures room_count, timing breakdowns,
memory), runs all query pairs (captures planning_time, path_length, success, nodes).
Computes path_overhead vs Surface A* baseline.

Usage:
    python3 sweep_tau_full.py --csv benchmark.csv --taus 1,2,3,4,5,6,8,10 \
        --resolution 0.075 --trials 3 --output tau_sweep_results.csv

Requires: topo_nav_node running, JPS baseline results in --baseline-csv.
"""

import argparse
import csv
import sys
import threading
import time
from pathlib import Path
from typing import List, Dict, Optional, Tuple, Any

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Point, Pose
from nav_interfaces.srv import SetVoxelMap


def _ensure_voxel_io():
    workspace_root = Path(__file__).resolve().parents[3]
    occmap_pkg = workspace_root / 'src' / 'occmap_tools'
    if str(occmap_pkg) not in sys.path:
        sys.path.insert(0, str(occmap_pkg))
    from occmap_tools import voxel_io
    return voxel_io


class TauSweepNode(Node):
    def __init__(self, service_timeout=10.0, map_load_timeout=120.0,
                 robot_diameter=0.4):
        super().__init__('tau_sweep')
        self.service_timeout = service_timeout
        self.map_load_timeout = map_load_timeout
        self.robot_diameter = robot_diameter
        self.cb_group = ReentrantCallbackGroup()

        self.set_map_client = self.create_client(
            SetVoxelMap, '/set_voxel_map', callback_group=self.cb_group)

        self._topo_ready = False

    def _init_topo(self):
        if self._topo_ready:
            return
        from nav_interfaces.srv import BuildTopoMap, PlanHierarchical
        self.build_topo_client = self.create_client(
            BuildTopoMap, '/build_topo_map', callback_group=self.cb_group)
        self.plan_topo_client = self.create_client(
            PlanHierarchical, '/plan_hierarchical', callback_group=self.cb_group)
        self._topo_ready = True

    def _call(self, client, request, name, timeout=None):
        t = timeout or self.service_timeout
        future = client.call_async(request)
        deadline = time.time() + t
        while not future.done():
            if time.time() > deadline:
                self.get_logger().error(f'{name} timeout ({t:.1f}s)')
                return None
            time.sleep(0.01)
        return future.result()

    def upload_map(self, voxels, seed, resolution):
        req = SetVoxelMap.Request()
        req.resolution = float(resolution)
        req.seed_point.x, req.seed_point.y, req.seed_point.z = \
            float(seed[0]), float(seed[1]), float(seed[2])
        for v in voxels:
            p = Point()
            p.x, p.y, p.z = float(v[0]), float(v[1]), float(v[2])
            req.occupied.append(p)
        resp = self._call(self.set_map_client, req, 'set_voxel_map',
                          timeout=self.map_load_timeout)
        if resp and resp.success:
            return True, float(getattr(resp, 'traversability_time_ms', 0.0))
        return False, 0.0

    def build_topo(self, tau, mode='full'):
        self._init_topo()
        from nav_interfaces.srv import BuildTopoMap
        req = BuildTopoMap.Request()
        req.persistence_tau = tau
        req.floor_separation_m = 2.5
        req.corridor_aspect_ratio = 3.0
        req.staircase_variance_m = 0.5
        req.robot_diameter_m = self.robot_diameter
        req.planner_mode = mode
        resp = self._call(self.build_topo_client, req, 'build_topo_map',
                          timeout=self.map_load_timeout)
        if resp and resp.success:
            return {
                'success': True,
                'build_ms': float(resp.build_time_ms),
                'num_rooms': int(resp.num_rooms),
                'num_floors': int(resp.num_floors),
                'persistence_ms': float(getattr(resp, 'persistence_time_ms', 0.0)),
                'segmentation_ms': float(getattr(resp, 'segmentation_time_ms', 0.0)),
                'precompute_ms': float(getattr(resp, 'precompute_time_ms', 0.0)),
                'memory_bytes': int(getattr(resp, 'memory_usage_bytes', 0)),
            }
        return {'success': False}

    def plan(self, start, goal):
        self._init_topo()
        from nav_interfaces.srv import PlanHierarchical
        req = PlanHierarchical.Request()
        req.start.position.x, req.start.position.y, req.start.position.z = \
            float(start[0]), float(start[1]), float(start[2])
        req.start.orientation.w = 1.0
        req.goal.position.x, req.goal.position.y, req.goal.position.z = \
            float(goal[0]), float(goal[1]), float(goal[2])
        req.goal.orientation.w = 1.0
        req.snap_to_surface = True
        req.robot_diameter_m = self.robot_diameter
        resp = self._call(self.plan_topo_client, req, 'plan_hierarchical')
        if resp and resp.success:
            path = np.array([[p.pose.position.x, p.pose.position.y, p.pose.position.z]
                             for p in resp.path.poses])
            length = float(np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1))) \
                     if len(path) > 1 else 0.0
            return {
                'success': True,
                'ts_ms': float(resp.planning_time_ms),
                'nodes': int(resp.voxel_nodes_expanded),
                'path_m': length,
            }
        return {'success': False, 'ts_ms': 0.0, 'nodes': 0, 'path_m': 0.0}


def main():
    parser = argparse.ArgumentParser(description='τ sensitivity sweep')
    parser.add_argument('--csv', required=True, help='Input benchmark CSV (with scenes/endpoints)')
    parser.add_argument('--taus', default='1,2,3,4,5,6,8,10',
                        help='Comma-separated τ values')
    parser.add_argument('--resolution', type=float, default=0.075)
    parser.add_argument('--trials', type=int, default=3)
    parser.add_argument('--output', default='tau_sweep_results.csv')
    parser.add_argument('--occmap-dir', default=None)
    parser.add_argument('--planner-mode', default='full')
    parser.add_argument('--baseline-csv', default=None,
                        help='JPS baseline CSV for path overhead calculation')
    args = parser.parse_args()

    taus = [float(t) for t in args.taus.split(',')]

    # Read input CSV for scene/endpoint info
    with open(args.csv, newline='') as f:
        all_rows = list(csv.DictReader(f))

    # Filter TopoNav rows at target resolution
    topo_rows = [r for r in all_rows
                 if r.get('planner', '').strip() == 'TopoNav'
                 and abs(float(r.get('resolution', 0)) - args.resolution) < 0.001]
    if not topo_rows:
        # Try JPS rows as template
        topo_rows = [r for r in all_rows
                     if r.get('planner', '').strip() == 'JPS_3D'
                     and abs(float(r.get('resolution', 0)) - args.resolution) < 0.001]
    if not topo_rows:
        print(f'No rows found at resolution {args.resolution}m')
        sys.exit(1)

    # Load baseline JPS path lengths for overhead calculation
    baseline_paths = {}
    if args.baseline_csv:
        with open(args.baseline_csv, newline='') as f:
            for r in csv.DictReader(f):
                if r.get('planner', '').strip() == 'JPS_3D' and r.get('success') == 'True':
                    scene = r.get('scene', '')
                    baseline_paths[scene] = float(r.get('path_length_m', 0))

    print(f'Sweep: τ ∈ {taus}, {len(topo_rows)} scenes, res={args.resolution}m, '
          f'trials={args.trials}, mode={args.planner_mode}')

    rclpy.init()
    node = TauSweepNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    vio = _ensure_voxel_io()
    results = []
    scene_cache = {}

    try:
        # Wait for services
        if not node.set_map_client.wait_for_service(timeout_sec=10.0):
            print('set_voxel_map service not available')
            return
        node._init_topo()
        if not node.build_topo_client.wait_for_service(timeout_sec=10.0):
            print('build_topo_map service not available')
            return

        for row in topo_rows:
            scene = row.get('scene', 'unknown')
            obj_path = row.get('obj_path', '').strip()
            resolution = float(row.get('resolution', args.resolution))

            # Load voxels (cached)
            sk = (obj_path, scene, resolution)
            if sk not in scene_cache:
                npy_path = vio.find_npy_for_scan(obj_path, resolution, args.occmap_dir) \
                    if obj_path else None
                if npy_path:
                    pts = vio.load_npy(npy_path)
                elif obj_path and obj_path.lower() != 'none':
                    pts, _ = vio.load_voxels(obj_path, resolution)
                else:
                    print(f'  Cannot load scene {scene}')
                    continue
                scene_cache[sk] = pts

            pts = scene_cache[sk]
            seed = np.array([float(row['seed_x']), float(row['seed_y']), float(row['seed_z'])])
            start = np.array([float(row['start_x']), float(row['start_y']), float(row['start_z'])])
            goal = np.array([float(row['goal_x']), float(row['goal_y']), float(row['goal_z'])])

            # Upload map once per scene
            ok, te_ms = node.upload_map(pts, seed, resolution)
            if not ok:
                print(f'  Failed to upload map for {scene}')
                continue

            # Sweep τ
            for tau in taus:
                print(f'\n[τ={tau}] scene={scene}')
                build_info = node.build_topo(tau, mode=args.planner_mode)
                if not build_info['success']:
                    print(f'  Build failed for τ={tau}')
                    results.append({
                        'tau': tau, 'scene': scene, 'resolution': resolution,
                        'planner_mode': args.planner_mode,
                        'success': False,
                    })
                    continue

                # Run queries
                ts_list, nodes_list, paths_list = [], [], []
                for t in range(args.trials):
                    r = node.plan(start, goal)
                    if r['success']:
                        ts_list.append(r['ts_ms'])
                        nodes_list.append(r['nodes'])
                        paths_list.append(r['path_m'])

                success_rate = len(ts_list) / args.trials if args.trials > 0 else 0.0
                mean_path = float(np.mean(paths_list)) if paths_list else 0.0
                jps_path = baseline_paths.get(scene, 0.0)
                overhead = ((mean_path - jps_path) / jps_path * 100.0) \
                    if jps_path > 0 and mean_path > 0 else None

                row_result = {
                    'tau': tau,
                    'scene': scene,
                    'resolution': resolution,
                    'planner_mode': args.planner_mode,
                    'success': success_rate > 0,
                    'success_rate': f'{success_rate:.2f}',
                    'num_rooms': build_info.get('num_rooms', 0),
                    'build_ms': f'{build_info.get("build_ms", 0):.1f}',
                    'persistence_ms': f'{build_info.get("persistence_ms", 0):.1f}',
                    'segmentation_ms': f'{build_info.get("segmentation_ms", 0):.1f}',
                    'precompute_ms': f'{build_info.get("precompute_ms", 0):.1f}',
                    'memory_bytes': build_info.get('memory_bytes', 0),
                    'mean_query_ms': f'{np.mean(ts_list):.3f}' if ts_list else '',
                    'std_query_ms': f'{np.std(ts_list):.3f}' if ts_list else '',
                    'mean_nodes': f'{np.mean(nodes_list):.0f}' if nodes_list else '',
                    'mean_path_m': f'{mean_path:.3f}' if paths_list else '',
                    'path_overhead_pct': f'{overhead:.1f}' if overhead is not None else '',
                }
                results.append(row_result)
                print(f'  rooms={build_info.get("num_rooms")}, '
                      f'Ts={row_result["mean_query_ms"]}ms, '
                      f'path={row_result["mean_path_m"]}m, '
                      f'overhead={row_result.get("path_overhead_pct", "N/A")}%')

    finally:
        node.destroy_node()
        rclpy.shutdown()

    # Write results
    if results:
        cols = list(results[0].keys())
        with open(args.output, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=cols)
            w.writeheader()
            w.writerows(results)
        print(f'\nResults written to {args.output}')


if __name__ == '__main__':
    main()
