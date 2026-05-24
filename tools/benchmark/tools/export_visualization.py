#!/usr/bin/env python3
"""P3.3: Qualitative visualization export.

Runs specific queries with ALT and TG methods, exports paths as PLY files
and room/portal data as CSV. Used for paper qualitative figures.

Usage:
    # Export top bad-case queries from overhead analysis:
    python3 export_visualization.py \
      --bad-cases experiments/results/analysis/top_bad_overhead_tg_k5.csv \
      --top-n 5 \
      --out-dir experiments/results/visualization

    # Export specific scene/query:
    python3 export_visualization.py \
      --scene hm3d_5LpN3gDmAk7 --resolution 0.1 \
      --start -1.0 0.5 1.2 --goal 3.0 -2.0 1.2 \
      --out-dir experiments/results/visualization
"""
import argparse
import csv
import sys
import os
import time
import threading
import struct
from pathlib import Path

import numpy as np


def init_ros():
    """Initialize ROS2 and return node + service clients."""
    import rclpy
    from rclpy.node import Node
    from nav_interfaces.srv import BuildOccMap, SetVoxelMap, BuildTopoMap, PlanHierarchical
    from jps_planner_3d.srv import PlanPath

    rclpy.init()
    node = rclpy.create_node('visualization_exporter')

    clients = {
        'build_map': node.create_client(BuildOccMap, 'build_occ_map'),
        'set_voxel': node.create_client(SetVoxelMap, 'set_voxel_map'),
        'build_topo': node.create_client(BuildTopoMap, 'build_topo_map'),
        'plan_topo': node.create_client(PlanHierarchical, 'plan_hierarchical'),
        'plan_jps': node.create_client(PlanPath, 'plan_path'),
    }

    # Spin in background
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Wait for services
    for name, client in clients.items():
        if not client.wait_for_service(timeout_sec=5.0):
            print(f"[WARN] Service {name} not available")

    return node, clients


def call_service(node, client, request, timeout=120.0):
    """Call a ROS2 service synchronously."""
    future = client.call_async(request)
    start = time.time()
    while not future.done():
        if time.time() - start > timeout:
            return None
        time.sleep(0.01)
    return future.result()


def voxelize_mesh(node, clients, obj_path, resolution, seed):
    """Voxelize mesh and set voxel map."""
    from nav_interfaces.srv import BuildOccMap, SetVoxelMap
    import trimesh

    mesh = trimesh.load(obj_path, process=False)
    vertices = np.array(mesh.vertices, dtype=np.float32)
    faces = np.array(mesh.faces, dtype=np.int32)

    req = BuildOccMap.Request()
    req.vertices = vertices.flatten().tolist()
    req.triangles = faces.flatten().tolist()
    req.resolution = float(resolution)
    req.num_vertices = len(vertices)
    req.num_triangles = len(faces)

    resp = call_service(node, clients['build_map'], req, timeout=300.0)
    if resp is None or not resp.success:
        return None, None

    centers = np.array(resp.voxel_centers).reshape(-1, 3)

    # Set voxel map
    req2 = SetVoxelMap.Request()
    req2.voxel_centers = resp.voxel_centers
    req2.resolution = float(resolution)
    req2.seed_points = [float(seed[0]), float(seed[1]), float(seed[2])]
    req2.num_seeds = 1

    resp2 = call_service(node, clients['set_voxel'], req2)
    if resp2 is None or not resp2.success:
        return None, None

    return centers, resp2


def build_topo(node, clients, tau, max_portals, planner_mode='ph_alt_tg'):
    """Build topo map."""
    from nav_interfaces.srv import BuildTopoMap

    req = BuildTopoMap.Request()
    req.persistence_threshold = float(tau)
    req.floor_separation_threshold = 2.5
    req.max_portals_per_edge = int(max_portals)
    req.planner_mode = planner_mode

    resp = call_service(node, clients['build_topo'], req, timeout=300.0)
    if resp is None or not resp.success:
        return None
    return resp


def plan_query(node, clients, start, goal, method='topo'):
    """Run a planning query, return path waypoints."""
    if method == 'topo':
        from nav_interfaces.srv import PlanHierarchical
        req = PlanHierarchical.Request()
        req.start.position.x, req.start.position.y, req.start.position.z = map(float, start)
        req.start.orientation.w = 1.0
        req.goal.position.x, req.goal.position.y, req.goal.position.z = map(float, goal)
        req.goal.orientation.w = 1.0
        req.snap_to_surface = True
        req.robot_diameter_m = 0.4

        resp = call_service(node, clients['plan_topo'], req)
        if resp is None or not resp.success:
            return None, resp

        waypoints = np.array([[p.pose.position.x, p.pose.position.y, p.pose.position.z]
                              for p in resp.path.poses])
        return waypoints, resp

    elif method == 'jps':
        from jps_planner_3d.srv import PlanPath
        req = PlanPath.Request()
        req.start.position.x, req.start.position.y, req.start.position.z = map(float, start)
        req.start.orientation.w = 1.0
        req.goal.position.x, req.goal.position.y, req.goal.position.z = map(float, goal)
        req.goal.orientation.w = 1.0
        req.snap_to_surface = True

        resp = call_service(node, clients['plan_jps'], req)
        if resp is None or not resp.success:
            return None, resp

        waypoints = np.array([[p.pose.position.x, p.pose.position.y, p.pose.position.z]
                              for p in resp.path.poses])
        return waypoints, resp


def write_ply(filepath, points, colors=None):
    """Write points to PLY file (ASCII)."""
    n = len(points)
    with open(filepath, 'w') as f:
        f.write('ply\n')
        f.write('format ascii 1.0\n')
        f.write(f'element vertex {n}\n')
        f.write('property float x\n')
        f.write('property float y\n')
        f.write('property float z\n')
        if colors is not None:
            f.write('property uchar red\n')
            f.write('property uchar green\n')
            f.write('property uchar blue\n')
        f.write('end_header\n')
        for i in range(n):
            line = f'{points[i][0]:.6f} {points[i][1]:.6f} {points[i][2]:.6f}'
            if colors is not None:
                line += f' {int(colors[i][0])} {int(colors[i][1])} {int(colors[i][2])}'
            f.write(line + '\n')


def write_path_csv(filepath, waypoints, method_label):
    """Write path waypoints to CSV."""
    with open(filepath, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['method', 'index', 'x', 'y', 'z'])
        for i, p in enumerate(waypoints):
            w.writerow([method_label, i, f'{p[0]:.6f}', f'{p[1]:.6f}', f'{p[2]:.6f}'])


def export_single_query(node, clients, scene, resolution, obj_path,
                        seed, start, goal, out_dir):
    """Export visualization data for a single query across methods."""
    scene_dir = Path(out_dir) / scene
    scene_dir.mkdir(parents=True, exist_ok=True)

    qid = f'{start[0]:.1f}_{start[1]:.1f}_{goal[0]:.1f}_{goal[1]:.1f}'
    query_prefix = scene_dir / qid

    print(f"\n{'='*60}")
    print(f"  Exporting: {scene} @ {resolution}m")
    print(f"  Start: {start}")
    print(f"  Goal:  {goal}")
    print(f"{'='*60}")

    # Voxelize
    print("  [1/4] Voxelizing mesh...")
    centers, vmap_resp = voxelize_mesh(node, clients, obj_path, resolution, seed)
    if centers is None:
        print("  [FAIL] Voxelization failed")
        return False

    # Export surface map
    surface_ply = f'{query_prefix}_surface.ply'
    write_ply(surface_ply, centers, colors=np.full((len(centers), 3), [200, 200, 200]))
    print(f"  Surface: {len(centers)} voxels → {surface_ply}")

    all_paths = {}

    # ALT baseline
    print("  [2/4] ALT baseline query...")
    build_resp = build_topo(node, clients, tau=0, max_portals=1, planner_mode='alt_astar')
    if build_resp:
        path_alt, resp_alt = plan_query(node, clients, start, goal, method='topo')
        if path_alt is not None and len(path_alt) > 0:
            alt_ply = f'{query_prefix}_alt_path.ply'
            write_ply(alt_ply, path_alt, colors=np.full((len(path_alt), 3), [0, 0, 255]))
            write_path_csv(f'{query_prefix}_alt_path.csv', path_alt, 'ALT')
            all_paths['ALT'] = path_alt
            length = np.sum(np.linalg.norm(np.diff(path_alt, axis=0), axis=1))
            print(f"  ALT: {len(path_alt)} waypoints, {length:.2f}m → {alt_ply}")
        else:
            print(f"  ALT: FAILED ({getattr(resp_alt, 'failure_reason', 'unknown')})")

    # TG K=1
    print("  [3/4] TG K=1 query...")
    build_resp = build_topo(node, clients, tau=-1, max_portals=1, planner_mode='ph_alt_tg')
    if build_resp:
        topo_info = {
            'rooms': build_resp.num_rooms,
            'tau': getattr(build_resp, 'selected_tau', 0),
            'portals': getattr(build_resp, 'num_portals', 0),
        }
        path_k1, resp_k1 = plan_query(node, clients, start, goal, method='topo')
        if path_k1 is not None and len(path_k1) > 0:
            k1_ply = f'{query_prefix}_tg_k1_path.ply'
            write_ply(k1_ply, path_k1, colors=np.full((len(path_k1), 3), [255, 165, 0]))
            write_path_csv(f'{query_prefix}_tg_k1_path.csv', path_k1, 'TG_K1')
            all_paths['TG_K1'] = path_k1
            length = np.sum(np.linalg.norm(np.diff(path_k1, axis=0), axis=1))
            print(f"  TG K=1: {len(path_k1)} waypoints, {length:.2f}m, "
                  f"rooms={topo_info['rooms']}, tau={topo_info['tau']:.0f} → {k1_ply}")
        else:
            print(f"  TG K=1: FAILED ({getattr(resp_k1, 'failure_reason', 'unknown')})")

    # TG K=5
    print("  [4/4] TG K=5 query...")
    build_resp = build_topo(node, clients, tau=-1, max_portals=5, planner_mode='ph_alt_tg')
    if build_resp:
        topo_info = {
            'rooms': build_resp.num_rooms,
            'tau': getattr(build_resp, 'selected_tau', 0),
            'portals': getattr(build_resp, 'num_portals', 0),
            'k_eff': getattr(build_resp, 'portal_k_effective', 0),
        }
        path_k5, resp_k5 = plan_query(node, clients, start, goal, method='topo')
        if path_k5 is not None and len(path_k5) > 0:
            k5_ply = f'{query_prefix}_tg_k5_path.ply'
            write_ply(k5_ply, path_k5, colors=np.full((len(path_k5), 3), [0, 200, 0]))
            write_path_csv(f'{query_prefix}_tg_k5_path.csv', path_k5, 'TG_K5')
            all_paths['TG_K5'] = path_k5
            length = np.sum(np.linalg.norm(np.diff(path_k5, axis=0), axis=1))
            print(f"  TG K=5: {len(path_k5)} waypoints, {length:.2f}m, "
                  f"rooms={topo_info['rooms']}, tau={topo_info['tau']:.0f}, "
                  f"portals={topo_info['portals']}, k_eff={topo_info['k_eff']:.1f} → {k5_ply}")
        else:
            print(f"  TG K=5: FAILED ({getattr(resp_k5, 'failure_reason', 'unknown')})")

    # Write combined path comparison CSV
    if all_paths:
        combined_csv = f'{query_prefix}_all_paths.csv'
        with open(combined_csv, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['method', 'index', 'x', 'y', 'z'])
            for method, waypoints in all_paths.items():
                for i, p in enumerate(waypoints):
                    w.writerow([method, i, f'{p[0]:.6f}', f'{p[1]:.6f}', f'{p[2]:.6f}'])
        print(f"\n  Combined paths → {combined_csv}")

        # Summary
        print(f"\n  Path comparison:")
        alt_len = np.sum(np.linalg.norm(np.diff(all_paths.get('ALT', [[0,0,0]]), axis=0), axis=1)) if 'ALT' in all_paths else 0
        for method, wp in all_paths.items():
            length = np.sum(np.linalg.norm(np.diff(wp, axis=0), axis=1)) if len(wp) > 1 else 0
            oh = (length / alt_len - 1) * 100 if alt_len > 0 and method != 'ALT' else 0
            oh_str = f', OH={oh:+.1f}%' if method != 'ALT' else ''
            print(f"    {method:>8}: {length:.2f}m, {len(wp)} waypoints{oh_str}")

    return True


def main():
    ap = argparse.ArgumentParser(description='Export visualization data for paper figures')
    ap.add_argument('--bad-cases', help='CSV with bad-case queries (from analyze_overhead.py)')
    ap.add_argument('--top-n', type=int, default=5, help='Number of top bad cases to export')
    ap.add_argument('--scene', help='Single scene name')
    ap.add_argument('--resolution', type=float, default=0.1)
    ap.add_argument('--start', nargs=3, type=float, help='Start x y z')
    ap.add_argument('--goal', nargs=3, type=float, help='Goal x y z')
    ap.add_argument('--benchmark-csv', help='Benchmark CSV to look up obj_path and seeds')
    ap.add_argument('--out-dir', default='experiments/results/visualization')
    args = ap.parse_args()

    node, clients = init_ros()

    if args.bad_cases:
        # Load bad-case queries and export top N
        with open(args.bad_cases) as f:
            rows = list(csv.DictReader(f))
        rows.sort(key=lambda r: float(r.get('overhead_pct', 0)), reverse=True)
        top = rows[:args.top_n]

        # Look up scene info from benchmark CSV
        scene_info = {}
        if args.benchmark_csv:
            with open(args.benchmark_csv) as f:
                for r in csv.DictReader(f):
                    key = r.get('scene', '')
                    if key not in scene_info:
                        scene_info[key] = r

        for i, row in enumerate(top):
            scene = row['scene']
            start = [float(row['start_x']), float(row['start_y']), float(row['start_z'])]
            goal = [float(row['goal_x']), float(row['goal_y']), float(row['goal_z'])]

            info = scene_info.get(scene, {})
            obj_path = info.get('obj_path', '')
            seed = [float(info.get('seed_x', 0)), float(info.get('seed_y', 0)),
                    float(info.get('seed_z', 0))]

            if not obj_path or not os.path.exists(obj_path):
                print(f"[SKIP] {scene}: obj_path not found ({obj_path})")
                continue

            print(f"\n[{i+1}/{len(top)}] Bad case OH={float(row.get('overhead_pct', 0)):+.1f}%")
            export_single_query(node, clients, scene, args.resolution, obj_path,
                                seed, start, goal, args.out_dir)

    elif args.scene and args.start and args.goal:
        # Look up obj_path from benchmark CSV
        obj_path = ''
        seed = [0, 0, 0]
        if args.benchmark_csv:
            with open(args.benchmark_csv) as f:
                for r in csv.DictReader(f):
                    if r.get('scene') == args.scene:
                        obj_path = r['obj_path']
                        seed = [float(r['seed_x']), float(r['seed_y']), float(r['seed_z'])]
                        break

        if not obj_path:
            print(f"Scene {args.scene} not found in benchmark CSV. Provide --benchmark-csv.")
            sys.exit(1)

        export_single_query(node, clients, args.scene, args.resolution, obj_path,
                            seed, args.start, args.goal, args.out_dir)
    else:
        print("Specify either --bad-cases or --scene/--start/--goal")
        sys.exit(1)

    import rclpy
    rclpy.shutdown()


if __name__ == '__main__':
    main()
