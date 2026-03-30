#!/usr/bin/env python3
"""
自动化 Benchmark 脚本

从 benchmark.csv 读取测试用例，自动调用 ROS2 规划服务并写回结果。
无需 RViz 交互，直接使用 CSV 中的种子点/起点/终点。

CSV 列说明:
    planner         - JPS_3D | TopoNav
    timestamp       - 记录时间戳（保持不变）
    scene           - 场景名
    resolution      - 体素分辨率 (m)
    obj_path        - OBJ 文件路径（None 则用 PCD）
    seed_x/y/z      - BFS 建图种子点
    start_x/y/z     - 导航起点
    goal_x/y/z      - 导航终点
    planning_time_ms - 规划时间 (None 表示未运行)

新增结果列:
    success              - 是否规划成功
    path_length_m        - 路径长度 (m)
    ns_visited_nodes     - 展开节点数 (JPS_3D)
    tp_ms                - 点云预处理时间
    te_ms                - 遍历面提取时间
    topo_ms              - 拓扑地图构建时间 (TopoNav only)
    memory_usage_bytes   - 场景表示内存
    total_time_ms        - Tall = Tp + Te + [Ttopo] + Ts
    trials_count         - 实际运行次数
    planning_time_std_ms - 规划时间标准差

用法:
    # 先启动对应规划器节点:
    ros2 run jps_planner_3d jps_planner_node
    ros2 run topo_nav topo_nav_node

    # 运行全部未完成的测试 (planning_time_ms == None):
    python3 auto_benchmark.py --csv /path/to/benchmark.csv

    # 指定规划器、强制重跑所有、多次试验取均值:
    python3 auto_benchmark.py --csv benchmark.csv --planner JPS_3D --trials 10 --force

    # TopoNav 参数示例:
    python3 auto_benchmark.py --csv benchmark.csv --planner TopoNav --trials 5 \\
        --tau 3.0 --floor-sep 2.5 --robot-diameter 0.4
"""

import argparse
import csv
import sys
import time
from pathlib import Path
from datetime import datetime
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Point, Pose
from nav_interfaces.srv import SetVoxelMap, LoadPcdMap, PlanPath, GetDistanceField

# ── Column definitions ─────────────────────────────────────────────────────────

INPUT_COLS = [
    'planner', 'timestamp', 'scene', 'resolution', 'obj_path',
    'seed_x', 'seed_y', 'seed_z',
    'start_x', 'start_y', 'start_z',
    'goal_x', 'goal_y', 'goal_z',
    'planning_time_ms',
]

RESULT_COLS = [
    'success',
    'planning_time_ms',
    'planning_time_std_ms',
    'path_length_m',
    'ns_visited_nodes',
    'tp_ms',
    'te_ms',
    'topo_ms',
    'memory_usage_bytes',
    'total_time_ms',
    'trials_count',
]

ALL_COLS = [c for c in INPUT_COLS if c != 'planning_time_ms'] + RESULT_COLS


# ── Helpers ────────────────────────────────────────────────────────────────────

def _parse_float(v) -> Optional[float]:
    if v is None or str(v).strip().lower() in ('none', '', 'nan'):
        return None
    return float(v)


def _row_needs_run(row: dict, force: bool, planner_filter: Optional[str]) -> bool:
    if planner_filter and row.get('planner', '').strip() != planner_filter:
        return False
    if force:
        return True
    return _parse_float(row.get('planning_time_ms')) is None


def _load_voxels_from_obj(obj_path: str, resolution: float,
                           obj_origin: List[float], logger) -> Tuple[np.ndarray, list, float]:
    """Return (voxel_centers, occupied_list, preprocessing_ms)."""
    workspace_root = Path(__file__).resolve().parents[3]
    occmap_pkg = workspace_root / 'src' / 'occmap_tools'
    if str(occmap_pkg) not in sys.path:
        sys.path.insert(0, str(occmap_pkg))

    from occmap_tools.obj_mesh_loader import load_obj_mesh, voxelize_mesh

    t0 = time.time()
    vertices, faces = load_obj_mesh(obj_path)
    if vertices.size == 0 or faces.size == 0:
        raise RuntimeError(f'OBJ 无有效几何数据: {obj_path}')

    voxel_centers = voxelize_mesh(vertices, faces, resolution,
                                   tuple(obj_origin), logger=logger)
    t1 = time.time()

    if voxel_centers.size == 0:
        raise RuntimeError(f'OBJ 体素化结果为空: {obj_path}')

    occupied_list = [tuple(v) for v in voxel_centers]
    return voxel_centers, occupied_list, (t1 - t0) * 1000.0


def _load_voxels_from_pcd(scene_name: str, resolution: float) -> Tuple[np.ndarray, list, float]:
    benchmark_root = Path(__file__).resolve().parent.parent
    if str(benchmark_root) not in sys.path:
        sys.path.insert(0, str(benchmark_root))
    from converters.pcd_to_voxel import get_voxel_map

    t0 = time.time()
    vmap = get_voxel_map(scene_name, resolution)
    t1 = time.time()
    return vmap.voxel_centers, vmap.occupied_list, (t1 - t0) * 1000.0


# ── ROS2 Node ──────────────────────────────────────────────────────────────────

class AutoBenchmarkNode(Node):
    def __init__(self,
                 service_timeout_sec: float = 10.0,
                 map_load_timeout_sec: float = 120.0,
                 # Topo params
                 persistence_tau: float = 3.0,
                 floor_separation_m: float = 2.5,
                 corridor_aspect_ratio: float = 3.0,
                 staircase_variance_m: float = 0.5,
                 robot_diameter_m: float = 0.4,
                 connectivity_height: float = 0.3,
                 surface_clearance: float = 1.6,
                 distance_field_max: int = 10):
        super().__init__('auto_benchmark')

        self.service_timeout_sec = service_timeout_sec
        self.map_load_timeout_sec = map_load_timeout_sec

        # Topo params
        self.persistence_tau = persistence_tau
        self.floor_separation_m = floor_separation_m
        self.corridor_aspect_ratio = corridor_aspect_ratio
        self.staircase_variance_m = staircase_variance_m
        self.robot_diameter_m = robot_diameter_m
        self.connectivity_height = connectivity_height
        self.surface_clearance = surface_clearance
        self.distance_field_max = distance_field_max

        self.cb_group = ReentrantCallbackGroup()

        # JPS_3D services
        self.set_map_client = self.create_client(
            SetVoxelMap, '/set_voxel_map', callback_group=self.cb_group)
        self.plan_jps_client = self.create_client(
            PlanPath, '/plan_path', callback_group=self.cb_group)
        self.get_surface_client = self.create_client(
            GetDistanceField, '/get_distance_field', callback_group=self.cb_group)

        # TopoNav services (imported lazily if needed)
        self._topo_clients_ready = False

    def _init_topo_clients(self):
        if self._topo_clients_ready:
            return
        from nav_interfaces.srv import BuildTopoMap, PlanHierarchical
        from rcl_interfaces.srv import SetParameters
        self.build_topo_client = self.create_client(
            BuildTopoMap, '/build_topo_map', callback_group=self.cb_group)
        self.plan_topo_client = self.create_client(
            PlanHierarchical, '/plan_hierarchical', callback_group=self.cb_group)
        self.set_params_client = self.create_client(
            SetParameters, '/topo_nav_node/set_parameters', callback_group=self.cb_group)
        self._topo_clients_ready = True

    def _call_service(self, client, request, name: str,
                      timeout_sec: Optional[float] = None):
        t = float(timeout_sec if timeout_sec is not None else self.service_timeout_sec)
        try:
            response = client.call(request, timeout_sec=t)
        except TypeError:
            response = client.call(request)
        except Exception as exc:
            self.get_logger().error(f'{name} 调用异常: {exc}')
            return None
        if response is None:
            self.get_logger().error(f'{name} 超时或失败 (timeout={t:.1f}s)')
        return response

    def wait_for_jps_services(self) -> bool:
        self.get_logger().info('等待 JPS 服务...')
        if not self.set_map_client.wait_for_service(timeout_sec=self.service_timeout_sec):
            self.get_logger().error('set_voxel_map 服务不可用')
            return False
        if not self.plan_jps_client.wait_for_service(timeout_sec=self.service_timeout_sec):
            self.get_logger().error('plan_path 服务不可用')
            return False
        self.get_logger().info('JPS 服务已连接')
        return True

    def wait_for_topo_services(self) -> bool:
        self._init_topo_clients()
        self.get_logger().info('等待 TopoNav 服务...')
        for client, name in [
            (self.set_map_client, 'set_voxel_map'),
            (self.build_topo_client, 'build_topo_map'),
            (self.plan_topo_client, 'plan_hierarchical'),
        ]:
            if not client.wait_for_service(timeout_sec=self.service_timeout_sec):
                self.get_logger().error(f'{name} 服务不可用')
                return False
        self.get_logger().info('TopoNav 服务已连接')
        return True

    # ── Map loading ────────────────────────────────────────────────────────────

    def set_voxel_map(self, occupied_list: list, seed: np.ndarray,
                      resolution: float) -> Tuple[bool, float, int, float]:
        """发送体素地图到规划器节点。
        Returns: (success, traversability_ms, surface_count, memory_bytes)
        """
        request = SetVoxelMap.Request()
        request.resolution = float(resolution)
        request.seed_point.x = float(seed[0])
        request.seed_point.y = float(seed[1])
        request.seed_point.z = float(seed[2])
        for v in occupied_list:
            p = Point()
            p.x, p.y, p.z = float(v[0]), float(v[1]), float(v[2])
            request.occupied.append(p)

        print(f'  [MAP] 发送 {len(occupied_list)} 个体素 + seed {seed.tolist()} ...')
        response = self._call_service(
            self.set_map_client, request, 'set_voxel_map',
            timeout_sec=self.map_load_timeout_sec)

        if response and response.success:
            te_ms = float(getattr(response, 'traversability_time_ms', 0.0))
            mem = int(getattr(response, 'map_memory_usage_bytes', 0))
            surf = int(getattr(response, 'surface_voxel_count', 0))
            print(f'  [MAP] 建图成功: {surf} 可通行体素, Te={te_ms:.1f}ms, '
                  f'Size={mem} bytes')
            return True, te_ms, surf, mem
        else:
            msg = response.message if response else 'No response'
            print(f'  [MAP] 建图失败: {msg}')
            return False, 0.0, 0, 0

    def build_topo_map(self) -> Tuple[bool, float]:
        """构建拓扑地图。Returns: (success, build_ms)"""
        from nav_interfaces.srv import BuildTopoMap
        request = BuildTopoMap.Request()
        request.persistence_tau = self.persistence_tau
        request.floor_separation_m = self.floor_separation_m
        request.corridor_aspect_ratio = self.corridor_aspect_ratio
        request.staircase_variance_m = self.staircase_variance_m
        request.robot_diameter_m = self.robot_diameter_m

        print(f'  [TOPO] 构建拓扑地图 (τ={self.persistence_tau})...')
        response = self._call_service(
            self.build_topo_client, request, 'build_topo_map')

        if response and response.success:
            topo_ms = float(getattr(response, 'build_time_ms', 0.0))
            print(f'  [TOPO] 拓扑地图构建成功: '
                  f'{response.num_rooms} 房间, {response.num_floors} 楼层, '
                  f'{response.num_staircases} 楼梯间, Ttopo={topo_ms:.1f}ms')
            return True, topo_ms
        else:
            msg = response.message if response else 'No response'
            print(f'  [TOPO] 拓扑地图构建失败: {msg}')
            return False, 0.0

    # ── Planning ───────────────────────────────────────────────────────────────

    def plan_jps(self, start: np.ndarray, goal: np.ndarray) -> Dict[str, Any]:
        request = PlanPath.Request()
        request.start.position.x = float(start[0])
        request.start.position.y = float(start[1])
        request.start.position.z = float(start[2])
        request.start.orientation.w = 1.0
        request.goal.position.x = float(goal[0])
        request.goal.position.y = float(goal[1])
        request.goal.position.z = float(goal[2])
        request.goal.orientation.w = 1.0
        request.snap_to_surface = True

        response = self._call_service(self.plan_jps_client, request, 'plan_path')
        if response is None:
            return {'success': False, 'error': 'Service timeout',
                    'ts_ms': 0.0, 'ns_visited_nodes': 0, 'path_length_m': 0.0}
        if response.success:
            path = np.array([[p.pose.position.x, p.pose.position.y, p.pose.position.z]
                             for p in response.path.poses])
            length = (np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1))
                      if len(path) > 1 else 0.0)
            return {
                'success': True,
                'ts_ms': float(response.planning_time_ms),
                'ns_visited_nodes': int(response.expanded_nodes),
                'path_length_m': float(length),
                'memory_usage_bytes': int(getattr(response, 'memory_usage_bytes', 0)),
                'te_ms': float(getattr(response, 'traversability_time_ms', 0.0)),
                'error': None,
            }
        else:
            return {
                'success': False,
                'error': response.message,
                'ts_ms': float(response.planning_time_ms),
                'ns_visited_nodes': int(response.expanded_nodes),
                'path_length_m': 0.0,
            }

    def plan_topo(self, start: np.ndarray, goal: np.ndarray) -> Dict[str, Any]:
        from nav_interfaces.srv import PlanHierarchical
        request = PlanHierarchical.Request()
        request.start.position.x = float(start[0])
        request.start.position.y = float(start[1])
        request.start.position.z = float(start[2])
        request.start.orientation.w = 1.0
        request.goal.position.x = float(goal[0])
        request.goal.position.y = float(goal[1])
        request.goal.position.z = float(goal[2])
        request.goal.orientation.w = 1.0
        request.snap_to_surface = True
        request.robot_diameter_m = self.robot_diameter_m

        response = self._call_service(self.plan_topo_client, request, 'plan_hierarchical')
        if response is None:
            return {'success': False, 'error': 'Service timeout',
                    'ts_ms': 0.0, 'ns_visited_nodes': 0, 'path_length_m': 0.0}
        if response.success:
            path = np.array([[p.pose.position.x, p.pose.position.y, p.pose.position.z]
                             for p in response.path.poses])
            length = (np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1))
                      if len(path) > 1 else 0.0)
            return {
                'success': True,
                'ts_ms': float(response.planning_time_ms),
                'ns_visited_nodes': int(response.voxel_nodes_expanded),
                'path_length_m': float(length),
                'error': None,
            }
        else:
            return {
                'success': False,
                'error': response.message,
                'ts_ms': float(response.planning_time_ms),
                'ns_visited_nodes': int(response.voxel_nodes_expanded),
                'path_length_m': 0.0,
            }

    # ── High-level per-row runner ──────────────────────────────────────────────

    def run_jps_row(self, row: dict, occupied_list: list, voxel_centers: np.ndarray,
                    tp_ms: float, num_trials: int) -> dict:
        seed = np.array([float(row['seed_x']), float(row['seed_y']), float(row['seed_z'])])
        start = np.array([float(row['start_x']), float(row['start_y']), float(row['start_z'])])
        goal = np.array([float(row['goal_x']), float(row['goal_y']), float(row['goal_z'])])

        ok, te_ms, _, mem = self.set_voxel_map(occupied_list, seed, float(row['resolution']))
        if not ok:
            return _failed_result(tp_ms=tp_ms)

        ts_list, nodes_list, lengths = [], [], []
        for i in range(num_trials):
            r = self.plan_jps(start, goal)
            status = (f"✓ Ts={r['ts_ms']:.2f}ms | {r.get('path_length_m', 0):.2f}m"
                      if r['success'] else f"✗ {r.get('error', '?')}")
            print(f'    [{i+1:3d}/{num_trials}] {status}')
            if r['success']:
                ts_list.append(r['ts_ms'])
                nodes_list.append(r['ns_visited_nodes'])
                lengths.append(r['path_length_m'])
                mem = max(mem, int(r.get('memory_usage_bytes', 0)))
                te_ms = max(te_ms, float(r.get('te_ms', 0.0)))

        if not ts_list:
            return _failed_result(tp_ms=tp_ms, te_ms=te_ms, mem=mem)

        total_ms = tp_ms + te_ms + float(np.mean(ts_list))
        return {
            'success': True,
            'planning_time_ms': float(np.mean(ts_list)),
            'planning_time_std_ms': float(np.std(ts_list)),
            'path_length_m': float(np.mean(lengths)),
            'ns_visited_nodes': int(np.mean(nodes_list)),
            'tp_ms': tp_ms,
            'te_ms': te_ms,
            'topo_ms': None,
            'memory_usage_bytes': mem,
            'total_time_ms': total_ms,
            'trials_count': len(ts_list),
        }

    def run_topo_row(self, row: dict, occupied_list: list, voxel_centers: np.ndarray,
                     tp_ms: float, topo_ms: float, te_ms: float, mem: int,
                     num_trials: int) -> dict:
        start = np.array([float(row['start_x']), float(row['start_y']), float(row['start_z'])])
        goal = np.array([float(row['goal_x']), float(row['goal_y']), float(row['goal_z'])])

        ts_list, nodes_list, lengths = [], [], []
        for i in range(num_trials):
            r = self.plan_topo(start, goal)
            status = (f"✓ Ts={r['ts_ms']:.2f}ms | {r.get('path_length_m', 0):.2f}m"
                      if r['success'] else f"✗ {r.get('error', '?')}")
            print(f'    [{i+1:3d}/{num_trials}] {status}')
            if r['success']:
                ts_list.append(r['ts_ms'])
                nodes_list.append(r['ns_visited_nodes'])
                lengths.append(r['path_length_m'])

        if not ts_list:
            return _failed_result(tp_ms=tp_ms, te_ms=te_ms, topo_ms=topo_ms, mem=mem)

        total_ms = tp_ms + te_ms + topo_ms + float(np.mean(ts_list))
        return {
            'success': True,
            'planning_time_ms': float(np.mean(ts_list)),
            'planning_time_std_ms': float(np.std(ts_list)),
            'path_length_m': float(np.mean(lengths)),
            'ns_visited_nodes': int(np.mean(nodes_list)),
            'tp_ms': tp_ms,
            'te_ms': te_ms,
            'topo_ms': topo_ms,
            'memory_usage_bytes': mem,
            'total_time_ms': total_ms,
            'trials_count': len(ts_list),
        }


def _failed_result(tp_ms=0.0, te_ms=0.0, topo_ms=None, mem=0) -> dict:
    return {
        'success': False,
        'planning_time_ms': None,
        'planning_time_std_ms': None,
        'path_length_m': None,
        'ns_visited_nodes': None,
        'tp_ms': tp_ms,
        'te_ms': te_ms,
        'topo_ms': topo_ms,
        'memory_usage_bytes': mem,
        'total_time_ms': None,
        'trials_count': 0,
    }


# ── Scene cache key ────────────────────────────────────────────────────────────

def _scene_key(row: dict) -> tuple:
    """Rows with same key share a single voxel map load."""
    return (
        row.get('obj_path', '').strip(),
        row.get('scene', ''),
        float(row.get('resolution', 0)),
    )


def _seed_key(row: dict) -> tuple:
    """Rows with same scene key AND seed share a single set_voxel_map call (TopoNav)."""
    return _scene_key(row) + (
        float(row.get('seed_x', 0)),
        float(row.get('seed_y', 0)),
        float(row.get('seed_z', 0)),
    )


# ── CSV I/O ────────────────────────────────────────────────────────────────────

def read_csv(path: str) -> List[dict]:
    with open(path, newline='') as f:
        return list(csv.DictReader(f))


def write_csv(path: str, rows: List[dict]):
    # Collect all columns present
    all_cols = list(dict.fromkeys(
        list(INPUT_COLS) + RESULT_COLS +
        [k for row in rows for k in row.keys()]
    ))
    with open(path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=all_cols, extrasaction='ignore')
        writer.writeheader()
        writer.writerows(rows)


# ── Main ───────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description='自动化 Benchmark（CSV 驱动）')
    parser.add_argument('--csv', required=True, help='输入 benchmark.csv 路径')
    parser.add_argument('--output', default='', help='输出 CSV 路径（默认覆盖输入文件）')
    parser.add_argument('--planner', default='',
                        help='只运行指定 planner (JPS_3D | TopoNav)，不指定则运行全部')
    parser.add_argument('--trials', type=int, default=5,
                        help='每个 start/goal 对运行次数，取均值（默认 5）')
    parser.add_argument('--force', action='store_true',
                        help='强制重跑所有行（包括已有结果的）')
    parser.add_argument('--skip-completed', action='store_true',
                        help='跳过 planning_time_ms 非 None 的行（与 --force 互斥）')

    # TopoNav parameters
    parser.add_argument('--tau', type=float, default=3.0)
    parser.add_argument('--floor-sep', type=float, default=2.5)
    parser.add_argument('--corridor-ratio', type=float, default=3.0)
    parser.add_argument('--stair-var', type=float, default=0.5)
    parser.add_argument('--robot-diameter', type=float, default=0.4)
    parser.add_argument('--z-conn', type=float, default=0.3)
    parser.add_argument('--clearance', type=float, default=1.6)
    parser.add_argument('--dist-max', type=int, default=10)

    parser.add_argument('--service-timeout', type=float, default=10.0)
    parser.add_argument('--map-load-timeout', type=float, default=120.0)

    args = parser.parse_args()

    csv_path = Path(args.csv).expanduser().resolve()
    if not csv_path.exists():
        print(f'[ERROR] CSV 文件不存在: {csv_path}')
        sys.exit(1)

    out_path = Path(args.output).expanduser().resolve() if args.output else csv_path

    rows = read_csv(str(csv_path))
    print(f'[INFO] 读取 {len(rows)} 行 from {csv_path}')

    # Filter rows to process
    to_run = [r for r in rows if _row_needs_run(r, args.force, args.planner or None)]
    print(f'[INFO] 待运行: {len(to_run)} 行  (planner={args.planner or "ALL"}, '
          f'force={args.force}, trials={args.trials})')

    if not to_run:
        print('[INFO] 没有需要运行的行，退出。')
        return

    # Determine which planners are needed
    planners_needed = set(r['planner'].strip() for r in to_run)
    print(f'[INFO] 涉及规划器: {planners_needed}')

    rclpy.init()
    node = AutoBenchmarkNode(
        service_timeout_sec=args.service_timeout,
        map_load_timeout_sec=args.map_load_timeout,
        persistence_tau=args.tau,
        floor_separation_m=args.floor_sep,
        corridor_aspect_ratio=args.corridor_ratio,
        staircase_variance_m=args.stair_var,
        robot_diameter_m=args.robot_diameter,
        connectivity_height=args.z_conn,
        surface_clearance=args.clearance,
        distance_field_max=args.dist_max,
    )
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        # ── Check service availability ────────────────────────────────────────
        if 'JPS_3D' in planners_needed:
            if not node.wait_for_jps_services():
                print('[ERROR] JPS 服务不可用，请先启动 jps_planner_node。')
                return
        if 'TopoNav' in planners_needed:
            if not node.wait_for_topo_services():
                print('[ERROR] TopoNav 服务不可用，请先启动 topo_nav_node。')
                return

        # ── Group rows by scene (for efficient map loading) ───────────────────
        from itertools import groupby
        # Sort to_run by (planner, scene_key) so same scenes are together
        to_run_sorted = sorted(to_run, key=lambda r: (
            r['planner'],
            _scene_key(r),
            float(r.get('seed_x', 0)),
            float(r.get('seed_y', 0)),
            float(r.get('seed_z', 0)),
        ))

        # Build a lookup dict: row index → result
        results_by_idx: Dict[int, dict] = {}

        # Process JPS_3D rows grouped by scene
        jps_rows = [(i, r) for i, r in enumerate(to_run_sorted)
                    if r['planner'].strip() == 'JPS_3D']

        prev_scene_key = None
        occupied_list, voxel_centers, tp_ms = None, None, 0.0

        for orig_idx, row in jps_rows:
            sk = _scene_key(row)
            if sk != prev_scene_key:
                # Load new scene
                obj_path = row.get('obj_path', '').strip()
                scene = row.get('scene', 'unknown')
                resolution = float(row.get('resolution', 0.1))
                print(f'\n[SCENE] 加载场景: scene={scene}, res={resolution}m, '
                      f'obj={obj_path or "PCD"}')
                try:
                    if obj_path and obj_path.lower() != 'none':
                        voxel_centers, occupied_list, tp_ms = _load_voxels_from_obj(
                            obj_path, resolution, [0.0, 0.0, 0.0], node.get_logger())
                    else:
                        voxel_centers, occupied_list, tp_ms = _load_voxels_from_pcd(
                            scene, resolution)
                    print(f'  [SCENE] {len(occupied_list)} 体素, Tp={tp_ms:.1f}ms')
                except Exception as exc:
                    print(f'  [SCENE] 加载失败: {exc}')
                    occupied_list, voxel_centers, tp_ms = None, None, 0.0
                prev_scene_key = sk

            if occupied_list is None:
                results_by_idx[id(row)] = _failed_result()
                continue

            seed = np.array([float(row['seed_x']), float(row['seed_y']),
                             float(row['seed_z'])])
            start = np.array([float(row['start_x']), float(row['start_y']),
                              float(row['start_z'])])
            goal = np.array([float(row['goal_x']), float(row['goal_y']),
                             float(row['goal_z'])])

            print(f'\n[JPS] scene={row["scene"]} | '
                  f'start={start.tolist()} → goal={goal.tolist()}')
            result = node.run_jps_row(row, occupied_list, voxel_centers,
                                      tp_ms, args.trials)
            results_by_idx[id(row)] = result

        # Process TopoNav rows grouped by (scene, seed)
        topo_rows = [(i, r) for i, r in enumerate(to_run_sorted)
                     if r['planner'].strip() == 'TopoNav']

        prev_seed_key = None
        topo_ms, te_ms, topo_mem = 0.0, 0.0, 0

        for orig_idx, row in topo_rows:
            sk = _scene_key(row)
            seed_k = _seed_key(row)

            if sk != prev_scene_key:
                obj_path = row.get('obj_path', '').strip()
                scene = row.get('scene', 'unknown')
                resolution = float(row.get('resolution', 0.1))
                print(f'\n[SCENE] 加载场景: scene={scene}, res={resolution}m, '
                      f'obj={obj_path or "PCD"}')
                try:
                    if obj_path and obj_path.lower() != 'none':
                        voxel_centers, occupied_list, tp_ms = _load_voxels_from_obj(
                            obj_path, resolution, [0.0, 0.0, 0.0], node.get_logger())
                    else:
                        voxel_centers, occupied_list, tp_ms = _load_voxels_from_pcd(
                            scene, resolution)
                    print(f'  [SCENE] {len(occupied_list)} 体素, Tp={tp_ms:.1f}ms')
                except Exception as exc:
                    print(f'  [SCENE] 加载失败: {exc}')
                    occupied_list, voxel_centers, tp_ms = None, None, 0.0
                prev_scene_key = sk
                prev_seed_key = None  # force re-build topo

            if occupied_list is None:
                results_by_idx[id(row)] = _failed_result()
                continue

            if seed_k != prev_seed_key:
                # set_voxel_map + build_topo_map
                seed = np.array([float(row['seed_x']), float(row['seed_y']),
                                 float(row['seed_z'])])
                ok, te_ms, _, topo_mem = node.set_voxel_map(
                    occupied_list, seed, float(row['resolution']))
                if not ok:
                    results_by_idx[id(row)] = _failed_result(tp_ms=tp_ms)
                    continue
                ok2, topo_ms = node.build_topo_map()
                if not ok2:
                    results_by_idx[id(row)] = _failed_result(
                        tp_ms=tp_ms, te_ms=te_ms, mem=topo_mem)
                    continue
                prev_seed_key = seed_k

            start = np.array([float(row['start_x']), float(row['start_y']),
                              float(row['start_z'])])
            goal = np.array([float(row['goal_x']), float(row['goal_y']),
                             float(row['goal_z'])])
            print(f'\n[TOPO] scene={row["scene"]} | '
                  f'start={start.tolist()} → goal={goal.tolist()}')
            result = node.run_topo_row(
                row, occupied_list, voxel_centers,
                tp_ms, topo_ms, te_ms, topo_mem, args.trials)
            results_by_idx[id(row)] = result

        # ── Merge results back into all rows ──────────────────────────────────
        run_set = {id(r): r for r in to_run_sorted}
        for row in rows:
            matching = next((r for r in to_run_sorted if r is row), None)
            if matching is not None and id(matching) in results_by_idx:
                row.update(results_by_idx[id(matching)])

        write_csv(str(out_path), rows)
        print(f'\n[INFO] 结果已写入: {out_path}')

        # ── Summary ───────────────────────────────────────────────────────────
        done = [r for r in results_by_idx.values()]
        n_ok = sum(1 for r in done if r.get('success'))
        print(f'[INFO] 完成 {len(done)} 行: {n_ok} 成功, {len(done)-n_ok} 失败')
        if n_ok:
            times = [r['planning_time_ms'] for r in done
                     if r.get('success') and r.get('planning_time_ms') is not None]
            if times:
                print(f'[INFO] 规划时间 Ts: '
                      f'{np.mean(times):.2f} ± {np.std(times):.2f} ms '
                      f'[{np.min(times):.2f}, {np.max(times):.2f}]')

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
