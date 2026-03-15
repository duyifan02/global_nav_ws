#!/usr/bin/env python3
"""
RViz2 交互式 TopoNav Benchmark 工具

流程:
1. 加载 OBJ/PCD → 发布白色点云到 RViz2
2. 第一次点击: 选择 BFS 种子点 → 建图 + 构建拓扑地图 → 发布房间着色点云
3. 第二次点击: 选择导航起点
4. 第三次点击: 选择导航终点 → 分层 A* 规划
5. 继续点击开始新一轮（起点→终点）

用法:
    # Terminal 1: 启动 topo_nav 节点
    ros2 run topo_nav topo_nav_node

    # Terminal 2: 启动交互工具
    python3 rviz_topo_benchmark.py --obj-path /path/to/scene.obj --resolution 0.1

    # Terminal 3: 启动 RViz2
    rviz2

RViz2 配置:
    1. Fixed Frame: map
    2. Add → PointCloud2 → /topo_benchmark/pointcloud  (原始占据体素, 白色)
    3. Add → PointCloud2 → /topo_benchmark/room_cloud   (房间着色, Color: RGB8)
    4. Add → PointCloud2 → /topo_benchmark/surface       (可通行面, 绿色)
    5. Add → Path          → /topo_benchmark/path
    6. Add → MarkerArray   → /topo_benchmark/markers
    7. 使用工具栏 "Publish Point" 选点 (topic: /clicked_point)
"""

import argparse
import colorsys
import json
import struct
import sys
import time
from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Header
from geometry_msgs.msg import Point, PointStamped, Pose, PoseStamped
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Path as NavPath
from visualization_msgs.msg import Marker, MarkerArray
from nav_interfaces.srv import (
    SetVoxelMap,
    GetDistanceField,
    BuildTopoMap,
    GetRoomLabels,
    PlanHierarchical,
)


class SelectionState(Enum):
    WAITING_SEED = 1
    WAITING_START = 2
    WAITING_GOAL = 3


# ── Colour helpers ────────────────────────────────────────────────────────────

def _room_colour(room_id: int, n_rooms: int):
    """Return (r, g, b) in [0,255] for room_id using golden-angle HSV spacing."""
    if n_rooms <= 1:
        return (100, 200, 255)
    hue = (room_id * 137.508) % 360.0
    r, g, b = colorsys.hsv_to_rgb(hue / 360.0, 0.85, 0.90)
    return (int(r * 255), int(g * 255), int(b * 255))


def _pack_rgb(r, g, b):
    """Pack (r,g,b) into a float for PointCloud2 RGB field."""
    rgb_int = (r << 16) | (g << 8) | b
    return struct.unpack('f', struct.pack('I', rgb_int))[0]


# ── Node ──────────────────────────────────────────────────────────────────────

class RVizTopoBenchmarkNode(Node):
    def __init__(self, scene_name: str, resolution: float, num_trials: int,
                 obj_path: str = '', obj_auto_center: bool = False,
                 obj_origin: Optional[List[float]] = None,
                 persistence_tau: float = 3.0,
                 floor_separation_m: float = 2.5,
                 corridor_aspect_ratio: float = 3.0,
                 staircase_variance_m: float = 0.5,
                 robot_diameter_m: float = 0.4,
                 service_timeout_sec: float = 10.0,
                 map_load_timeout_sec: float = 120.0):
        super().__init__('rviz_topo_benchmark')

        self.scene_name = scene_name
        self.resolution = resolution
        self.num_trials = num_trials
        self.obj_path = obj_path.strip()
        self.obj_auto_center = bool(obj_auto_center)
        self.obj_origin = obj_origin if obj_origin is not None else [0.0, 0.0, 0.0]
        self.service_timeout_sec = float(service_timeout_sec)
        self.map_load_timeout_sec = float(map_load_timeout_sec)

        # Topo-map parameters
        self.persistence_tau = persistence_tau
        self.floor_separation_m = floor_separation_m
        self.corridor_aspect_ratio = corridor_aspect_ratio
        self.staircase_variance_m = staircase_variance_m
        self.robot_diameter_m = robot_diameter_m

        # State
        self.state = SelectionState.WAITING_SEED
        self.seed_point: Optional[np.ndarray] = None
        self.start_point: Optional[np.ndarray] = None
        self.goal_point: Optional[np.ndarray] = None
        self.voxel_centers: Optional[np.ndarray] = None
        self.occupied_list: Optional[List] = None
        self.surface_points: Optional[np.ndarray] = None
        self.room_labels: Optional[np.ndarray] = None
        self._map_memory_usage_bytes: int = 0
        self._preprocessing_time_ms: float = 0.0
        self._traversability_time_ms: float = 0.0
        self._topo_build_time_ms: float = 0.0
        self._topo_info: Dict[str, Any] = {}

        self.all_runs: List[Dict] = []
        self.map_info: Dict[str, Any] = {}

        # Callback group for service calls inside callbacks
        self.cb_group = ReentrantCallbackGroup()
        latching_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        # Publishers
        self.pc_pub = self.create_publisher(PointCloud2, '/topo_benchmark/pointcloud', latching_qos)
        self.room_pub = self.create_publisher(PointCloud2, '/topo_benchmark/room_cloud', latching_qos)
        self.surface_pub = self.create_publisher(PointCloud2, '/topo_benchmark/surface', latching_qos)
        self.path_pub = self.create_publisher(NavPath, '/topo_benchmark/path', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/topo_benchmark/markers', 10)

        # Click subscription
        self.click_sub = self.create_subscription(
            PointStamped, '/clicked_point', self.on_clicked_point, 10,
            callback_group=self.cb_group)

        # Service clients (short names → matches ros2 run topo_nav topo_nav_node)
        self.set_map_client = self.create_client(
            SetVoxelMap, '/set_voxel_map', callback_group=self.cb_group)
        self.get_surface_client = self.create_client(
            GetDistanceField, '/get_distance_field', callback_group=self.cb_group)
        self.build_topo_client = self.create_client(
            BuildTopoMap, '/build_topo_map', callback_group=self.cb_group)
        self.get_labels_client = self.create_client(
            GetRoomLabels, '/get_room_labels', callback_group=self.cb_group)
        self.plan_client = self.create_client(
            PlanHierarchical, '/plan_hierarchical', callback_group=self.cb_group)

        # Wait for topo_nav services
        self.get_logger().info("等待 topo_nav 服务...")
        if not self.set_map_client.wait_for_service(timeout_sec=self.service_timeout_sec):
            raise RuntimeError('set_voxel_map service not available. '
                               'Please start: ros2 run topo_nav topo_nav_node')
        if not self.build_topo_client.wait_for_service(timeout_sec=self.service_timeout_sec):
            raise RuntimeError('build_topo_map service not available')
        if not self.plan_client.wait_for_service(timeout_sec=self.service_timeout_sec):
            raise RuntimeError('plan_hierarchical service not available')
        self.get_logger().info("topo_nav 服务已连接")

        # Load scene
        if self.obj_path:
            self.load_scene_obj()
        else:
            self.load_scene_pcd()
        self.publish_pointcloud()
        self.print_instructions()

    # ── Instructions ──────────────────────────────────────────────────────────

    def print_instructions(self):
        print("\n" + "=" * 70)
        print("RViz2 TopoNav Benchmark 工具 — 持久拓扑分层规划")
        print("=" * 70)
        print(f"场景: {self.scene_name}")
        print(f"分辨率: {self.resolution}m")
        if self.obj_path:
            print(f"模式: OBJ ({self.obj_path})")
        print(f"体素数: {self.map_info.get('num_voxels', 'N/A')}")
        print(f"\n拓扑参数:")
        print(f"  τ (persistence)    = {self.persistence_tau}")
        print(f"  H_floor            = {self.floor_separation_m} m")
        print(f"  γ (corridor ratio) = {self.corridor_aspect_ratio}")
        print(f"  σ_stair            = {self.staircase_variance_m} m")
        print(f"  robot diameter     = {self.robot_diameter_m} m")
        print(f"  trials per run     = {self.num_trials}")
        print(f"\nRViz2 设置:")
        print(f"  1. Fixed Frame: map")
        print(f"  2. Add PointCloud2: /topo_benchmark/pointcloud (白色，原始占据体素)")
        print(f"  3. Add PointCloud2: /topo_benchmark/room_cloud (房间着色，设 Color: RGB8)")
        print(f"  4. Add PointCloud2: /topo_benchmark/surface    (绿色，可通行面)")
        print(f"  5. Add Path:        /topo_benchmark/path")
        print(f"  6. Add MarkerArray: /topo_benchmark/markers")
        print(f"\n操作流程:")
        print(f"  1. 点击选择【BFS 种子点】(蓝色球) — 确定可通行区域")
        print(f"     → 自动建图 + 构建拓扑地图 + 发布房间着色点云")
        print(f"  2. 点击选择【导航起点】(绿色球) — 必须在彩色点云上！")
        print(f"  3. 点击选择【导航终点】(红色球) — 自动分层 A* 规划")
        print(f"  4. 继续点击新的起点/终点进行更多规划")
        print(f"  5. Ctrl+C 退出并保存结果")
        print(f"\n当前状态: 等待选择 BFS 种子点...")
        print("=" * 70)

    # ── Service call helper ───────────────────────────────────────────────────

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
            self.get_logger().error(f'{name} 超时或失败')
        return response

    # ── Scene loading ─────────────────────────────────────────────────────────

    def load_scene_obj(self):
        obj_file = Path(self.obj_path).expanduser().resolve()
        if not obj_file.exists():
            raise RuntimeError(f'OBJ 文件不存在: {obj_file}')

        workspace_root = Path(__file__).resolve().parents[3]
        occmap_pkg = workspace_root / 'src' / 'occmap_tools'
        if str(occmap_pkg) not in sys.path:
            sys.path.insert(0, str(occmap_pkg))

        from occmap_tools.obj_mesh_loader import load_obj_mesh, voxelize_mesh

        origin = tuple(float(v) for v in self.obj_origin)
        t0 = time.time()
        vertices, faces = load_obj_mesh(str(obj_file))
        if vertices.size == 0 or faces.size == 0:
            raise RuntimeError('OBJ 无有效几何数据')

        if self.obj_auto_center:
            mesh_center = vertices.mean(axis=0)
            origin = tuple((-mesh_center).tolist())

        voxel_centers = voxelize_mesh(
            vertices, faces, self.resolution, origin,
            logger=self.get_logger())
        t1 = time.time()

        if voxel_centers.size == 0:
            raise RuntimeError('OBJ 体素化结果为空')

        self.voxel_centers = voxel_centers
        self.occupied_list = [tuple(v) for v in voxel_centers]
        self._preprocessing_time_ms = (t1 - t0) * 1000.0

        self.map_info = {
            'scene': self.scene_name,
            'resolution': self.resolution,
            'source': 'obj_local_voxelization',
            'obj_path': str(obj_file),
            'num_voxels': int(voxel_centers.shape[0]),
        }

        print(f"[LOG] OBJ 加载完成: {voxel_centers.shape[0]} 体素, "
              f"Tp={self._preprocessing_time_ms:.1f}ms")

    def load_scene_pcd(self):
        benchmark_root = Path(__file__).resolve().parent.parent
        if str(benchmark_root) not in sys.path:
            sys.path.insert(0, str(benchmark_root))
        from converters.pcd_to_voxel import get_voxel_map

        t0 = time.time()
        vmap = get_voxel_map(self.scene_name, self.resolution)
        t1 = time.time()

        self.voxel_centers = vmap.voxel_centers
        self.occupied_list = vmap.occupied_list
        self._preprocessing_time_ms = (t1 - t0) * 1000.0

        self.map_info = {
            'scene': self.scene_name,
            'resolution': self.resolution,
            'source': 'pcd',
            'num_voxels': len(self.voxel_centers),
        }
        print(f"[LOG] PCD 加载完成: {len(self.voxel_centers)} 体素, "
              f"Tp={self._preprocessing_time_ms:.1f}ms")

    # ── Point cloud publishing ────────────────────────────────────────────────

    def publish_pointcloud(self):
        """Publish raw occupied voxels as white PointCloud2."""
        pts = self.voxel_centers.astype(np.float32)
        msg = PointCloud2()
        msg.header = Header(frame_id='map', stamp=self.get_clock().now().to_msg())
        msg.height = 1
        msg.width = len(pts)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * len(pts)
        msg.is_dense = True
        msg.data = pts.tobytes()
        self.pc_pub.publish(msg)

    def publish_surface_cloud(self, surface_points: np.ndarray):
        """Publish surface points as green PointCloud2."""
        pts = surface_points.astype(np.float32)
        msg = PointCloud2()
        msg.header = Header(frame_id='map', stamp=self.get_clock().now().to_msg())
        msg.height = 1
        msg.width = len(pts)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * len(pts)
        msg.is_dense = True
        msg.data = pts.tobytes()
        self.surface_pub.publish(msg)

    def publish_room_cloud(self, points: np.ndarray, labels: np.ndarray):
        """Publish room-colored PointCloud2 (XYZRGB)."""
        n = len(points)
        n_rooms = int(labels.max()) + 1 if n > 0 else 1

        data = bytearray()
        for i in range(n):
            x, y, z = float(points[i, 0]), float(points[i, 1]), float(points[i, 2])
            lbl = int(labels[i])
            if lbl >= 0:
                r, g, b = _room_colour(lbl, n_rooms)
            else:
                r, g, b = 60, 60, 60
            data += struct.pack('ffff', x, y, z, _pack_rgb(r, g, b))

        msg = PointCloud2()
        msg.header = Header(frame_id='map', stamp=self.get_clock().now().to_msg())
        msg.height = 1
        msg.width = n
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = 16 * n
        msg.is_dense = True
        msg.data = bytes(data)
        self.room_pub.publish(msg)
        self.get_logger().info(f"房间着色点云已发布 ({n} 点, {n_rooms} 房间)")

    # ── Marker publishing ─────────────────────────────────────────────────────

    def publish_markers(self):
        markers = MarkerArray()
        delete = Marker()
        delete.action = Marker.DELETEALL
        markers.markers.append(delete)

        for label, point, colour, size in [
            ('seed', self.seed_point, (0.0, 0.5, 1.0), 0.6),
            ('start', self.start_point, (0.0, 1.0, 0.0), 0.5),
            ('goal', self.goal_point, (1.0, 0.0, 0.0), 0.5),
        ]:
            if point is not None:
                m = Marker()
                m.header.frame_id = 'map'
                m.header.stamp = self.get_clock().now().to_msg()
                m.ns = 'endpoints'
                m.id = {'seed': 2, 'start': 0, 'goal': 1}[label]
                m.type = Marker.SPHERE
                m.action = Marker.ADD
                m.pose.position.x = float(point[0])
                m.pose.position.y = float(point[1])
                m.pose.position.z = float(point[2])
                m.pose.orientation.w = 1.0
                m.scale.x = m.scale.y = m.scale.z = size
                m.color.r, m.color.g, m.color.b, m.color.a = *colour, 1.0
                markers.markers.append(m)

        self.marker_pub.publish(markers)

    def publish_path(self, path_points):
        msg = NavPath()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        for p in path_points:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = float(p[0])
            ps.pose.position.y = float(p[1])
            ps.pose.position.z = float(p[2])
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.path_pub.publish(msg)

    # ── Point snapping ────────────────────────────────────────────────────────

    def find_nearest_voxel(self, point):
        if self.voxel_centers is not None:
            d = np.linalg.norm(self.voxel_centers - point, axis=1)
            idx = d.argmin()
            return self.voxel_centers[idx], d[idx]
        elif self.surface_points is not None and len(self.surface_points) > 0:
            d = np.linalg.norm(self.surface_points - point, axis=1)
            idx = d.argmin()
            return self.surface_points[idx], d[idx]
        return point, 0.0

    def find_nearest_surface(self, point):
        if self.surface_points is None or len(self.surface_points) == 0:
            return None, None
        d = np.linalg.norm(self.surface_points - point, axis=1)
        idx = d.argmin()
        return self.surface_points[idx], d[idx]

    # ── Click handler (state machine) ─────────────────────────────────────────

    def on_clicked_point(self, msg: PointStamped):
        point = np.array([msg.point.x, msg.point.y, msg.point.z])
        nearest, dist = self.find_nearest_voxel(point)

        if self.state == SelectionState.WAITING_SEED:
            self.seed_point = nearest
            print(f"\n种子点 (点击): [{point[0]:.2f}, {point[1]:.2f}, {point[2]:.2f}]")
            print(f"  → 吸附体素:  [{nearest[0]:.2f}, {nearest[1]:.2f}, {nearest[2]:.2f}]  (偏移 {dist:.2f}m)")
            if dist > 1.0:
                print(f"  ⚠️  偏移 > 1m，请点击白色点云本体！")
            self.publish_markers()

            # Step 1: set_voxel_map with seed
            if not self._do_set_voxel_map(nearest):
                print("建图失败，请重新选择种子点...")
                self.seed_point = None
                return

            # Step 2: build_topo_map
            if not self._do_build_topo_map():
                print("拓扑地图构建失败，请重新选择种子点...")
                self.seed_point = None
                return

            # Step 3: get_room_labels → publish room cloud
            self._do_get_room_labels()

            self.state = SelectionState.WAITING_START
            print("\n等待选择导航起点... (请在彩色点云上选择)")

        elif self.state == SelectionState.WAITING_START:
            _, surface_dist = self.find_nearest_surface(point)
            if surface_dist is not None and surface_dist > self.resolution * 2:
                print(f"\n⚠️  警告: 起点距离可通行面 {surface_dist:.2f}m，请选在彩色点云上！")
            self.start_point = point
            print(f"\n导航起点: [{point[0]:.2f}, {point[1]:.2f}, {point[2]:.2f}]")
            self.publish_markers()
            self.state = SelectionState.WAITING_GOAL
            print("等待选择导航终点...")

        elif self.state == SelectionState.WAITING_GOAL:
            _, surface_dist = self.find_nearest_surface(point)
            if surface_dist is not None and surface_dist > self.resolution * 2:
                print(f"\n⚠️  警告: 终点距离可通行面 {surface_dist:.2f}m，请选在彩色点云上！")
            self.goal_point = point
            print(f"导航终点: [{point[0]:.2f}, {point[1]:.2f}, {point[2]:.2f}]")
            self.publish_markers()

            self.run_benchmark()

            self.state = SelectionState.WAITING_START
            self.start_point = None
            self.goal_point = None
            print("\n等待选择新的导航起点...")

    # ── Step 1: set_voxel_map ─────────────────────────────────────────────────

    def _do_set_voxel_map(self, seed: np.ndarray) -> bool:
        request = SetVoxelMap.Request()
        request.resolution = self.resolution
        request.seed_point.x = float(seed[0])
        request.seed_point.y = float(seed[1])
        request.seed_point.z = float(seed[2])

        print(f"[LOG] 发送 {len(self.occupied_list)} 个体素 + seed 到 topo_nav_node...")
        for v in self.occupied_list:
            p = Point()
            p.x, p.y, p.z = float(v[0]), float(v[1]), float(v[2])
            request.occupied.append(p)

        response = self._call_service(
            self.set_map_client, request, 'set_voxel_map',
            timeout_sec=self.map_load_timeout_sec)

        if response and response.success:
            self._traversability_time_ms = response.traversability_time_ms
            self._map_memory_usage_bytes = int(
                getattr(response, 'map_memory_usage_bytes', 0))
            print(f"[LOG] 建图成功: {response.surface_voxel_count} 可通行体素, "
                  f"Te={self._traversability_time_ms:.1f}ms")

            # Also fetch surface for green cloud
            self._request_surface_async()
            return True
        else:
            msg = response.message if response else "No response"
            print(f"[LOG] 建图失败: {msg}")
            return False

    def _request_surface_async(self):
        if not self.get_surface_client.service_is_ready():
            return
        future = self.get_surface_client.call_async(GetDistanceField.Request())
        future.add_done_callback(self._on_surface_response)

    def _on_surface_response(self, future):
        try:
            resp = future.result()
        except Exception:
            return
        if not resp or not resp.success:
            return
        pts = np.array([[p.x, p.y, p.z] for p in resp.surface_points], dtype=np.float32)
        if len(pts) > 0:
            self.surface_points = pts
            self.publish_surface_cloud(pts)

    # ── Step 2: build_topo_map ────────────────────────────────────────────────

    def _do_build_topo_map(self) -> bool:
        request = BuildTopoMap.Request()
        request.persistence_tau = self.persistence_tau
        request.floor_separation_m = self.floor_separation_m
        request.corridor_aspect_ratio = self.corridor_aspect_ratio
        request.staircase_variance_m = self.staircase_variance_m
        request.robot_diameter_m = self.robot_diameter_m

        print(f"[LOG] 构建拓扑地图 (τ={self.persistence_tau})...")
        response = self._call_service(
            self.build_topo_client, request, 'build_topo_map',
            timeout_sec=self.service_timeout_sec)

        if response and response.success:
            self._topo_build_time_ms = response.build_time_ms
            self._topo_info = {
                'num_rooms': response.num_rooms,
                'num_floors': response.num_floors,
                'num_staircases': response.num_staircases,
                'num_corridors': response.num_corridors,
            }
            print(f"[LOG] 拓扑地图构建成功: "
                  f"{response.num_rooms} 房间, {response.num_floors} 楼层, "
                  f"{response.num_staircases} 楼梯间, {response.num_corridors} 走廊, "
                  f"{response.build_time_ms:.1f}ms")
            return True
        else:
            msg = response.message if response else "No response"
            print(f"[LOG] 拓扑地图构建失败: {msg}")
            return False

    # ── Step 3: get_room_labels → visualize ───────────────────────────────────

    def _do_get_room_labels(self):
        response = self._call_service(
            self.get_labels_client, GetRoomLabels.Request(), 'get_room_labels')

        if not response or not response.success:
            print("[LOG] 获取房间标签失败")
            return

        pts = np.array([[p.x, p.y, p.z] for p in response.surface_points],
                       dtype=np.float32)
        labels = np.array(response.room_labels, dtype=np.int32)

        self.surface_points = pts
        self.room_labels = labels
        self.publish_room_cloud(pts, labels)

    # ── Benchmark run ─────────────────────────────────────────────────────────

    def run_benchmark(self):
        euclidean_dist = np.linalg.norm(self.goal_point - self.start_point)

        tp_ms = self._preprocessing_time_ms
        te_ms = self._traversability_time_ms
        topo_ms = self._topo_build_time_ms

        print(f"\n{'=' * 60}")
        print(f"运行 {self.num_trials} 次分层规划测试")
        print(f"起点: {self.start_point}")
        print(f"终点: {self.goal_point}")
        print(f"直线距离: {euclidean_dist:.2f}m")
        print(f"Tp={tp_ms:.1f}ms | Te={te_ms:.1f}ms | T_topo={topo_ms:.1f}ms")
        print(f"{'=' * 60}")

        trials = []
        last_path = None

        for i in range(self.num_trials):
            result = self._call_plan_hierarchical()

            total_ms = tp_ms + te_ms + topo_ms + result['ts_ms']
            result['tp_ms'] = tp_ms
            result['te_ms'] = te_ms
            result['topo_ms'] = topo_ms
            result['total_ms'] = total_ms
            trials.append(result)

            if result['success']:
                last_path = result['path']
                self.publish_path(last_path)
                rooms = result.get('room_sequence', [])
                floors = result.get('floor_sequence', [])
                status = (f"✓ Ts={result['ts_ms']:.2f}ms | "
                          f"{result['path_length_m']:.2f}m | "
                          f"Ns={result['voxel_nodes']} | "
                          f"rooms={rooms} floors={floors}")
            else:
                status = f"✗ {result['error']}"

            print(f"  [{i+1:3d}/{self.num_trials}] {status}")

        run_data = {
            'start': self.start_point.tolist(),
            'goal': self.goal_point.tolist(),
            'euclidean_distance': euclidean_dist,
            'topo_info': dict(self._topo_info),
            'trials': trials,
        }
        self.all_runs.append(run_data)

        # Print stats
        successful = [t for t in trials if t['success']]
        print(f"\n{'=' * 60}")
        print(f"本轮统计:")
        print(f"  成功率: {len(successful)}/{len(trials)}")
        if successful:
            ts_vals = [t['ts_ms'] for t in successful]
            lens = [t['path_length_m'] for t in successful]
            nodes = [t['voxel_nodes'] for t in successful]
            floor_ms = [t.get('floor_ms', 0) for t in successful]
            room_ms = [t.get('room_ms', 0) for t in successful]
            voxel_ms = [t.get('voxel_ms', 0) for t in successful]
            print(f"  [Tp]    点云预处理:     {tp_ms:.1f} ms")
            print(f"  [Te]    遍历面提取:     {te_ms:.1f} ms")
            print(f"  [Ttopo] 拓扑地图构建:   {topo_ms:.1f} ms")
            print(f"  [Ts]    分层规划总时间:  {np.mean(ts_vals):.2f} ± {np.std(ts_vals):.2f} ms")
            print(f"     楼层 A*: {np.mean(floor_ms):.3f} ms")
            print(f"     房间 A*: {np.mean(room_ms):.3f} ms")
            print(f"     体素 A*: {np.mean(voxel_ms):.2f} ms")
            print(f"  [Tall]  总时间:         {tp_ms + te_ms + topo_ms + np.mean(ts_vals):.1f} ms")
            print(f"  路径长度: {np.mean(lens):.2f} ± {np.std(lens):.2f} m")
            print(f"  体素节点展开数: {np.mean(nodes):.0f}")
        print(f"{'=' * 60}")

    # ── Plan call ─────────────────────────────────────────────────────────────

    def _call_plan_hierarchical(self) -> Dict[str, Any]:
        request = PlanHierarchical.Request()
        request.start.position.x = float(self.start_point[0])
        request.start.position.y = float(self.start_point[1])
        request.start.position.z = float(self.start_point[2])
        request.start.orientation.w = 1.0
        request.goal.position.x = float(self.goal_point[0])
        request.goal.position.y = float(self.goal_point[1])
        request.goal.position.z = float(self.goal_point[2])
        request.goal.orientation.w = 1.0
        request.snap_to_surface = True
        request.robot_diameter_m = self.robot_diameter_m

        response = self._call_service(
            self.plan_client, request, 'plan_hierarchical')

        if response is None:
            return {'success': False, 'error': 'Service timeout',
                    'ts_ms': 0.0, 'voxel_nodes': 0, 'path_length_m': 0.0}

        if response.success:
            path = np.array([[p.pose.position.x, p.pose.position.y,
                              p.pose.position.z]
                             for p in response.path.poses])
            length = (np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1))
                      if len(path) > 1 else 0.0)
            return {
                'success': True,
                'error': None,
                'ts_ms': response.planning_time_ms,
                'floor_ms': response.floor_planning_ms,
                'room_ms': response.room_planning_ms,
                'voxel_ms': response.voxel_planning_ms,
                'path_length_m': length,
                'num_waypoints': len(path),
                'voxel_nodes': int(response.voxel_nodes_expanded),
                'room_sequence': list(response.room_sequence),
                'floor_sequence': list(response.floor_sequence),
                'path': path,
            }
        else:
            return {
                'success': False,
                'error': response.message,
                'ts_ms': response.planning_time_ms,
                'voxel_nodes': int(response.voxel_nodes_expanded),
                'path_length_m': 0.0,
            }

    # ── Save results ──────────────────────────────────────────────────────────

    def save_results(self) -> Optional[str]:
        if not self.all_runs:
            return None

        output_dir = Path(__file__).parent.parent / 'results'
        output_dir.mkdir(exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = output_dir / f"topo_benchmark_{self.scene_name}_{timestamp}.json"

        data = {
            'planner': 'TOPO_NAV_HIERARCHICAL',
            'timestamp': timestamp,
            'map_info': self.map_info,
            'seed_point': self.seed_point.tolist() if self.seed_point is not None else None,
            'topo_params': {
                'persistence_tau': self.persistence_tau,
                'floor_separation_m': self.floor_separation_m,
                'corridor_aspect_ratio': self.corridor_aspect_ratio,
                'staircase_variance_m': self.staircase_variance_m,
                'robot_diameter_m': self.robot_diameter_m,
            },
            'topo_info': dict(self._topo_info),
            'timing': {
                'tp_ms': self._preprocessing_time_ms,
                'te_ms': self._traversability_time_ms,
                'topo_build_ms': self._topo_build_time_ms,
            },
            'config': {
                'trials_per_run': self.num_trials,
                'snap_to_surface': True,
            },
            'runs': self.all_runs,
        }

        with open(filename, 'w') as f:
            json.dump(data, f, indent=2, default=float)

        return str(filename)


# ── main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description='RViz2 交互式 TopoNav Benchmark')
    parser.add_argument('--scene', default='custom',
                        help='场景名（仅用于结果命名）')
    parser.add_argument('--resolution', type=float, default=0.1)
    parser.add_argument('--trials', type=int, default=1,
                        help='每组起终点测试次数')

    # Map source
    parser.add_argument('--obj-path', type=str, default='',
                        help='OBJ 文件路径')
    parser.add_argument('--obj-auto-center', action='store_true',
                        help='自动居中 OBJ mesh')
    parser.add_argument('--obj-origin', nargs=3, type=float,
                        default=[0.0, 0.0, 0.0],
                        help='OBJ 体素化 origin (x y z)')

    # Topo parameters
    parser.add_argument('--tau', type=float, default=3.0,
                        help='持久性阈值 τ (体素单位)')
    parser.add_argument('--floor-sep', type=float, default=2.5,
                        help='楼层间距阈值 H_floor (m)')
    parser.add_argument('--corridor-ratio', type=float, default=3.0,
                        help='走廊长宽比阈值 γ')
    parser.add_argument('--stair-var', type=float, default=0.5,
                        help='楼梯高度标准差 σ_stair (m)')
    parser.add_argument('--robot-diameter', type=float, default=0.4,
                        help='机器人直径 (m)')

    # Timeouts
    parser.add_argument('--service-timeout', type=float, default=10.0)
    parser.add_argument('--map-load-timeout', type=float, default=120.0)

    args = parser.parse_args()
    rclpy.init()

    node = RVizTopoBenchmarkNode(
        scene_name=args.scene,
        resolution=args.resolution,
        num_trials=args.trials,
        obj_path=args.obj_path,
        obj_auto_center=args.obj_auto_center,
        obj_origin=args.obj_origin,
        persistence_tau=args.tau,
        floor_separation_m=args.floor_sep,
        corridor_aspect_ratio=args.corridor_ratio,
        staircase_variance_m=args.stair_var,
        robot_diameter_m=args.robot_diameter,
        service_timeout_sec=args.service_timeout,
        map_load_timeout_sec=args.map_load_timeout,
    )

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        print("\n\n中断...")
    finally:
        print("\n" + "=" * 70)
        print("最终统计:")
        print("=" * 70)

        if node.all_runs:
            total_trials = sum(len(r['trials']) for r in node.all_runs)
            total_ok = sum(
                sum(1 for t in r['trials'] if t['success'])
                for r in node.all_runs)
            print(f"总测试组数: {len(node.all_runs)}")
            print(f"总测试次数: {total_trials}")
            print(f"总成功率: {total_ok}/{total_trials}")

            all_ok = [t for r in node.all_runs for t in r['trials']
                      if t['success']]
            if all_ok:
                ts = [t['ts_ms'] for t in all_ok]
                lens = [t['path_length_m'] for t in all_ok]
                print(f"  [Ts] 分层规划: {np.mean(ts):.2f} ± {np.std(ts):.2f} ms")
                print(f"  路径长度: {np.mean(lens):.2f} ± {np.std(lens):.2f} m")

        filename = node.save_results()
        if filename:
            print(f"\n结果已保存到: {filename}")

        node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
