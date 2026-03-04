#!/usr/bin/env python3
"""
RViz2交互式Benchmark工具

流程:
1. 发布点云到RViz2
2. 第一次点击: 选择建图种子点（BFS扩展起点）
3. 第二次点击: 选择导航起点
4. 第三次点击: 选择导航终点 → 自动规划
5. 继续点击开始新一轮（起点→终点）

用法:
    ros2 run jps_planner_3d jps_planner_node  # 先启动planner节点
    python3 rviz_benchmark.py --scene spiral --trials 100

RViz2配置:
    1. Add -> PointCloud2 -> topic: /benchmark/pointcloud
    2. Add -> Path -> topic: /benchmark/path
    3. Add -> Marker -> topic: /benchmark/markers
    4. 使用工具栏 "Publish Point" 选点 (topic: /clicked_point)
"""

import argparse
import sys
import time
import json
import threading
from pathlib import Path
from datetime import datetime
from dataclasses import dataclass, field
from typing import Optional, List, Dict, Any
from enum import Enum

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Header
from geometry_msgs.msg import Point, PointStamped, Pose
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Path as NavPath
from visualization_msgs.msg import Marker, MarkerArray
from nav_interfaces.srv import SetVoxelMap, PlanPath, GetDistanceField


class SelectionState(Enum):
    WAITING_SEED = 1
    WAITING_START = 2
    WAITING_GOAL = 3


@dataclass 
class TrialResult:
    success: bool
    planning_time_ms: float  # Ts: path searching time
    expanded_nodes: int  # Ns: visited graph nodes
    path_length_m: float
    num_waypoints: int
    preprocessing_time_ms: float = 0.0  # Tp: point cloud pre-processing time
    memory_usage_bytes: int = 0  # Size: memory usage of scene representation
    traversability_time_ms: float = 0.0  # Te: traversability estimation time
    optimization_time_ms: float = 0.0  # To: trajectory optimization time
    total_time_ms: float = 0.0  # Tall: total navigation time
    error: Optional[str] = None


@dataclass
class RunResult:
    start: List[float]
    goal: List[float]
    euclidean_distance: float
    trials: List[TrialResult] = field(default_factory=list)
    
    @property
    def success_count(self) -> int:
        return sum(1 for t in self.trials if t.success)
    
    @property
    def success_rate(self) -> float:
        return self.success_count / len(self.trials) if self.trials else 0.0
    
    @property
    def successful_trials(self) -> List[TrialResult]:
        return [t for t in self.trials if t.success]
    
    def get_stats(self) -> Dict[str, Any]:
        successful = self.successful_trials
        if not successful:
            return {
                'success_rate': 0.0,
                'planning_time_ms': {'mean': 0, 'std': 0, 'min': 0, 'max': 0},
                'path_length_m': {'mean': 0, 'std': 0, 'min': 0, 'max': 0},
                'expanded_nodes': {'mean': 0, 'std': 0, 'min': 0, 'max': 0},
            }
        
        times = [t.planning_time_ms for t in successful]
        lengths = [t.path_length_m for t in successful]
        nodes = [t.expanded_nodes for t in successful]
        total_times = [t.total_time_ms for t in successful]
        
        return {
            'success_rate': self.success_rate,
            'planning_time_ms': {
                'mean': np.mean(times),
                'std': np.std(times),
                'min': np.min(times),
                'max': np.max(times),
            },
            'path_length_m': {
                'mean': np.mean(lengths),
                'std': np.std(lengths),
                'min': np.min(lengths),
                'max': np.max(lengths),
            },
            'expanded_nodes': {
                'mean': np.mean(nodes),
                'std': np.std(nodes),
                'min': int(np.min(nodes)),
                'max': int(np.max(nodes)),
            },
            'total_time_ms': {
                'mean': np.mean(total_times),
                'std': np.std(total_times),
                'min': np.min(total_times),
                'max': np.max(total_times),
            },
        }


class RVizBenchmarkNode(Node):
    def __init__(self, scene_name: str, resolution: float, num_trials: int,
                 preloaded: bool = False):
        super().__init__('rviz_benchmark')

        self.scene_name = scene_name
        self.resolution = resolution
        self.num_trials = num_trials
        self.preloaded = preloaded  # True: 地图已由外部节点加载（如 obj_mesh_loader）

        # preloaded 模式跳过种子选择，直接等待起点
        self.state = SelectionState.WAITING_START if preloaded else SelectionState.WAITING_SEED
        self.seed_point: Optional[np.ndarray] = None
        self.start_point: Optional[np.ndarray] = None
        self.goal_point: Optional[np.ndarray] = None
        self.voxel_centers: Optional[np.ndarray] = None   # PCD 模式下有值
        self.occupied_list: Optional[List] = None
        self.surface_points: Optional[np.ndarray] = None  # 可通行区域，用于检查选点

        self.all_runs: List[RunResult] = []
        self.map_info: Dict[str, Any] = {}

        # 用于异步处理的锁和状态
        self._processing = False
        self._pending_action = None

        # 使用 ReentrantCallbackGroup 允许嵌套回调
        self.cb_group = ReentrantCallbackGroup()

        latching_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.pc_pub = self.create_publisher(PointCloud2, '/benchmark/pointcloud', latching_qos)
        self.surface_pub = self.create_publisher(PointCloud2, '/benchmark/surface', latching_qos)
        self.path_pub = self.create_publisher(NavPath, '/benchmark/path', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/benchmark/markers', 10)

        self.click_sub = self.create_subscription(
            PointStamped, '/clicked_point', self.on_clicked_point, 10,
            callback_group=self.cb_group)

        self.set_map_client = self.create_client(SetVoxelMap, '/set_voxel_map', callback_group=self.cb_group)
        self.plan_client = self.create_client(PlanPath, '/plan_path', callback_group=self.cb_group)
        self.get_surface_client = self.create_client(GetDistanceField, '/get_distance_field', callback_group=self.cb_group)

        self.get_logger().info("等待JPS服务...")
        self.set_map_client.wait_for_service(timeout_sec=10.0)
        self.plan_client.wait_for_service(timeout_sec=10.0)
        self.get_logger().info("JPS服务已连接")

        if preloaded:
            # 地图已由外部加载（obj_mesh_loader），直接拉取 surface 发布到 RViz
            self.get_surface_client.wait_for_service(timeout_sec=5.0)
            self.map_info = {'scene': scene_name, 'resolution': resolution,
                             'source': 'preloaded (obj_mesh_loader)'}
            self.publish_surface()
        else:
            self.load_scene()
            self.publish_pointcloud()

        self.print_instructions()
    
    def print_instructions(self):
        print("\n" + "="*70)
        print("RViz2 Benchmark 工具 - JPS 3D Planner")
        print("="*70)
        print(f"场景: {self.scene_name}")
        print(f"分辨率: {self.resolution}m")
        if self.preloaded:
            print("模式: Preloaded (地图已由外部节点加载，如 obj_mesh_loader)")
        else:
            print(f"体素数: {self.map_info.get('num_voxels', 'N/A')}")
        print(f"每组测试次数: {self.num_trials}")
        print()
        print("RViz2设置:")
        print("  1. Fixed Frame: map")
        if not self.preloaded:
            print("  2. Add PointCloud2: /benchmark/pointcloud (原始点云，白色)")
        print("  2. Add PointCloud2: /benchmark/surface (可通行区域，绿色)")
        print("  3. Add Path: /benchmark/path")
        print("  4. Add MarkerArray: /benchmark/markers")
        print()
        print("重要提示:")
        print("  ⚠️  起点和终点必须选在【绿色 surface 点云】上！")
        print("  ⚠️  否则会规划失败 (Planning was cancelled)")
        print()
        print("操作流程:")
        if self.preloaded:
            print("  1. 点击选择【导航起点】(绿色) - 必须在绿色点云上！")
            print("  2. 点击选择【导航终点】(红色) - 必须在绿色点云上！")
            print("  3. 继续点击选择新的起点/终点")
            print("  4. Ctrl+C 退出并保存结果")
            print()
            print("当前状态: 等待选择导航起点...")
        else:
            print("  1. 点击选择【建图种子点】(蓝色) - 确定可行走区域")
            print("  2. 点击选择【导航起点】(绿色) - 必须在绿色点云上！")
            print("  3. 点击选择【导航终点】(红色) - 必须在绿色点云上！")
            print("  4. 继续点击选择新的起点/终点")
            print("  5. Ctrl+C 退出并保存结果")
            print()
            print("当前状态: 等待选择建图种子点...")
        print("="*70)
    
    def load_scene(self):
        import time as _time
        sys.path.insert(0, str(Path(__file__).parent.parent))
        from converters.pcd_to_voxel import get_voxel_map
        
        self.get_logger().info(f"加载场景 {self.scene_name}...")
        print(f"[LOG] === 加载场景: {self.scene_name} ===")
        
        t0 = _time.time()
        vmap = get_voxel_map(self.scene_name, self.resolution)
        t1 = _time.time()
        
        self.voxel_centers = vmap.voxel_centers
        self.occupied_list = vmap.occupied_list
        
        # 记录点云预处理时间 (Tp)
        self._preprocessing_time_ms = (t1 - t0) * 1000.0
        
        print(f"[LOG] PCD文件: {vmap.pcd_path}")
        print(f"[LOG] 原始点云数: {len(vmap.points)}")
        print(f"[LOG] 体素化后体素数: {len(self.voxel_centers)}")
        print(f"[LOG] 体素化耗时 (Tp): {self._preprocessing_time_ms:.2f}ms")
        print(f"[LOG] 边界: {vmap.bounds_min} -> {vmap.bounds_max}")
        
        self.map_info = {
            'scene': self.scene_name,
            'resolution': self.resolution,
            'num_voxels': len(self.voxel_centers),
            'bounds_min': vmap.bounds_min.tolist(),
            'bounds_max': vmap.bounds_max.tolist(),
        }
        
        self.get_logger().info(f"体素数: {len(self.voxel_centers)}")
    
    def set_jps_map(self, seed_point: np.ndarray):
        import time as _time
        
        request = SetVoxelMap.Request()
        request.resolution = self.resolution
        
        request.seed_point.x = float(seed_point[0])
        request.seed_point.y = float(seed_point[1])
        request.seed_point.z = float(seed_point[2])
        
        print(f"[LOG] 准备发送 {len(self.occupied_list)} 个体素到 JPS...")
        print(f"[LOG] 种子点: ({seed_point[0]:.2f}, {seed_point[1]:.2f}, {seed_point[2]:.2f})")
        print(f"[LOG] 分辨率: {self.resolution}m")
        
        t0 = _time.time()
        for v in self.occupied_list:
            p = Point()
            p.x, p.y, p.z = float(v[0]), float(v[1]), float(v[2])
            request.occupied.append(p)
        t1 = _time.time()
        print(f"[LOG] 构建请求耗时: {t1-t0:.2f}s")
        
        print("[LOG] 正在调用 set_voxel_map 服务...")
        t2 = _time.time()
        
        # 同步调用服务
        response = self.set_map_client.call(request)
        
        t3 = _time.time()
        print(f"[LOG] 服务调用耗时: {t3-t2:.2f}s")
        
        if response and response.success:
            surface_count = response.surface_voxel_count
            # 保存可通行性估计时间 (Te)
            self._traversability_time_ms = response.traversability_time_ms
            self.get_logger().info(f"JPS地图设置成功，可行走表面体素: {surface_count}, Te: {self._traversability_time_ms:.2f}ms")
            print(f"[LOG] 建图完成！可行走表面体素: {surface_count}")
            print(f"[LOG] 可通行性估计时间 (Te): {self._traversability_time_ms:.2f}ms")
            print("[LOG] 正在获取并发布 surface...")
            t4 = _time.time()
            self.publish_surface()
            t5 = _time.time()
            print(f"[LOG] 发布 surface 耗时: {t5-t4:.2f}s")
            return True
        else:
            msg = response.message if response else "No response"
            self.get_logger().error(f"JPS地图设置失败: {msg}")
            print(f"[LOG] 建图失败: {msg}")
            return False
    
    def publish_surface(self):
        response = self.get_surface_client.call(GetDistanceField.Request())
        
        if not response or not response.success:
            self.get_logger().warn("无法获取可通行区域")
            return
        
        surface_points = np.array([[p.x, p.y, p.z] for p in response.surface_points], dtype=np.float32)
        self.surface_points = surface_points  # 保存用于后续检查
        
        if len(surface_points) == 0:
            self.get_logger().warn("可通行区域为空")
            return
        
        msg = PointCloud2()
        msg.header = Header()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        
        msg.height = 1
        msg.width = len(surface_points)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * len(surface_points)
        msg.is_dense = True
        msg.data = surface_points.tobytes()
        
        self.surface_pub.publish(msg)
        self.get_logger().info(f"可通行区域已发布到 /benchmark/surface ({len(surface_points)} 点)")
    
    def publish_pointcloud(self):
        points = self.voxel_centers
        
        msg = PointCloud2()
        msg.header = Header()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        
        msg.height = 1
        msg.width = len(points)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * len(points)
        msg.is_dense = True
        msg.data = points.astype(np.float32).tobytes()
        
        self.pc_pub.publish(msg)
        self.get_logger().info("点云已发布到 /benchmark/pointcloud")
    
    def on_clicked_point(self, msg: PointStamped):
        point = np.array([msg.point.x, msg.point.y, msg.point.z])
        nearest, dist = self.find_nearest_voxel(point)
        
        if self.state == SelectionState.WAITING_SEED:
            # 吸附到最近占据体素，修正 RViz 地板点击导致的 z≈0 问题
            self.seed_point = nearest
            print(f"\n建图种子点 (原始点击): [{point[0]:.2f}, {point[1]:.2f}, {point[2]:.2f}]")
            print(f"  → 吸附到最近体素:      [{nearest[0]:.2f}, {nearest[1]:.2f}, {nearest[2]:.2f}]  (偏移 {dist:.2f}m)")
            if dist > 1.0:
                print(f"  ⚠️  偏移 > 1m，请点击白色点云本体（道路表面），而非 RViz 地板！")
            self.publish_markers()

            if self.set_jps_map(nearest):
                self.state = SelectionState.WAITING_START
                print("\n等待选择导航起点...")
            else:
                print("\n建图失败，请重新选择种子点...")
                self.seed_point = None
        
        elif self.state == SelectionState.WAITING_START:
            # 检查是否在 surface 上
            _, surface_dist = self.find_nearest_surface(point)
            if surface_dist > self.resolution * 2:
                print(f"\n⚠️  警告: 起点距离可通行区域 {surface_dist:.2f}m")
                print("    请选在【绿色 surface】上的点！")
                print(f"    将在 {self.resolution*2:.2f}m 范围内自动吸附")
            
            self.start_point = point
            print(f"\n导航起点: [{point[0]:.2f}, {point[1]:.2f}, {point[2]:.2f}]")
            print(f"  最近体素距离: {dist:.3f}m")
            self.publish_markers()
            self.state = SelectionState.WAITING_GOAL
            print("等待选择导航终点...")

        elif self.state == SelectionState.WAITING_GOAL:
            # 检查是否在 surface 上
            _, surface_dist = self.find_nearest_surface(point)
            if surface_dist > self.resolution * 2:
                print(f"\n⚠️  警告: 终点距离可通行区域 {surface_dist:.2f}m")
                print("    请选在【绿色 surface】上的点！")
                print(f"    将在 {self.resolution*2:.2f}m 范围内自动吸附")
            
            self.goal_point = point
            print(f"导航终点: [{point[0]:.2f}, {point[1]:.2f}, {point[2]:.2f}]")
            print(f"  最近体素距离: {dist:.3f}m")
            self.publish_markers()

            self.run_benchmark()
            
            self.state = SelectionState.WAITING_START
            self.start_point = None
            self.goal_point = None
            print("\n等待选择新的导航起点...")
    
    def find_nearest_voxel(self, point):
        """找到离点最近的体素中心及距离。preloaded 模式下 voxel_centers 为 None，退而使用 surface_points。"""
        if self.voxel_centers is not None:
            distances = np.linalg.norm(self.voxel_centers - point, axis=1)
            idx = distances.argmin()
            return self.voxel_centers[idx], distances[idx]
        elif self.surface_points is not None and len(self.surface_points) > 0:
            distances = np.linalg.norm(self.surface_points - point, axis=1)
            idx = distances.argmin()
            return self.surface_points[idx], distances[idx]
        return point, 0.0
    
    def find_nearest_surface(self, point):
        """找到离点最近的 surface 点及距离"""
        if self.surface_points is None or len(self.surface_points) == 0:
            return None, float('inf')
        distances = np.linalg.norm(self.surface_points - point, axis=1)
        idx = distances.argmin()
        return self.surface_points[idx], distances[idx]
    
    def publish_markers(self):
        markers = MarkerArray()
        
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        markers.markers.append(delete_marker)
        
        if self.seed_point is not None:
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'endpoints'
            m.id = 2
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(self.seed_point[0])
            m.pose.position.y = float(self.seed_point[1])
            m.pose.position.z = float(self.seed_point[2])
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.6
            m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 0.5, 1.0, 1.0
            markers.markers.append(m)
        
        if self.start_point is not None:
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'endpoints'
            m.id = 0
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(self.start_point[0])
            m.pose.position.y = float(self.start_point[1])
            m.pose.position.z = float(self.start_point[2])
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.5
            m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.0, 1.0
            markers.markers.append(m)
        
        if self.goal_point is not None:
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'endpoints'
            m.id = 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(self.goal_point[0])
            m.pose.position.y = float(self.goal_point[1])
            m.pose.position.z = float(self.goal_point[2])
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.5
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 0.0, 1.0
            markers.markers.append(m)
        
        self.marker_pub.publish(markers)
    
    def run_benchmark(self):
        import time
        euclidean_dist = np.linalg.norm(self.goal_point - self.start_point)
        
        run_result = RunResult(
            start=self.start_point.tolist(),
            goal=self.goal_point.tolist(),
            euclidean_distance=euclidean_dist,
        )
        
        # 点云预处理时间 (Tp) - 加载场景的时间
        preprocessing_time_ms = getattr(self, '_preprocessing_time_ms', 0.0)
        
        # 可通行性估计时间 (Te) - buildReachableMap 的时间
        traversability_time_ms = getattr(self, '_traversability_time_ms', 0.0)
        
        # 内存使用估算 (Size) - 每个体素约 1 byte (surface_array)
        memory_usage_bytes = len(self.occupied_list) if self.occupied_list else 0
        
        print(f"\n{'='*60}")
        print(f"运行 {self.num_trials} 次规划测试")
        print(f"起点: {self.start_point}")
        print(f"终点: {self.goal_point}")
        print(f"直线距离: {euclidean_dist:.2f}m")
        print(f"Tp (预处理): {preprocessing_time_ms:.2f}ms | Te (可通行性): {traversability_time_ms:.2f}ms")
        print(f"Size (内存): {memory_usage_bytes} bytes")
        print(f"{'='*60}")
        
        last_path = None
        
        for i in range(self.num_trials):
            result = self.call_plan_service()

            # Tall = Tp + Te + Ts + To (纯算法总时间，不含ROS通信)
            total_time_ms = preprocessing_time_ms + traversability_time_ms + result['planning_time_ms'] + 0.0

            trial = TrialResult(
                success=result['success'],
                planning_time_ms=result['planning_time_ms'],  # Ts
                expanded_nodes=result['expanded_nodes'],  # Ns
                path_length_m=result.get('path_length_m', 0.0),
                num_waypoints=result.get('num_waypoints', 0),
                preprocessing_time_ms=preprocessing_time_ms,  # Tp
                memory_usage_bytes=memory_usage_bytes,  # Size
                traversability_time_ms=traversability_time_ms,  # Te
                optimization_time_ms=0.0,  # To (当前无轨迹优化)
                total_time_ms=total_time_ms,  # Tall = Tp + Te + Ts + To
                error=result.get('error'),
            )
            run_result.trials.append(trial)
            
            if result['success']:
                last_path = result['path']
                status = f"✓ {result['planning_time_ms']:.2f}ms | {result['path_length_m']:.2f}m | {result['expanded_nodes']} nodes"
                self.publish_path(last_path)
            else:
                status = f"✗ {result['error']}"
            
            print(f"  [{i+1:3d}/{self.num_trials}] {status}")
        
        if last_path is not None:
            self.publish_path(last_path)
        
        self.all_runs.append(run_result)
        
        stats = run_result.get_stats()
        print(f"\n{'='*60}")
        print("本轮统计 (纯算法耗时，不含ROS通信):")
        print(f"  成功率: {stats['success_rate']*100:.1f}%")
        if stats['success_rate'] > 0:
            print(f"  [Tp] 预处理时间: {preprocessing_time_ms:.2f} ms")
            print(f"  [Te] 可通行性估计: {traversability_time_ms:.2f} ms")
            print(f"  [Ts] 路径搜索时间: {stats['planning_time_ms']['mean']:.2f} ± {stats['planning_time_ms']['std']:.2f} ms")
            print(f"  [To] 轨迹优化时间: 0.00 ms (未启用)")
            print(f"  [Tall] 总导航时间: {stats.get('total_time_ms', {}).get('mean', 0):.2f} ms")
            print(f"  [Size] 内存使用: {memory_usage_bytes} bytes")
            print(f"  [Ns] 扩展节点: {stats['expanded_nodes']['mean']:.0f} ± {stats['expanded_nodes']['std']:.0f}")
            print(f"  路径长度: {stats['path_length_m']['mean']:.2f} ± {stats['path_length_m']['std']:.2f} m")
        
        self.print_cumulative_stats()
    
    def print_cumulative_stats(self):
        if len(self.all_runs) < 2:
            return
        
        all_successful = []
        total_trials = 0
        total_successes = 0
        
        for run in self.all_runs:
            total_trials += len(run.trials)
            total_successes += run.success_count
            all_successful.extend(run.successful_trials)
        
        print(f"\n累计统计 ({len(self.all_runs)} 组, {total_trials} 次):")
        print(f"  总成功率: {total_successes/total_trials*100:.1f}%")
        
        if all_successful:
            times = [t.planning_time_ms for t in all_successful]
            lengths = [t.path_length_m for t in all_successful]
            nodes = [t.expanded_nodes for t in all_successful]
            total_times = [t.total_time_ms for t in all_successful]
            tps = [t.preprocessing_time_ms for t in all_successful]
            tes = [t.traversability_time_ms for t in all_successful]
            sizes = [t.memory_usage_bytes for t in all_successful]
            print(f"  [Tp] 预处理时间: {np.mean(tps):.2f} ms")
            print(f"  [Te] 可通行性估计: {np.mean(tes):.2f} ms")
            print(f"  [Ts] 路径搜索: {np.mean(times):.2f} ± {np.std(times):.2f} ms")
            print(f"  [To] 轨迹优化: 0.00 ms (未启用)")
            print(f"  [Tall] 总导航时间: {np.mean(total_times):.2f} ± {np.std(total_times):.2f} ms")
            print(f"  [Size] 内存使用: {np.mean(sizes):.0f} bytes")
            print(f"  [Ns] 扩展节点: {np.mean(nodes):.0f} ± {np.std(nodes):.0f}")
            print(f"  平均路径长度: {np.mean(lengths):.2f} ± {np.std(lengths):.2f} m")
    
    def call_plan_service(self) -> Dict[str, Any]:
        request = PlanPath.Request()
        
        request.start.position.x = float(self.start_point[0])
        request.start.position.y = float(self.start_point[1])
        request.start.position.z = float(self.start_point[2])
        request.start.orientation.w = 1.0
        
        request.goal.position.x = float(self.goal_point[0])
        request.goal.position.y = float(self.goal_point[1])
        request.goal.position.z = float(self.goal_point[2])
        request.goal.orientation.w = 1.0
        
        request.snap_to_surface = True
        
        response = self.plan_client.call(request)
        
        if response is None:
            return {
                'success': False, 
                'error': 'Service timeout',
                'planning_time_ms': 0.0,
                'expanded_nodes': 0,
            }
        
        if response.success:
            path = np.array([[p.pose.position.x, p.pose.position.y, p.pose.position.z] 
                           for p in response.path.poses])
            length = np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1)) if len(path) > 1 else 0.0
            
            return {
                'success': True,
                'planning_time_ms': response.planning_time_ms,
                'expanded_nodes': response.expanded_nodes,
                'path_length_m': length,
                'num_waypoints': len(path),
                'path': path,
                'error': None,
            }
        else:
            return {
                'success': False,
                'error': response.message,
                'planning_time_ms': response.planning_time_ms,
                'expanded_nodes': response.expanded_nodes,
            }
    
    def publish_path(self, path_points):
        msg = NavPath()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        
        from geometry_msgs.msg import PoseStamped
        for p in path_points:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(p[0])
            pose.pose.position.y = float(p[1])
            pose.pose.position.z = float(p[2])
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        
        self.path_pub.publish(msg)
    
    def save_results(self) -> Optional[str]:
        if not self.all_runs:
            return None
        
        output_dir = Path(__file__).parent.parent / 'results'
        output_dir.mkdir(exist_ok=True)
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = output_dir / f"jps_benchmark_{self.scene_name}_{timestamp}.json"
        
        all_successful = []
        total_trials = 0
        total_successes = 0
        
        for run in self.all_runs:
            total_trials += len(run.trials)
            total_successes += run.success_count
            all_successful.extend(run.successful_trials)
        
        cumulative_stats = {}
        if all_successful:
            times = [t.planning_time_ms for t in all_successful]
            lengths = [t.path_length_m for t in all_successful]
            nodes = [t.expanded_nodes for t in all_successful]
            cumulative_stats = {
                'total_runs': len(self.all_runs),
                'total_trials': total_trials,
                'total_successes': total_successes,
                'success_rate': total_successes / total_trials,
                'planning_time_ms': {
                    'mean': float(np.mean(times)),
                    'std': float(np.std(times)),
                    'min': float(np.min(times)),
                    'max': float(np.max(times)),
                    'median': float(np.median(times)),
                },
                'path_length_m': {
                    'mean': float(np.mean(lengths)),
                    'std': float(np.std(lengths)),
                    'min': float(np.min(lengths)),
                    'max': float(np.max(lengths)),
                },
                'expanded_nodes': {
                    'mean': float(np.mean(nodes)),
                    'std': float(np.std(nodes)),
                    'min': int(np.min(nodes)),
                    'max': int(np.max(nodes)),
                },
            }
        
        runs_data = []
        for run in self.all_runs:
            run_data = {
                'start': run.start,
                'goal': run.goal,
                'euclidean_distance': run.euclidean_distance,
                'stats': run.get_stats(),
                'trials': [
                    {
                        'success': t.success,
                        'planning_time_ms': t.planning_time_ms,
                        'expanded_nodes': t.expanded_nodes,
                        'path_length_m': t.path_length_m,
                        'num_waypoints': t.num_waypoints,
                        'error': t.error,
                    }
                    for t in run.trials
                ],
            }
            runs_data.append(run_data)
        
        data = {
            'planner': 'JPS_3D',
            'timestamp': timestamp,
            'map_info': self.map_info,
            'seed_point': self.seed_point.tolist() if self.seed_point is not None else None,
            'config': {
                'trials_per_run': self.num_trials,
                'snap_to_surface': True,
            },
            'cumulative_stats': cumulative_stats,
            'runs': runs_data,
        }
        
        with open(filename, 'w') as f:
            json.dump(data, f, indent=2)
        
        return str(filename)


def main():
    parser = argparse.ArgumentParser(description='RViz2交互式JPS Benchmark')
    parser.add_argument('--scene', default='spiral', choices=['spiral', 'building', 'plaza'])
    parser.add_argument('--resolution', type=float, default=0.5)
    parser.add_argument('--trials', type=int, default=100, help='每组起终点的测试次数')
    parser.add_argument('--preloaded', action='store_true',
                        help='地图已由外部节点加载（如 obj_mesh_loader），跳过种子点选择和点云发布')
    args = parser.parse_args()

    rclpy.init()

    node = RVizBenchmarkNode(args.scene, args.resolution, args.trials,
                             preloaded=args.preloaded)
    
    # 使用 MultiThreadedExecutor 支持在回调中等待服务响应
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        print("\n\n中断...")
    finally:
        print("\n" + "="*70)
        print("最终统计:")
        print("="*70)
        
        if node.all_runs:
            all_successful = []
            total_trials = 0
            total_successes = 0
            
            for run in node.all_runs:
                total_trials += len(run.trials)
                total_successes += run.success_count
                all_successful.extend(run.successful_trials)
            
            print(f"总测试组数: {len(node.all_runs)}")
            print(f"总测试次数: {total_trials}")
            print(f"总成功次数: {total_successes}")
            print(f"总成功率: {total_successes/total_trials*100:.1f}%")
            
            if all_successful:
                times = [t.planning_time_ms for t in all_successful]
                lengths = [t.path_length_m for t in all_successful]
                nodes = [t.expanded_nodes for t in all_successful]
                print(f"\n纯算法性能 (不含ROS通信开销):")
                print(f"  规划时间: {np.mean(times):.2f} ± {np.std(times):.2f} ms")
                print(f"            min={np.min(times):.2f}, max={np.max(times):.2f}, median={np.median(times):.2f} ms")
                print(f"  路径长度: {np.mean(lengths):.2f} ± {np.std(lengths):.2f} m")
                print(f"  扩展节点: {np.mean(nodes):.0f} ± {np.std(nodes):.0f}")
        
        filename = node.save_results()
        if filename:
            print(f"\n结果已保存到: {filename}")
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
