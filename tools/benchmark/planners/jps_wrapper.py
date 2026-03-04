"""
JPS Planner封装器

封装JPS (Jump Point Search) 3D规划器调用接口
通过ROS2 service交互
"""

import time
import numpy as np
from pathlib import Path
from typing import Optional, List, Tuple
from dataclasses import dataclass

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Point, Pose, PoseStamped
    from nav_msgs.msg import Path as NavPath
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    print("[JPS] Warning: ROS2 not available")


PROJECT_ROOT = Path(__file__).parent.parent.parent
JPS_ROOT = PROJECT_ROOT / "global_nav_ws"


@dataclass
class JPSConfig:
    """JPS规划器配置"""
    resolution: float = 0.1
    snap_to_surface: bool = False
    service_timeout: float = 10.0


class JPSPlanner:
    """
    JPS规划器封装 (ROS2)
    
    调用流程:
    1. 调用 set_voxel_map 服务设置地图
    2. 调用 plan_path 服务执行规划
    
    服务接口:
    - SetVoxelMap.srv: resolution, occupied[] -> success, message
    - PlanPath.srv: start, goal, snap_to_surface -> success, message, path
    """
    
    def __init__(self, config: JPSConfig = None):
        self.config = config or JPSConfig()
        self._node = None
        self._set_map_client = None
        self._plan_client = None
        self._map_loaded = False
        self._loaded_scene = None
        
    def init_ros2(self) -> bool:
        """初始化ROS2节点"""
        if not ROS2_AVAILABLE:
            return False
            
        try:
            if not rclpy.ok():
                rclpy.init()
            self._node = rclpy.create_node('jps_benchmark_client')
            
            # 导入服务类型
            from nav_interfaces.srv import SetVoxelMap, PlanPath
            
            self._set_map_client = self._node.create_client(
                SetVoxelMap, '/set_voxel_map'
            )
            self._plan_client = self._node.create_client(
                PlanPath, '/plan_path'
            )
            
            # 等待服务可用
            if not self._set_map_client.wait_for_service(timeout_sec=5.0):
                print("[JPS] set_voxel_map service not available")
                return False
            if not self._plan_client.wait_for_service(timeout_sec=5.0):
                print("[JPS] plan_path service not available")
                return False
                
            return True
        except Exception as e:
            print(f"[JPS] Failed to init ROS2: {e}")
            return False
    
    def set_voxel_map(self, occupied_points: List[Tuple[float, float, float]],
                      resolution: float = None,
                      seed_point: np.ndarray = None) -> bool:
        """
        设置体素地图

        Args:
            occupied_points: 占据体素中心坐标列表 [(x,y,z), ...]
            resolution: 体素分辨率
            seed_point: BFS种子点世界坐标 [x, y, z]，指定可行走区域起点
                        应传入规划起始位置，确保BFS从正确位置扩展

        Returns:
            是否成功
        """
        if self._set_map_client is None:
            print("[JPS] ROS2 not initialized")
            return False

        from nav_interfaces.srv import SetVoxelMap
        from geometry_msgs.msg import Point

        request = SetVoxelMap.Request()
        request.resolution = resolution or self.config.resolution

        # 转换点格式
        for p in occupied_points:
            point = Point()
            point.x, point.y, point.z = float(p[0]), float(p[1]), float(p[2])
            request.occupied.append(point)

        # 设置BFS种子点（论文中以机器人当前位置为种子）
        if seed_point is not None:
            request.seed_point.x = float(seed_point[0])
            request.seed_point.y = float(seed_point[1])
            request.seed_point.z = float(seed_point[2]) if len(seed_point) > 2 else 0.0

        # 调用服务
        future = self._set_map_client.call_async(request)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=self.config.service_timeout)

        if future.result() is not None:
            response = future.result()
            self._map_loaded = response.success
            if not response.success:
                print(f"[JPS] set_voxel_map failed: {response.message}")
            else:
                print(f"[JPS] surface voxels: {response.surface_voxel_count}, "
                      f"Te={response.traversability_time_ms:.1f}ms")
            return response.success
        else:
            print("[JPS] set_voxel_map service call failed")
            return False
    
    def load_scene(self, scene_name: str, voxel_map=None, seed_point: np.ndarray = None) -> bool:
        """
        加载场景地图

        Args:
            scene_name: 场景名
            voxel_map: VoxelMap对象 (来自pcd_to_voxel)，None则自动加载
            seed_point: BFS种子点，None则使用该场景的默认起始位置

        Returns:
            是否成功
        """
        if voxel_map is None:
            from converters.pcd_to_voxel import get_voxel_map
            voxel_map = get_voxel_map(scene_name, self.config.resolution)

        # 使用规划起始位置作为BFS种子（论文：以机器人位置为种子）
        if seed_point is None:
            seed_point = SCENE_ENDPOINTS.get(scene_name.lower(), {}).get("start")

        success = self.set_voxel_map(
            voxel_map.occupied_list,
            voxel_map.resolution,
            seed_point=seed_point
        )

        if success:
            self._loaded_scene = scene_name.lower()
        return success
    
    def plan(self, start: np.ndarray, goal: np.ndarray) -> dict:
        """
        执行路径规划
        
        Args:
            start: 起点 [x, y, z]
            goal: 终点 [x, y, z]
            
        Returns:
            规划结果字典
        """
        if not self._map_loaded:
            return {
                'success': False,
                'planning_time_ms': 0,
                'path': None,
                'path_length_m': None,
                'error': 'Map not loaded. Call load_scene() first.'
            }
        
        from nav_interfaces.srv import PlanPath
        from geometry_msgs.msg import Pose
        
        request = PlanPath.Request()
        
        # 设置起点
        request.start.position.x = float(start[0])
        request.start.position.y = float(start[1])
        request.start.position.z = float(start[2]) if len(start) > 2 else 0.0
        request.start.orientation.w = 1.0
        
        # 设置终点
        request.goal.position.x = float(goal[0])
        request.goal.position.y = float(goal[1])
        request.goal.position.z = float(goal[2]) if len(goal) > 2 else 0.0
        request.goal.orientation.w = 1.0
        
        request.snap_to_surface = self.config.snap_to_surface
        
        # 计时调用
        t_start = time.perf_counter()
        future = self._plan_client.call_async(request)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=self.config.service_timeout)
        t_end = time.perf_counter()
        
        planning_time_ms = (t_end - t_start) * 1000
        
        if future.result() is not None:
            response = future.result()
            
            if response.success:
                # 提取路径点
                path_points = []
                for pose_stamped in response.path.poses:
                    p = pose_stamped.pose.position
                    path_points.append([p.x, p.y, p.z])
                path = np.array(path_points)
                
                # 计算路径长度
                path_length = self._compute_path_length(path)
                
                return {
                    'success': True,
                    'planning_time_ms': planning_time_ms,
                    'path': path,
                    'path_length_m': path_length,
                    'error': None
                }
            else:
                return {
                    'success': False,
                    'planning_time_ms': planning_time_ms,
                    'path': None,
                    'path_length_m': None,
                    'error': response.message
                }
        else:
            return {
                'success': False,
                'planning_time_ms': planning_time_ms,
                'path': None,
                'path_length_m': None,
                'error': 'Service call failed or timed out'
            }
    
    def _compute_path_length(self, path: np.ndarray) -> float:
        """计算路径长度"""
        if path is None or len(path) < 2:
            return 0.0
        diffs = np.diff(path, axis=0)
        return np.sum(np.linalg.norm(diffs, axis=1))
    
    def shutdown(self):
        """关闭ROS2节点"""
        if self._node is not None:
            self._node.destroy_node()
            self._node = None
        # 注意: 不要在这里调用rclpy.shutdown()，可能影响其他节点


class JPSPlannerStandalone:
    """
    JPS规划器独立封装 (不依赖ROS2运行时)
    
    直接调用编译后的JPS库，用于不需要ROS2环境的场景
    """
    
    def __init__(self, config: JPSConfig = None):
        self.config = config or JPSConfig()
        self._map_data = None
        self._loaded_scene = None
    
    def load_scene(self, scene_name: str) -> bool:
        """加载场景 (仅准备数据，实际规划需要ROS2)"""
        from converters.pcd_to_voxel import get_voxel_map
        self._map_data = get_voxel_map(scene_name, self.config.resolution)
        self._loaded_scene = scene_name.lower()
        return True
    
    def plan(self, start: np.ndarray, goal: np.ndarray) -> dict:
        """执行规划 (需要ROS2节点运行)"""
        return {
            'success': False,
            'planning_time_ms': 0,
            'path': None,
            'error': 'Standalone mode requires external JPS node. Start jps_planner_node first.'
        }


# 场景默认起终点
SCENE_ENDPOINTS = {
    "spiral": {
        "start": np.array([-16.0, -6.0, 2.8]),
        "goal": np.array([-26.0, -5.0, 2.8])
    },
    "building": {
        "start": np.array([5.0, 5.0, 0.0]),
        "goal": np.array([-6.0, -1.0, 0.0])
    },
    "plaza": {
        "start": np.array([0.0, 0.0, 0.0]),
        "goal": np.array([23.0, 10.0, 0.0])
    }
}


def run_jps_benchmark(scene_name: str,
                      start: np.ndarray = None,
                      goal: np.ndarray = None,
                      num_trials: int = 1,
                      resolution: float = 0.1) -> list:
    """
    运行JPS benchmark
    
    前提条件:
    1. ROS2环境已source
    2. jps_planner_node正在运行
    
    Args:
        scene_name: 场景名
        start: 起点，None则使用默认
        goal: 终点，None则使用默认
        num_trials: 试验次数
        resolution: 体素分辨率
        
    Returns:
        结果列表
    """
    if not ROS2_AVAILABLE:
        return [{'success': False, 'error': 'ROS2 not available'}]
    
    # 使用默认起终点
    if start is None:
        start = SCENE_ENDPOINTS[scene_name.lower()]["start"]
    if goal is None:
        goal = SCENE_ENDPOINTS[scene_name.lower()]["goal"]
    
    config = JPSConfig(resolution=resolution)
    planner = JPSPlanner(config)
    
    if not planner.init_ros2():
        return [{'success': False, 'error': 'Failed to init ROS2'}]
    
    if not planner.load_scene(scene_name):
        return [{'success': False, 'error': 'Failed to load scene'}]
    
    results = []
    for i in range(num_trials):
        result = planner.plan(start, goal)
        result['trial'] = i
        results.append(result)
    
    planner.shutdown()
    return results


if __name__ == "__main__":
    print("JPS Wrapper")
    print("=" * 50)
    print()
    print("使用方法:")
    print("1. 启动JPS节点:")
    print("   cd global_nav_ws")
    print("   source install/setup.bash")
    print("   ros2 run jps_planner_3d jps_planner_node")
    print()
    print("2. 运行benchmark:")
    print("   python jps_wrapper.py --scene spiral --trials 10")
    print()
    
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", default="spiral", choices=["spiral", "building", "plaza"])
    parser.add_argument("--trials", type=int, default=1)
    parser.add_argument("--resolution", type=float, default=0.1)
    args = parser.parse_args()
    
    if ROS2_AVAILABLE:
        print(f"Running JPS on {args.scene}...")
        results = run_jps_benchmark(args.scene, num_trials=args.trials, resolution=args.resolution)
        
        for r in results:
            if r['success']:
                print(f"Trial {r.get('trial', 0)}: {r['planning_time_ms']:.2f}ms, {r['path_length_m']:.2f}m")
            else:
                print(f"Trial {r.get('trial', 0)}: FAILED - {r.get('error', 'Unknown')}")
    else:
        print("ROS2 not available. Install ROS2 and source the workspace first.")
