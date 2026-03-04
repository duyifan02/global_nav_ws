"""
3D2M Planner封装器

封装3D2M (3D to Multi-layer) 规划器调用接口
通过ROS1 topic/service交互，解析输出获取计时
"""

import subprocess
import time
import re
import os
import signal
import numpy as np
from pathlib import Path
from typing import Optional, Tuple, List
from dataclasses import dataclass


PROJECT_ROOT = Path(__file__).parent.parent.parent
D2M_ROOT = PROJECT_ROOT / "3D2M-planner"


@dataclass
class D2MConfig:
    """3D2M规划器配置"""
    pcd_dir: str = "planner/src/read_pcd/PCDFiles/"
    occupancy_resolution: float = 0.3
    travelmap_resolution: float = 0.3
    ros_master_uri: str = "http://localhost:11311"


class D2MPlanner:
    """
    3D2M规划器封装
    
    3D2M是ROS1节点，通过以下方式交互:
    1. 启动roscore和planner节点
    2. 通过rostopic发布goal触发规划
    3. 从stdout解析计时信息
    
    注意: 3D2M有内置benchmark模式(BENCHMARKTIME)，
    如果需要大量测试，建议直接使用其内置功能
    """
    
    # 场景配置
    SCENE_CONFIG = {
        "spiral": {
            "pcd_file": "spiral0.3_2.pcd",
            "init_x": -10.0,
            "init_y": -1.0,
            "min_height": 2.502,
            "max_height": 3.219,
        },
        "building": {
            "pcd_file": "building2_9.pcd",
            "init_x": 5.0,
            "init_y": 5.0,
            "min_height": 0.0,
            "max_height": 10.0,
        },
        "plaza": {
            "pcd_file": "plaza3_10.pcd",
            "init_x": 0.0,
            "init_y": 0.0,
            "min_height": 0.0,
            "max_height": 5.0,
        }
    }
    
    def __init__(self, config: D2MConfig = None):
        self.config = config or D2MConfig()
        self._process = None
        self._roscore = None
        self._loaded_scene = None
        
    def _check_ros1_available(self) -> bool:
        """检查ROS1环境是否可用"""
        try:
            result = subprocess.run(
                ["bash", "-c", "source /opt/ros/noetic/setup.bash && which roscore"],
                capture_output=True, text=True, timeout=5
            )
            return result.returncode == 0
        except:
            return False
    
    def start_roscore(self) -> bool:
        """启动roscore"""
        if self._roscore is not None:
            return True
        try:
            self._roscore = subprocess.Popen(
                ["bash", "-c", "source /opt/ros/noetic/setup.bash && roscore"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid
            )
            time.sleep(2)  # 等待roscore启动
            return True
        except Exception as e:
            print(f"[3D2M] Failed to start roscore: {e}")
            return False
    
    def stop_roscore(self):
        """停止roscore"""
        if self._roscore:
            os.killpg(os.getpgid(self._roscore.pid), signal.SIGTERM)
            self._roscore = None
    
    def load_scene(self, scene_name: str) -> bool:
        """
        加载场景 (准备但不启动节点)
        
        Args:
            scene_name: 场景名 (spiral/building/plaza)
        """
        scene_cfg = self.SCENE_CONFIG.get(scene_name.lower())
        if scene_cfg is None:
            raise ValueError(f"Unknown scene: {scene_name}")
        
        self._loaded_scene = scene_name.lower()
        self._scene_cfg = scene_cfg
        return True
    
    def plan(self, start: np.ndarray, goal: np.ndarray, timeout: float = 30.0) -> dict:
        """
        执行路径规划
        
        由于3D2M是完整的ROS系统，单次规划需要:
        1. 启动roscore
        2. 启动地图读取节点
        3. 启动规划节点
        4. 发布goal
        5. 等待结果并解析
        
        对于benchmark，建议使用run_builtin_benchmark()
        
        Args:
            start: 起点 [x, y, z]
            goal: 终点 [x, y, z]
            timeout: 超时时间(秒)
            
        Returns:
            规划结果字典
        """
        if self._loaded_scene is None:
            return {
                'success': False,
                'tp_ms': 0.0,
                'te_ms': 0.0,
                'ts_ms': 0,
                'ns_visited_nodes': 0,
                'planning_time_ms': 0,
                'path': None,
                'path_length_m': None,
                'error': 'Scene not loaded. Call load_scene() first.'
            }
        
        # 这里是简化实现
        # 完整实现需要启动ROS节点并通过topic交互
        return {
            'success': False,
            'tp_ms': 0.0,
            'te_ms': 0.0,
            'ts_ms': 0,
            'ns_visited_nodes': 0,
            'planning_time_ms': 0,
            'path': None,
            'path_length_m': None,
            'error': 'Single planning not implemented. Use run_builtin_benchmark() for batch testing.'
        }
    
    def run_builtin_benchmark(self, scene_name: str, num_trials: int = 100) -> dict:
        """
        使用3D2M内置的benchmark模式运行测试
        
        3D2M的planner_manager.cpp有两个benchmark模式:
        - BENCHMARK100: 随机100对起终点
        - BENCHMARKTIME: 随机2000对起终点，记录时间和长度
        
        需要先修改代码启用宏定义，然后重新编译
        
        Args:
            scene_name: 场景名
            num_trials: 试验次数 (仅供参考，实际由代码宏控制)
            
        Returns:
            benchmark结果
        """
        return {
            'note': '3D2M benchmark需要通过修改代码宏定义并重新编译来启用',
            'steps': [
                '1. 编辑 planner_manager.cpp',
                '2. 取消注释 #define BENCHMARKTIME 或 #define BENCHMARK100',
                '3. catkin_make 重新编译',
                '4. roslaunch 运行',
                '5. 结果会输出到指定文件'
            ],
            'output_files': {
                'BENCHMARK100': ['COORDS_FILE', 'TRAJS_FILE'],
                'BENCHMARKTIME': ['time_ms, length per trial']
            }
        }
    
    def cleanup(self):
        """清理资源"""
        self.stop_roscore()


def generate_launch_file(scene_name: str, 
                         pcd_file: str,
                         output_path: str,
                         init_x: float = 0.0,
                         init_y: float = 0.0) -> str:
    """
    生成用于benchmark的launch文件
    
    Args:
        scene_name: 场景名
        pcd_file: PCD文件路径
        output_path: 输出launch文件路径
        init_x, init_y: 初始位置
        
    Returns:
        生成的launch文件路径
    """
    launch_content = f'''<?xml version="1.0"?>
<launch>
  <!-- Generated launch file for {scene_name} benchmark -->
  
  <!-- PCD Map Reader -->
  <node pkg="read_pcd" name="read_pcd" type="read_pcd" output="screen">
    <param name="map/map_file" value="{pcd_file}"/>
  </node>
  
  <!-- Planner Node -->
  <node pkg="planner_manager" name="scf_planner_node" type="scf_planner_node" output="screen">
    <param name="occupancy_resolution" value="0.3"/>
    <param name="travelmap_resolution" value="0.3"/>
    <param name="sta_threshold" value="1"/>
    <param name="debug_output" value="false"/>
    <param name="traj_parlength" value="1.5"/>
    <param name="truncation_dis" value="2.5"/>
    
    <remap from="~globalmap" to="/globalmap"/>
    <remap from="~rcvmap_signal" to="/rcvmap_signal"/>
    <remap from="~trajectory" to="/trajectory"/>
  </node>
  
  <!-- Fake Robot for Odometry -->
  <include file="$(find fake_diablo)/launch/fake_diablo.launch">
    <arg name="init_x" value="{init_x}"/>
    <arg name="init_y" value="{init_y}"/>
  </include>
  
</launch>
'''
    with open(output_path, 'w') as f:
        f.write(launch_content)
    return output_path


def parse_3d2m_output(log_content: str) -> List[dict]:
    """
    解析3D2M的stdout输出，提取规划结果
    
    3D2M输出格式: "total cost: XXX ms"
    
    Args:
        log_content: 日志内容
        
    Returns:
        解析出的结果列表
    """
    results = []
    # 匹配 "total cost: XXX ms" 或类似格式
    pattern = r"total cost:\s*([\d.]+)\s*ms"
    matches = re.findall(pattern, log_content)
    
    for i, match in enumerate(matches):
        results.append({
            'trial': i,
            'tp_ms': 0.0,
            'te_ms': 0.0,
            'ts_ms': float(match),
            'ns_visited_nodes': 0,
            'planning_time_ms': float(match),
            'success': True
        })
    
    return results


# 场景默认起终点
SCENE_ENDPOINTS = {
    "spiral": {
        "start": np.array([-10.0, -1.0, 2.8]),
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


if __name__ == "__main__":
    print("3D2M Wrapper")
    print("=" * 50)
    print("3D2M是ROS1 C++项目，需要通过以下步骤运行benchmark:")
    print()
    print("方法1: 使用内置benchmark模式")
    print("  1. vim 3D2M-planner/planner/src/planner_manager/src/planner_manager.cpp")
    print("  2. 取消注释: #define BENCHMARKTIME")
    print("  3. cd 3D2M-planner/planner && catkin_make")
    print("  4. source devel/setup.bash")
    print("  5. roslaunch planner_manager scene1.launch")
    print()
    print("方法2: 通过topic发送goal")
    print("  1. 启动节点: roslaunch planner_manager scene1.launch")
    print("  2. 发送goal: rostopic pub /goal geometry_msgs/PoseStamped ...")
    print("  3. 观察stdout输出的 'total cost: XXX ms'")
