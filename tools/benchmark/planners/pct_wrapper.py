"""
PCT Planner封装器

封装PCT (Point Cloud Tomography) 规划器调用接口
复用原有TomogramPlanner类，添加计时和统一接口
"""

import sys
import time
import numpy as np
from pathlib import Path
from typing import Optional, Tuple
from dataclasses import dataclass

# 添加PCT路径
PROJECT_ROOT = Path(__file__).parent.parent.parent
PCT_ROOT = PROJECT_ROOT / "PCT_planner"
PCT_PLANNER_SCRIPTS = PCT_ROOT / "planner" / "scripts"
PCT_LIB = PCT_ROOT / "planner"

sys.path.insert(0, str(PCT_PLANNER_SCRIPTS))
sys.path.insert(0, str(PCT_LIB))


@dataclass
class PCTConfig:
    """PCT规划器配置"""
    tomo_dir: str = "/rsc/tomogram/"
    use_quintic: bool = True
    max_heading_rate: float = 1.0


class PCTPlanner:
    """
    PCT规划器封装
    
    使用方法:
        planner = PCTPlanner()
        planner.load_scene("spiral")
        result = planner.plan(start_2d, goal_2d)
    """
    
    # 场景到tomogram文件的映射
    SCENE_TOMO_FILES = {
        "spiral": "spiral0.3_2",
        "building": "building2_9", 
        "plaza": "plaza3_10",
    }
    
    def __init__(self, config: PCTConfig = None):
        self.config = config or PCTConfig()
        self._planner = None
        self._loaded_scene = None
        
    def _get_cfg_object(self):
        """构造PCT需要的cfg对象"""
        class Wrapper:
            tomo_dir = self.config.tomo_dir
        class Planner:
            use_quintic = self.config.use_quintic
            max_heading_rate = self.config.max_heading_rate
        class Cfg:
            wrapper = Wrapper()
            planner = Planner()
        return Cfg()
    
    def load_scene(self, scene_name: str) -> bool:
        """
        加载场景的tomogram
        
        Args:
            scene_name: 场景名 (spiral/building/plaza)
            
        Returns:
            是否加载成功
        """
        if scene_name.lower() == self._loaded_scene:
            return True
            
        tomo_file = self.SCENE_TOMO_FILES.get(scene_name.lower())
        if tomo_file is None:
            raise ValueError(f"Unknown scene: {scene_name}")
        
        try:
            # 动态导入避免启动时依赖
            from planner_wrapper import TomogramPlanner
            
            cfg = self._get_cfg_object()
            self._planner = TomogramPlanner(cfg)
            self._planner.loadTomogram(tomo_file)
            self._loaded_scene = scene_name.lower()
            return True
        except Exception as e:
            print(f"[PCT] Failed to load scene {scene_name}: {e}")
            return False
    
    def plan(self, start: np.ndarray, goal: np.ndarray) -> dict:
        """
        执行路径规划
        
        Args:
            start: 起点 [x, y] 或 [x, y, z] (z会被忽略，PCT是2D规划+高程)
            goal: 终点 [x, y] 或 [x, y, z]
            
        Returns:
            dict: {
                'success': bool,
                'ts_ms': float,
                'path': np.ndarray (Nx3) or None,
                'path_length_m': float or None,
                'error': str or None
            }
        """
        if self._planner is None:
            return {
                'success': False,
                'ts_ms': 0,
                'ns_visited_nodes': 0,
                'planning_time_ms': 0,
                'path': None,
                'path_length_m': None,
                'error': 'Planner not initialized. Call load_scene() first.'
            }
        
        # 转换为2D输入
        start_2d = np.array(start[:2], dtype=np.float32)
        goal_2d = np.array(goal[:2], dtype=np.float32)
        
        # 计时执行
        t_start = time.perf_counter()
        try:
            traj_3d = self._planner.plan(start_2d, goal_2d)
            t_end = time.perf_counter()
            
            if traj_3d is None or len(traj_3d) == 0:
                ts_ms = (t_end - t_start) * 1000
                return {
                    'success': False,
                    'ts_ms': ts_ms,
                    'ns_visited_nodes': 0,
                    'planning_time_ms': ts_ms,
                    'path': None,
                    'path_length_m': None,
                    'error': 'No path found'
                }
            
            # 计算路径长度
            path_length = self._compute_path_length(traj_3d)
            
            ts_ms = (t_end - t_start) * 1000
            return {
                'success': True,
                'ts_ms': ts_ms,
                'ns_visited_nodes': 0,
                'planning_time_ms': ts_ms,
                'path': traj_3d,
                'path_length_m': path_length,
                'error': None
            }
            
        except Exception as e:
            t_end = time.perf_counter()
            ts_ms = (t_end - t_start) * 1000
            return {
                'success': False,
                'ts_ms': ts_ms,
                'ns_visited_nodes': 0,
                'planning_time_ms': ts_ms,
                'path': None,
                'path_length_m': None,
                'error': str(e)
            }
    
    def _compute_path_length(self, path: np.ndarray) -> float:
        """计算路径长度"""
        if path is None or len(path) < 2:
            return 0.0
        diffs = np.diff(path, axis=0)
        return np.sum(np.linalg.norm(diffs, axis=1))


# 场景默认起终点 (来自PCT plan.py)
SCENE_ENDPOINTS = {
    "spiral": {
        "start": np.array([-16.0, -6.0]),
        "goal": np.array([-26.0, -5.0])
    },
    "building": {
        "start": np.array([5.0, 5.0]),
        "goal": np.array([-6.0, -1.0])
    },
    "plaza": {
        "start": np.array([0.0, 0.0]),
        "goal": np.array([23.0, 10.0])
    }
}


def run_pct_benchmark(scene_name: str, 
                      start: np.ndarray = None,
                      goal: np.ndarray = None,
                      num_trials: int = 1) -> list:
    """
    运行PCT benchmark
    
    Args:
        scene_name: 场景名
        start: 起点，None则使用默认
        goal: 终点，None则使用默认
        num_trials: 试验次数
        
    Returns:
        结果列表
    """
    # 使用默认起终点
    if start is None:
        start = SCENE_ENDPOINTS[scene_name.lower()]["start"]
    if goal is None:
        goal = SCENE_ENDPOINTS[scene_name.lower()]["goal"]
    
    planner = PCTPlanner()
    if not planner.load_scene(scene_name):
        return [{'success': False, 'error': 'Failed to load scene'}]
    
    results = []
    for i in range(num_trials):
        result = planner.plan(start, goal)
        result['trial'] = i
        results.append(result)
        
    return results


if __name__ == "__main__":
    # 测试
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", default="spiral", choices=["spiral", "building", "plaza"])
    parser.add_argument("--trials", type=int, default=1)
    args = parser.parse_args()
    
    print(f"Running PCT on {args.scene}...")
    results = run_pct_benchmark(args.scene, num_trials=args.trials)
    
    for r in results:
        if r['success']:
            ts_ms = r.get('ts_ms', r.get('planning_time_ms', 0.0))
            print(f"Trial {r['trial']}: Ts={ts_ms:.2f}ms, {r['path_length_m']:.2f}m")
        else:
            print(f"Trial {r['trial']}: FAILED - {r['error']}")
