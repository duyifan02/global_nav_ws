#!/usr/bin/env python3
"""
3D路径规划器Benchmark主运行脚本

运行PCT、3D2M、JPS三个规划器的对比实验
"""

import argparse
import json
import time
import os
import sys
from pathlib import Path
from datetime import datetime
from typing import List, Dict, Optional

import numpy as np

# 添加benchmark模块路径
BENCHMARK_ROOT = Path(__file__).parent
PROJECT_ROOT = BENCHMARK_ROOT.parent
sys.path.insert(0, str(BENCHMARK_ROOT))

from config import SCENES, NUM_TRIALS, TIMEOUT_SECONDS
from metrics import PlanningResult, aggregate_results, save_results_csv


def run_pct_tests(scene_name: str, num_trials: int, verbose: bool = True) -> List[dict]:
    """运行PCT规划器测试"""
    if verbose:
        print(f"\n[PCT] 测试场景: {scene_name}")
    
    try:
        from planners.pct_wrapper import PCTPlanner, SCENE_ENDPOINTS
        
        planner = PCTPlanner()
        if not planner.load_scene(scene_name):
            return [{'success': False, 'error': 'Failed to load scene', 'trial': i} 
                    for i in range(num_trials)]
        
        start = SCENE_ENDPOINTS[scene_name.lower()]["start"]
        goal = SCENE_ENDPOINTS[scene_name.lower()]["goal"]
        
        results = []
        for i in range(num_trials):
            result = planner.plan(start, goal)
            result['trial'] = i
            result['planner'] = 'PCT'
            result['scene'] = scene_name
            results.append(result)
            
            if verbose and (i + 1) % 10 == 0:
                print(f"  完成 {i + 1}/{num_trials} 次")
        
        return results
        
    except Exception as e:
        print(f"[PCT] 错误: {e}")
        return [{'success': False, 'error': str(e), 'trial': i, 'planner': 'PCT', 'scene': scene_name} 
                for i in range(num_trials)]


def run_jps_tests(scene_name: str, num_trials: int, resolution: float = 0.1, verbose: bool = True) -> List[dict]:
    """运行JPS规划器测试"""
    if verbose:
        print(f"\n[JPS] 测试场景: {scene_name}")
    
    try:
        from planners.jps_wrapper import JPSPlanner, JPSConfig, SCENE_ENDPOINTS, ROS2_AVAILABLE
        
        if not ROS2_AVAILABLE:
            print("[JPS] ROS2不可用，跳过JPS测试")
            return [{'success': False, 'error': 'ROS2 not available', 'trial': i,
                     'planner': 'JPS', 'scene': scene_name} for i in range(num_trials)]
        
        config = JPSConfig(resolution=resolution)
        planner = JPSPlanner(config)
        
        if not planner.init_ros2():
            return [{'success': False, 'error': 'Failed to init ROS2', 'trial': i,
                     'planner': 'JPS', 'scene': scene_name} for i in range(num_trials)]
        
        if not planner.load_scene(scene_name):
            return [{'success': False, 'error': 'Failed to load scene', 'trial': i,
                     'planner': 'JPS', 'scene': scene_name} for i in range(num_trials)]
        
        start = SCENE_ENDPOINTS[scene_name.lower()]["start"]
        goal = SCENE_ENDPOINTS[scene_name.lower()]["goal"]
        
        results = []
        for i in range(num_trials):
            result = planner.plan(start, goal)
            result['trial'] = i
            result['planner'] = 'JPS'
            result['scene'] = scene_name
            results.append(result)
            
            if verbose and (i + 1) % 10 == 0:
                print(f"  完成 {i + 1}/{num_trials} 次")
        
        planner.shutdown()
        return results
        
    except Exception as e:
        print(f"[JPS] 错误: {e}")
        return [{'success': False, 'error': str(e), 'trial': i, 'planner': 'JPS', 'scene': scene_name} 
                for i in range(num_trials)]


def run_3d2m_tests(scene_name: str, num_trials: int, verbose: bool = True) -> List[dict]:
    """
    运行3D2M规划器测试
    
    注意: 3D2M是ROS1 C++项目，需要单独运行
    这里只返回占位结果，实际需要:
    1. 修改代码启用BENCHMARKTIME宏
    2. catkin_make编译
    3. roslaunch运行
    4. 从输出文件读取结果
    """
    if verbose:
        print(f"\n[3D2M] 测试场景: {scene_name}")
        print("  注意: 3D2M需要单独运行ROS1环境")
        print("  请参考 benchmark/planners/d2m_wrapper.py 的使用说明")
    
    # 返回占位结果
    return [{'success': False, 'error': 'Run 3D2M separately via ROS1', 
             'trial': i, 'planner': '3D2M', 'scene': scene_name,
             'note': '需要通过roslaunch单独运行'} 
            for i in range(num_trials)]


def convert_to_planning_results(results: List[dict]) -> List[PlanningResult]:
    """将字典结果转换为PlanningResult对象"""
    planning_results = []
    for r in results:
        ts_ms = r.get('ts_ms', r.get('planning_time_ms', 0))
        ns = r.get('ns_visited_nodes', r.get('expanded_nodes', r.get('nodes_explored')))
        pr = PlanningResult(
            success=r.get('success', False),
            planning_time_ms=ts_ms,
            path_length_m=r.get('path_length_m'),
            nodes_explored=ns,
            error_message=r.get('error')
        )
        if r.get('path') is not None:
            pr.waypoints = np.array(r['path'])
        planning_results.append(pr)
    return planning_results


def print_summary(all_results: Dict[str, Dict[str, List[dict]]]):
    """打印汇总结果"""
    print("\n" + "=" * 70)
    print("Benchmark 汇总")
    print("=" * 70)
    
    # 表头
    print(f"\n{'规划器':<10} {'场景':<12} {'成功率':<10} {'平均Ts(ms)':<15} {'平均长度(m)':<12}")
    print("-" * 70)
    
    for planner_name, scene_results in all_results.items():
        for scene_name, results in scene_results.items():
            planning_results = convert_to_planning_results(results)
            stats = aggregate_results(planning_results)
            
            success_rate = f"{stats['success_rate']*100:.1f}%"
            avg_time = f"{stats.get('ts_mean_ms', stats.get('planning_time_mean_ms', 0)):.2f}" if stats['success_rate'] > 0 else "N/A"
            avg_length = f"{stats.get('path_length_mean_m', 0):.2f}" if stats.get('path_length_mean_m') else "N/A"
            
            print(f"{planner_name:<10} {scene_name:<12} {success_rate:<10} {avg_time:<15} {avg_length:<12}")
    
    print("=" * 70)


def save_results(all_results: Dict[str, Dict[str, List[dict]]], output_dir: str):
    """保存结果到文件"""
    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)
    
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    
    # 保存原始JSON
    json_file = output_path / f"benchmark_{timestamp}.json"
    
    # 转换numpy数组为列表以便JSON序列化
    def convert_for_json(obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        elif isinstance(obj, dict):
            return {k: convert_for_json(v) for k, v in obj.items()}
        elif isinstance(obj, list):
            return [convert_for_json(item) for item in obj]
        return obj
    
    with open(json_file, 'w') as f:
        json.dump(convert_for_json(all_results), f, indent=2)
    
    print(f"\n结果已保存到: {json_file}")
    
    # 保存CSV
    for planner_name, scene_results in all_results.items():
        for scene_name, results in scene_results.items():
            planning_results = convert_to_planning_results(results)
            csv_file = output_path / f"{planner_name}_{scene_name}_{timestamp}.csv"
            save_results_csv(planning_results, planner_name, scene_name, str(csv_file))


def main():
    parser = argparse.ArgumentParser(description='3D路径规划器Benchmark')
    parser.add_argument('--planners', nargs='+', default=['PCT', 'JPS'],
                        choices=['PCT', '3D2M', 'JPS', 'all'],
                        help='要测试的规划器')
    parser.add_argument('--scenes', nargs='+', default=['spiral'],
                        choices=['spiral', 'building', 'plaza', 'all'],
                        help='要测试的场景')
    parser.add_argument('--trials', type=int, default=NUM_TRIALS,
                        help=f'每个场景的测试次数 (默认: {NUM_TRIALS})')
    parser.add_argument('--resolution', type=float, default=0.1,
                        help='体素分辨率 (默认: 0.1)')
    parser.add_argument('--output', type=str, default='./results',
                        help='结果输出目录')
    parser.add_argument('--quiet', action='store_true',
                        help='减少输出')
    
    args = parser.parse_args()
    
    # 处理 'all' 选项
    if 'all' in args.planners:
        planners = ['PCT', '3D2M', 'JPS']
    else:
        planners = args.planners
    
    if 'all' in args.scenes:
        scenes = list(SCENES.keys())
    else:
        scenes = args.scenes
    
    verbose = not args.quiet
    
    print("=" * 70)
    print("3D路径规划器 Benchmark")
    print("=" * 70)
    print(f"规划器: {', '.join(planners)}")
    print(f"场景: {', '.join(scenes)}")
    print(f"每场景测试次数: {args.trials}")
    print(f"体素分辨率: {args.resolution}")
    print("=" * 70)
    
    all_results: Dict[str, Dict[str, List[dict]]] = {}
    
    for planner in planners:
        all_results[planner] = {}
        
        for scene in scenes:
            if planner == 'PCT':
                results = run_pct_tests(scene, args.trials, verbose)
            elif planner == 'JPS':
                results = run_jps_tests(scene, args.trials, args.resolution, verbose)
            elif planner == '3D2M':
                results = run_3d2m_tests(scene, args.trials, verbose)
            else:
                results = []
            
            all_results[planner][scene] = results
    
    # 打印汇总
    print_summary(all_results)
    
    # 保存结果
    save_results(all_results, args.output)
    
    print("\nBenchmark 完成!")


if __name__ == "__main__":
    main()
