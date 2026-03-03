#!/usr/bin/env python3
"""
半自动化起终点选择工具

用法:
    python select_endpoints.py --scene spiral --resolution 0.1

功能:
    1. 加载点云并显示3D可视化
    2. 打印点云统计信息
    3. 你可以在终端输入坐标测试是否有效
    4. 最后输出可以直接复制到jps_wrapper.py的配置
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

import numpy as np

def load_scene(scene_name: str, resolution: float):
    from converters.pcd_to_voxel import get_voxel_map
    return get_voxel_map(scene_name, resolution)

def print_stats(vmap):
    points = vmap.voxel_centers
    print("\n" + "="*60)
    print("点云统计")
    print("="*60)
    print(f"总体素数: {len(points)}")
    print(f"X范围: [{points[:,0].min():.2f}, {points[:,0].max():.2f}]")
    print(f"Y范围: [{points[:,1].min():.2f}, {points[:,1].max():.2f}]")
    print(f"Z范围: [{points[:,2].min():.2f}, {points[:,2].max():.2f}]")
    
    print("\n各Z层分布:")
    z_min, z_max = points[:,2].min(), points[:,2].max()
    for z in np.arange(int(z_min), int(z_max)+1, 2):
        mask = (points[:,2] >= z) & (points[:,2] < z+2)
        count = mask.sum()
        if count > 0:
            layer = points[mask]
            print(f"  z=[{z:2d},{z+2:2d}): {count:6d}点  "
                  f"x:[{layer[:,0].min():6.1f}, {layer[:,0].max():6.1f}]  "
                  f"y:[{layer[:,1].min():6.1f}, {layer[:,1].max():6.1f}]")

def find_nearest_voxel(point, voxels):
    distances = np.linalg.norm(voxels - point, axis=1)
    idx = distances.argmin()
    return voxels[idx], distances[idx]

def check_point(point, voxels, free_margin=0.3):
    nearest, dist = find_nearest_voxel(point, voxels)
    is_valid = dist <= free_margin
    return is_valid, nearest, dist

def visualize_with_matplotlib(vmap, start=None, goal=None, downsample=10):
    try:
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D
    except ImportError:
        print("matplotlib不可用，跳过可视化")
        return
    
    points = vmap.voxel_centers[::downsample]
    
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    ax.scatter(points[:,0], points[:,1], points[:,2], 
               c=points[:,2], cmap='viridis', s=1, alpha=0.3)
    
    if start is not None:
        ax.scatter(*start, c='green', s=200, marker='o', label='Start')
    if goal is not None:
        ax.scatter(*goal, c='red', s=200, marker='x', label='Goal')
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    
    plt.title(f"点云可视化 (每{downsample}个点采样1个)")
    plt.savefig('/tmp/pointcloud_view.png', dpi=150)
    print(f"\n可视化已保存到: /tmp/pointcloud_view.png")
    plt.show()

def interactive_mode(vmap):
    points = vmap.voxel_centers
    start = None
    goal = None
    
    print("\n" + "="*60)
    print("交互模式")
    print("="*60)
    print("命令:")
    print("  s x y z    - 设置起点")
    print("  g x y z    - 设置终点")
    print("  c x y z    - 检查点是否有效")
    print("  n x y z    - 找最近的体素")
    print("  v          - 可视化当前选择")
    print("  z <value>  - 显示指定z层的点")
    print("  p          - 打印当前配置")
    print("  q          - 退出并输出配置")
    print("="*60)
    
    while True:
        try:
            cmd = input("\n> ").strip().split()
            if not cmd:
                continue
            
            if cmd[0] == 'q':
                break
            elif cmd[0] == 'p':
                print(f"当前起点: {start}")
                print(f"当前终点: {goal}")
            elif cmd[0] == 'v':
                visualize_with_matplotlib(vmap, start, goal)
            elif cmd[0] == 'z' and len(cmd) >= 2:
                z = float(cmd[1])
                margin = float(cmd[2]) if len(cmd) > 2 else 0.5
                mask = np.abs(points[:,2] - z) < margin
                layer = points[mask]
                print(f"z≈{z} (±{margin}) 有 {len(layer)} 个体素")
                if len(layer) > 0:
                    print(f"  X: [{layer[:,0].min():.2f}, {layer[:,0].max():.2f}]")
                    print(f"  Y: [{layer[:,1].min():.2f}, {layer[:,1].max():.2f}]")
                    # 显示几个样本点
                    sample_idx = np.random.choice(len(layer), min(5, len(layer)), replace=False)
                    print("  样本点:")
                    for i in sample_idx:
                        print(f"    {layer[i]}")
            elif cmd[0] in ['s', 'g', 'c', 'n'] and len(cmd) >= 4:
                try:
                    point = np.array([float(cmd[1]), float(cmd[2]), float(cmd[3])])
                except ValueError:
                    print("无效的坐标格式")
                    continue
                
                is_valid, nearest, dist = check_point(point, points)
                
                if cmd[0] == 's':
                    start = point
                    print(f"起点设置为: {point}")
                    print(f"  最近体素: {nearest}, 距离: {dist:.3f}m")
                    print(f"  状态: {'✓ 有效' if is_valid else '✗ 无效 (距离太远)'}")
                elif cmd[0] == 'g':
                    goal = point
                    print(f"终点设置为: {point}")
                    print(f"  最近体素: {nearest}, 距离: {dist:.3f}m")
                    print(f"  状态: {'✓ 有效' if is_valid else '✗ 无效 (距离太远)'}")
                elif cmd[0] == 'c':
                    print(f"检查点: {point}")
                    print(f"  最近体素: {nearest}, 距离: {dist:.3f}m")
                    print(f"  状态: {'✓ 有效' if is_valid else '✗ 无效 (距离太远)'}")
                elif cmd[0] == 'n':
                    print(f"查询点: {point}")
                    print(f"  最近体素: {nearest}")
                    print(f"  距离: {dist:.3f}m")
            else:
                print("未知命令，输入 q 退出")
                
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"错误: {e}")
    
    return start, goal

def main():
    parser = argparse.ArgumentParser(description='起终点选择工具')
    parser.add_argument('--scene', default='spiral', choices=['spiral', 'building', 'plaza'])
    parser.add_argument('--resolution', type=float, default=0.1)
    parser.add_argument('--no-interactive', action='store_true', help='只打印统计信息')
    args = parser.parse_args()
    
    print(f"加载场景: {args.scene}, 分辨率: {args.resolution}")
    vmap = load_scene(args.scene, args.resolution)
    
    print_stats(vmap)
    
    if args.no_interactive:
        return
    
    start, goal = interactive_mode(vmap)
    
    if start is not None and goal is not None:
        print("\n" + "="*60)
        print("输出配置 (复制到 jps_wrapper.py)")
        print("="*60)
        print(f'    "{args.scene}": {{')
        print(f'        "start": np.array([{start[0]:.1f}, {start[1]:.1f}, {start[2]:.1f}]),')
        print(f'        "goal": np.array([{goal[0]:.1f}, {goal[1]:.1f}, {goal[2]:.1f}])')
        print(f'    }},')

if __name__ == "__main__":
    main()
