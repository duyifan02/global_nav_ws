#!/usr/bin/env python3
"""
诊断工具: 检查PCD→体素转换的坐标变换问题

对比Python转换器和C++ VoxelMap的坐标计算,找出不一致之处
"""

import numpy as np
import sys
from pathlib import Path

# Add benchmark to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from converters.pcd_to_voxel import VoxelMap, read_pcd, voxelize_points, filter_outliers


def simulate_cpp_origin_calculation(voxel_centers, resolution, free_space_margin=0.3):
    """
    模拟C++ VoxelMap::updateFromPoints的origin计算逻辑
    
    参考: voxel_map.cpp lines 49-76
    """
    if len(voxel_centers) == 0:
        return None, None, None
    
    # C++ computes min/max from received voxel centers
    min_x, min_y, min_z = voxel_centers.min(axis=0)
    max_x, max_y, max_z = voxel_centers.max(axis=0)
    
    # C++ expansion logic (lines 59-64)
    expansion = free_space_margin
    if expansion < resolution:
        expansion = resolution
    else:
        expansion += resolution
    
    # C++ min_bound calculation (lines 66-70)
    min_bound = np.array([
        np.floor(min_x / resolution) * resolution - expansion,
        np.floor(min_y / resolution) * resolution - expansion,
        np.floor(min_z / resolution) * resolution - expansion
    ])
    
    # C++ max_bound calculation (lines 71-75)
    max_bound = np.array([
        np.ceil(max_x / resolution) * resolution + expansion,
        np.ceil(max_y / resolution) * resolution + expansion,
        np.ceil(max_z / resolution) * resolution + expansion
    ])
    
    # C++ origin = min_bound (line 76)
    origin = min_bound
    
    # C++ grid size (lines 80-83)
    nx = int(np.ceil((max_bound[0] - min_bound[0]) / resolution)) + 1
    ny = int(np.ceil((max_bound[1] - min_bound[1]) / resolution)) + 1
    nz = int(np.ceil((max_bound[2] - min_bound[2]) / resolution)) + 1
    
    return origin, (nx, ny, nz), (min_bound, max_bound)


def simulate_cpp_reindexing(voxel_centers, origin, resolution):
    """
    模拟C++如何将接收到的voxel_centers重新索引到grid
    
    参考: voxel_map.cpp lines 89-98
    """
    reindexed = []
    for center in voxel_centers:
        # C++ line 91-93: ix = floor((x - origin_x) / resolution)
        ix = int(np.floor((center[0] - origin[0]) / resolution))
        iy = int(np.floor((center[1] - origin[1]) / resolution))
        iz = int(np.floor((center[2] - origin[2]) / resolution))
        reindexed.append((ix, iy, iz))
    
    return np.array(reindexed)


def analyze_conversion(pcd_path, resolution=0.5, free_space_margin=0.3):
    """分析完整的转换流程"""
    
    print("="*80)
    print(f"诊断PCD转换: {pcd_path}")
    print(f"分辨率: {resolution}m, Free Space Margin: {free_space_margin}m")
    print("="*80)
    
    # Step 1: Python转换器的处理
    print("\n[Python转换器]")
    vmap = VoxelMap(pcd_path, resolution, min_points=1)
    
    print(f"原始点云数: {len(vmap.points)}")
    print(f"过滤后点云范围:")
    print(f"  Min: {vmap.points.min(axis=0)}")
    print(f"  Max: {vmap.points.max(axis=0)}")
    
    print(f"\n体素化后:")
    print(f"  体素数: {len(vmap.voxel_centers)}")
    print(f"  体素中心范围:")
    print(f"    Min: {vmap.voxel_centers.min(axis=0)}")
    print(f"    Max: {vmap.voxel_centers.max(axis=0)}")
    print(f"  Python计算的bounds:")
    print(f"    Min: {vmap.bounds_min}")
    print(f"    Max: {vmap.bounds_max}")
    print(f"  Grid dimensions: {vmap.dimensions}")
    
    # Step 2: 模拟C++接收到voxel_centers后的处理
    print("\n[C++ VoxelMap模拟]")
    cpp_origin, cpp_grid_size, (cpp_min_bound, cpp_max_bound) = \
        simulate_cpp_origin_calculation(vmap.voxel_centers, resolution, free_space_margin)
    
    print(f"C++计算的origin: {cpp_origin}")
    print(f"C++计算的bounds:")
    print(f"  Min: {cpp_min_bound}")
    print(f"  Max: {cpp_max_bound}")
    print(f"C++计算的grid size: {cpp_grid_size}")
    
    # Step 3: 对比origin差异
    print("\n[坐标系差异分析]")
    print(f"Python假设的origin: [0.0, 0.0, 0.0]")
    print(f"C++计算的origin: {cpp_origin}")
    print(f"Origin偏移量: {cpp_origin - np.array([0.0, 0.0, 0.0])}")
    
    # Step 4: 检查重新索引的影响
    print("\n[重新索引影响]")
    
    # Python的原始voxel indices (假设origin=0)
    python_indices = np.floor(vmap.points / resolution).astype(np.int32)
    unique_python_indices = np.unique(python_indices, axis=0)
    
    # C++会如何重新索引这些voxel centers
    cpp_reindexed = simulate_cpp_reindexing(vmap.voxel_centers, cpp_origin, resolution)
    
    print(f"Python原始索引范围:")
    print(f"  Min: {unique_python_indices.min(axis=0)}")
    print(f"  Max: {unique_python_indices.max(axis=0)}")
    
    print(f"C++重新索引后范围:")
    print(f"  Min: {cpp_reindexed.min(axis=0)}")
    print(f"  Max: {cpp_reindexed.max(axis=0)}")
    
    # Step 5: 检查sample点的变换
    print("\n[样本点变换追踪]")
    sample_indices = [0, len(vmap.voxel_centers)//2, -1]
    
    for i in sample_indices:
        if i >= len(vmap.voxel_centers):
            continue
            
        original_point = vmap.points[i] if i < len(vmap.points) else vmap.points[-1]
        voxel_center = vmap.voxel_centers[i]
        cpp_idx = cpp_reindexed[i]
        
        # C++ gridToWorld: origin + (idx + 0.5) * resolution
        cpp_reconstructed = cpp_origin + (cpp_idx + 0.5) * resolution
        
        print(f"\n样本 {i}:")
        print(f"  原始点: {original_point}")
        print(f"  Python体素中心: {voxel_center}")
        print(f"  C++重新索引: {cpp_idx}")
        print(f"  C++重建坐标: {cpp_reconstructed}")
        print(f"  误差: {voxel_center - cpp_reconstructed}")
    
    # Step 6: 诊断结论
    print("\n" + "="*80)
    print("诊断结论:")
    print("="*80)
    
    max_error = np.abs(vmap.voxel_centers - 
                      (cpp_origin + (cpp_reindexed + 0.5) * resolution)).max()
    
    if max_error < resolution * 0.01:
        print("✓ 坐标变换一致 - 没有发现origin mismatch问题")
        print("  可能的其他原因:")
        print("  - RViz frame_id不匹配")
        print("  - 点云过滤过度")
        print("  - 分辨率设置错误")
    else:
        print("✗ 检测到坐标变换不一致!")
        print(f"  最大误差: {max_error:.6f}m")
        print(f"  Origin偏移: {cpp_origin}")
        print("\n问题原因:")
        print("  Python发送的voxel_centers假设origin=(0,0,0)")
        print("  C++接收后计算新的origin(包含expansion)")
        print("  C++将坐标重新索引到新grid,导致位置偏移")
        print("\n推荐修复方案:")
        print("  方案1: Python在发送前减去expansion后的origin")
        print("  方案2: C++不重新计算origin,直接使用接收到的坐标")
        print("  方案3: 修改协议,发送voxel indices而非world coordinates")
    
    return vmap, cpp_origin, cpp_grid_size


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("用法: python debug_conversion.py <pcd_file> [resolution] [free_space_margin]")
        print("\n示例:")
        print("  python debug_conversion.py ../PCT_planner/rsc/pcd/building2_9.pcd")
        print("  python debug_conversion.py ../PCT_planner/rsc/pcd/plaza3_10.pcd 0.15 0.3")
        sys.exit(1)
    
    pcd_path = sys.argv[1]
    resolution = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5
    free_space_margin = float(sys.argv[3]) if len(sys.argv) > 3 else 0.3
    
    analyze_conversion(pcd_path, resolution, free_space_margin)
