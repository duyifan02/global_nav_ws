"""
PCD点云转体素格式转换器

将PCD点云文件转换为JPS规划器需要的体素格式:
- 输出占据体素的世界坐标列表
- 支持指定分辨率的体素化
"""

import numpy as np
import struct
from typing import List, Tuple, Optional
from pathlib import Path


def read_pcd(pcd_path: str) -> np.ndarray:
    """
    读取PCD文件，返回点云数组
    支持ASCII和binary格式
    
    Args:
        pcd_path: PCD文件路径
        
    Returns:
        Nx3 numpy数组，每行是(x, y, z)坐标
    """
    with open(pcd_path, 'rb') as f:
        # 解析头部
        header = {}
        while True:
            line = f.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith('DATA'):
                data_type = line.split()[1]
                break
            if ' ' in line:
                key, value = line.split(' ', 1)
                header[key] = value
        
        num_points = int(header.get('POINTS', 0))
        fields = header.get('FIELDS', 'x y z').split()
        sizes = [int(s) for s in header.get('SIZE', '4 4 4').split()]
        types = header.get('TYPE', 'F F F').split()
        counts = [int(c) for c in header.get('COUNT', ' '.join(['1'] * len(fields))).split()]
        
        # 找到x, y, z字段的索引和偏移
        x_idx = fields.index('x') if 'x' in fields else 0
        y_idx = fields.index('y') if 'y' in fields else 1
        z_idx = fields.index('z') if 'z' in fields else 2
        
        # 计算每个字段的实际字节宽度 (SIZE * COUNT)
        field_byte_widths = [s * c for s, c in zip(sizes, counts)]
        
        # 计算每个点的字节大小
        point_size = sum(field_byte_widths)
        
        if data_type == 'ascii':
            # ASCII格式
            points = []
            for line in f:
                parts = line.decode('utf-8', errors='ignore').strip().split()
                if len(parts) >= 3:
                    try:
                        x = float(parts[x_idx])
                        y = float(parts[y_idx])
                        z = float(parts[z_idx])
                        points.append([x, y, z])
                    except (ValueError, IndexError):
                        continue
            return np.array(points, dtype=np.float64)
        
        else:  # binary
            # 二进制格式
            data = f.read()
            points = []
            
            # 计算x, y, z的偏移量
            offsets = [sum(field_byte_widths[:i]) for i in range(len(field_byte_widths))]
            x_offset = offsets[x_idx]
            y_offset = offsets[y_idx]
            z_offset = offsets[z_idx]
            
            for i in range(num_points):
                base = i * point_size
                if base + point_size > len(data):
                    break
                
                # 读取float32值
                x = struct.unpack('f', data[base + x_offset:base + x_offset + 4])[0]
                y = struct.unpack('f', data[base + y_offset:base + y_offset + 4])[0]
                z = struct.unpack('f', data[base + z_offset:base + z_offset + 4])[0]
                
                points.append([x, y, z])
            
            return np.array(points, dtype=np.float64)


def voxelize_points(points: np.ndarray, 
                    resolution: float,
                    min_points_per_voxel: int = 1) -> np.ndarray:
    """
    将点云体素化
    
    Args:
        points: Nx3点云数组
        resolution: 体素分辨率(米)
        min_points_per_voxel: 体素内最少点数阈值
        
    Returns:
        Mx3体素中心坐标数组
    """
    if len(points) == 0:
        return np.array([])
    
    # 计算体素索引
    voxel_indices = np.floor(points / resolution).astype(np.int32)
    
    # 去重并统计每个体素的点数
    unique_voxels, counts = np.unique(voxel_indices, axis=0, return_counts=True)
    
    # 过滤掉点数不足的体素
    valid_mask = counts >= min_points_per_voxel
    valid_voxels = unique_voxels[valid_mask]
    
    # 转换为体素中心的世界坐标
    voxel_centers = (valid_voxels.astype(np.float64) + 0.5) * resolution
    
    return voxel_centers


def filter_outliers(points: np.ndarray, 
                    max_abs_coord: float = 200.0) -> np.ndarray:
    """
    过滤点云中的离群点
    
    仅移除坐标绝对值超出合理范围的点（传感器错误等）。
    不做百分位裁剪，以保留完整的地图边界。
    
    Args:
        points: Nx3点云数组
        max_abs_coord: 单个坐标绝对值的最大允许范围(米)
        
    Returns:
        过滤后的点云数组
    """
    if len(points) == 0:
        return points
    
    # 移除坐标超出绝对范围的点
    mask = np.all(np.abs(points) <= max_abs_coord, axis=1)
    return points[mask]

def pcd_to_occupied_voxels(pcd_path: str, 
                           resolution: float,
                           min_points_per_voxel: int = 1) -> List[Tuple[float, float, float]]:
    """
    将PCD文件转换为占据体素坐标列表(JPS SetVoxelMap格式)
    
    Args:
        pcd_path: PCD文件路径
        resolution: 体素分辨率
        min_points_per_voxel: 体素内最少点数阈值
        
    Returns:
        占据体素中心坐标列表 [(x,y,z), ...]
    """
    points = read_pcd(pcd_path)
    voxel_centers = voxelize_points(points, resolution, min_points_per_voxel)
    return [tuple(v) for v in voxel_centers]


def get_bounds(points: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """获取点云边界"""
    return points.min(axis=0), points.max(axis=0)


class VoxelMap:
    """体素地图类，用于JPS规划器"""
    
    def __init__(self, pcd_path: str, resolution: float, min_points: int = 1):
        self.pcd_path = pcd_path
        self.resolution = resolution
        self.points = read_pcd(pcd_path)
        # 过滤离群点避免边界爆炸
        self.points = filter_outliers(self.points, max_abs_coord=200.0)
        self.voxel_centers = voxelize_points(self.points, resolution, min_points)
        self.bounds_min, self.bounds_max = get_bounds(self.voxel_centers)
        
    @property
    def occupied_list(self) -> List[Tuple[float, float, float]]:
        """返回JPS SetVoxelMap需要的格式"""
        return [tuple(v) for v in self.voxel_centers]
    
    @property
    def num_voxels(self) -> int:
        return len(self.voxel_centers)
    
    @property
    def dimensions(self) -> Tuple[int, int, int]:
        """返回体素网格维度"""
        dim = np.ceil((self.bounds_max - self.bounds_min) / self.resolution).astype(int)
        return tuple(dim)


# benchmark/src/ 目录下的PCD文件（路径相对于 benchmark/ 根目录）
_BENCHMARK_DIR = Path(__file__).parent.parent

PCD_PATHS = {
    "spiral": "src/spiral0.3_2.pcd",
    "building": "src/building2_9.pcd",
    "plaza": "src/plaza3_10.pcd",
}


def get_voxel_map(scene_name: str, resolution: float, project_root: str = None) -> VoxelMap:
    """
    获取指定场景的体素地图

    Args:
        scene_name: 场景名 (spiral/building/plaza)
        resolution: 体素分辨率
        project_root: benchmark根目录，默认自动推断

    Returns:
        VoxelMap对象
    """
    if project_root is None:
        project_root = str(_BENCHMARK_DIR)

    pcd_rel_path = PCD_PATHS.get(scene_name.lower())
    if pcd_rel_path is None:
        raise ValueError(f"Unknown scene: {scene_name}")

    pcd_path = str(Path(project_root) / pcd_rel_path)
    return VoxelMap(pcd_path, resolution)


if __name__ == "__main__":
    # 测试
    import sys
    if len(sys.argv) > 1:
        pcd_file = sys.argv[1]
        resolution = float(sys.argv[2]) if len(sys.argv) > 2 else 0.1
        
        vmap = VoxelMap(pcd_file, resolution)
        print(f"Points: {len(vmap.points)}")
        print(f"Voxels: {vmap.num_voxels}")
        print(f"Bounds: {vmap.bounds_min} -> {vmap.bounds_max}")
        print(f"Dimensions: {vmap.dimensions}")
