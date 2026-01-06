import open3d as o3d
import numpy as np
import json
from pathlib import Path

# 1. 加载 OBJ 模型
mesh = o3d.io.read_triangle_mesh("/home/steam/project/dataset/matterport3d/v1/scans/1LXtFkjw3qL/1LXtFkjw3qL/matterport_mesh/b94039b4eb8947bdb9ff5719d9173eae/b94039b4eb8947bdb9ff5719d9173eae.obj")

# 2. 预处理：如果模型太大，可以先缩放或移动到原点
mesh.compute_vertex_normals()

# 3. 从网格生成体素 (方法 A：基于表面)
voxel_size = 0.05  # 体素大小，单位通常为米 (5cm)
voxel_grid = o3d.geometry.VoxelGrid.create_from_triangle_mesh(mesh, voxel_size=voxel_size)

# 4. 转换为 Numpy 数组以便保存为占据地图
# 获取所有占据的体素索引
voxels = voxel_grid.get_voxels()
indices = np.stack([v.grid_index for v in voxels])

# 输出文件名基准
out_base = Path("occupancy_map")

# 保存为 .npy 供后续使用（存的是 Open3D 的 grid_index，非世界坐标）
np.save(str(out_base) + ".npy", indices)

# 同时保存 meta：origin + resolution，便于后续把 index 转世界坐标
origin = np.asarray(getattr(voxel_grid, "origin"), dtype=float).reshape(3).tolist()
meta = {
	"format": "open3d_grid_index",
	"resolution": float(voxel_size),
	"origin": origin,
	"note": "world = origin + (idx + 0.5) * resolution",
}
with open(str(out_base) + "_meta.json", "w", encoding="utf-8") as f:
	json.dump(meta, f, ensure_ascii=False, indent=2)

# 5. 可视化
o3d.visualization.draw_geometries([voxel_grid])
