# Global Navigation — 3D 路径规划 & 拓扑地图系统

面向地面机器人在复杂室内环境（楼梯、坡道、多层建筑）中的导航。包含两个子系统：

| 包 | 功能 | 论文 |
|----|------|------|
| `jps_planner_3d` | 遍历面提取 + 距离场 + 体素级 A* | Beyond Geometry |
| **`topo_nav`** | **持久拓扑房间分割 + 分层规划** | **Beyond Slicing** |

> **如果你要使用分层规划（`topo_nav`），请直接跳到 [topo_nav 快速上手](#topo_nav-快速上手)。**
> `topo_nav` 自包含所有功能，不需要单独启动 `jps_planner_node`。

---

## topo_nav 快速上手

详细文档见 [`src/topo_nav/doc/README.md`](src/topo_nav/doc/README.md)。

### 1. 编译

```bash
cd global_nav_ws
source /opt/ros/humble/setup.bash   # humble 或 jazzy
colcon build --packages-up-to topo_nav --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

> 若之前编译过旧版 `jps_planner_3d`，需先清理：
> ```bash
> rm -rf build/jps_planner_3d install/jps_planner_3d
> ```

### 2. 启动节点

```bash
ros2 launch topo_nav topo_nav.launch.py
```

### 3. 加载地图 → 构建拓扑图 → 规划

```bash
# 加载 PCD 地图
ros2 service call /topo_nav/load_pcd_map nav_interfaces/srv/LoadPcdMap "{
  pcd_path: '/path/to/scene.pcd', resolution: 0.1,
  min_points_per_voxel: 1, max_abs_coord: 100.0}"

# 构建拓扑地图（房间分割 + 楼层检测）
ros2 service call /topo_nav/build_topo_map nav_interfaces/srv/BuildTopoMap "{
  persistence_tau: 3.0, floor_separation_m: 2.5,
  corridor_aspect_ratio: 3.0, staircase_variance_m: 0.5, robot_diameter_m: 0.4}"

# 分层路径规划
ros2 service call /topo_nav/plan_hierarchical nav_interfaces/srv/PlanHierarchical "{
  start: {position: {x: 1.0, y: 2.0, z: 0.1}},
  goal:  {position: {x: 15.0, y: 8.0, z: 3.2}},
  snap_to_surface: true, robot_diameter_m: 0.4}"
```

### 4. 可视化

```bash
ros2 run topo_nav room_visualizer  # 发布彩色房间点云到 RViz
```

---

## jps_planner_3d 快速上手（原始体素级规划）

### Quick Start

### 1. Build Workspace
```bash
cd /home/steam/project/global_nav_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 2. Load Map from OBJ Mesh
```bash
ros2 launch occmap_tools load_obj_mesh.launch.py \
    obj_path:=/path/to/mesh.obj \
    resolution:=0.05
```

### 3. Visualize (Optional)
```bash
# Terminal 1: Distance field visualization
ros2 run occmap_tools distance_field_visualizer

# Terminal 2: RViz
rviz2
# Add topics: /occupied_voxels_cloud, /distance_field_cloud
```

### 4. Plan Path
```bash
ros2 service call /plan_path nav_interfaces/srv/PlanPath "{
  start: {position: {x: 0.0, y: 0.0, z: 0.5}},
  goal: {position: {x: 5.0, y: 3.0, z: 0.5}},
  snap_to_surface: true
}"
```

---

## System Architecture

```
occmap_tools (Python)  →  jps_planner_3d (C++)  →  nav_interfaces (Services)
     │                           │
     ├─ OBJ Mesh Loader          ├─ VoxelMap (Surface extraction)
     ├─ Distance Visualizer      ├─ JPS3D (A* on surfaces)
     └─ Path Client              └─ Distance Field
```

### Key Components

**1. VoxelMap** - Voxel management & surface extraction
- **Surface Definition**: Occupied voxel with clearance above (`surface_clearance_height`)
- **Connectivity**: Surfaces within `connectivity_height_threshold` are reachable
- **Distance Field**: 2-pass Chamfer transform for obstacle distances

**2. JPS3D** - Path planner
- **Algorithm**: A* search constrained to surface voxels
- **Cost Function**: Distance + Z-change penalty + obstacle proximity penalty
- **Heuristic**: 3D Euclidean distance with configurable weight

**3. OBJ Mesh Loader** - Load Matterport3D meshes
- **Surface Voxelization**: Samples triangle surfaces (85-95% fewer voxels than solid fill)
- **Edge Densification**: Ensures connectivity across thin structures
- **Auto-centering**: Optional centering at world origin

---

## ROS 2 Services

### SetVoxelMap
```bash
ros2 service call /set_voxel_map nav_interfaces/srv/SetVoxelMap "{
  resolution: 0.1,
  occupied: [{x: 1.0, y: 2.0, z: 0.0}, ...]
}"
```

### PlanPath
```bash
ros2 service call /plan_path nav_interfaces/srv/PlanPath "{
  start: {position: {x: 0, y: 0, z: 0.5}},
  goal: {position: {x: 5, y: 3, z: 1.0}},
  snap_to_surface: true
}"
```

### GetDistanceField (Visualization)
```bash
ros2 service call /get_distance_field nav_interfaces/srv/GetDistanceField
```

---

## Launch File Parameters

### load_obj_mesh.launch.py

| Parameter | Default | Description |
|-----------|---------|-------------|
| `obj_path` | *required* | Path to .obj file |
| `resolution` | 0.2 | Voxel size (meters) |
| `auto_center` | false | Center mesh at origin |
| `connectivity_height_threshold` | 0.6 | Max step height (meters) |
| `surface_clearance_height` | 0.8 | Required vertical clearance (meters) |
| `distance_field_max_distance` | 5 | Max distance in field (grid units) |
| `max_service_points` | 1000000 | Point limit for service |
| `truncate_service_points` | false | Truncate if over limit |

**Example:**
```bash
ros2 launch occmap_tools load_obj_mesh.launch.py \
    obj_path:=/dataset/mesh.obj \
    resolution:=0.05 \
    connectivity_height_threshold:=0.4 \
    distance_field_max_distance:=10
```

---

## RViz Visualization

### Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/occupied_voxels_cloud` | PointCloud2 | All occupied voxels (white) |
| `/distance_field_cloud` | PointCloud2 | Surface voxels colored by obstacle distance |
| `/planned_path` | Path | Path from PlanPath service |

### Distance Field Colors
- **Blue**: Far from obstacles (safe)
- **Red**: Near obstacles (危险)
- **Gradient**: Interpolated by Manhattan distance

### RViz Setup
1. **Fixed Frame**: `map`
2. **Add PointCloud2**: 
   - `/occupied_voxels_cloud` → Size: 0.03, Color: White
   - `/distance_field_cloud` → Size: 0.05, Color Transformer: RGB8
3. **Add Path**: `/planned_path` → Color: Green

---

## Algorithm Details

### Surface Extraction
```
For each occupied voxel (x,y,z):
  1. Check clearance: Is [z+1, z+surface_clearance_height] free?
  2. Mark as Surface if clearance satisfied
```
- **Purpose**: Only plan on walkable surfaces, not inside walls

### Distance Field (2-Pass Chamfer)
```
Pass 1 (Forward):  Scan ↓→, check left/up neighbors
Pass 2 (Backward): Scan ↑←, check right/down neighbors

For each neighbor:
  Search Z range [z-threshold, z+threshold] for reachable surfaces
  Update distance = min(current, neighbor_dist + 1)
```
- **Complexity**: O(n × 2 × 2) vs old O(n × 7 × 8)
- **Performance**: 3-5× faster than multi-pass convergence

### JPS Path Planning
```
Cost = g(n) + h(n) × heuristic_weight + penalties
  g(n) = Distance from start
  h(n) = Euclidean distance to goal
  
Penalties:
  - Z-change penalty: z_penalty_weight × |Δz|
  - Obstacle penalty: potential_weight / distance_to_obstacle²
```

---

## Performance Tips

### High-Resolution Maps
- Use `resolution:=0.05` for detailed environments
- Increase `distance_field_max_distance` to 10-15 for better obstacle avoidance
- Expect slower voxelization (1-2 min for large meshes)

### Memory Constraints
- Set `max_service_points:=5000000` for large maps
- Enable `truncate_service_points:=true` to cap transmission size
- Distance field uses sparse storage (`unordered_map<size_t, uint8_t>`)

### Planner Tuning
```bash
ros2 param set /jps_planner_node heuristic_weight 1.2  # More aggressive (less optimal)
ros2 param set /jps_planner_node potential_weight 50.0  # Less obstacle avoidance
ros2 param set /jps_planner_node z_penalty_weight 5.0   # Stronger preference for flat paths
```

---

## Troubleshooting

### No topics after launching
**Check nodes:**
```bash
ros2 node list  # Should show /jps_planner_node, /obj_mesh_loader
ros2 topic list  # Should show /occupied_voxels_cloud
```

**Fix:** Ensure workspace sourced: `source install/setup.bash`

### Path planning fails: "Start/Goal not on valid surface"
**Cause:** Points not on extracted surfaces

**Fix:**
1. Use `snap_to_surface: true` in PlanPath request
2. Adjust `surface_clearance_height` (lower = more surfaces)
3. Check voxelization: `ros2 topic echo /occupied_voxels_cloud`

### High memory usage
**Cause:** Too many voxels or dense distance field

**Fix:**
1. Increase `resolution` (0.05 → 0.1)
2. Reduce `distance_field_max_distance` (10 → 5)
3. Enable point truncation in service calls

### RViz shows empty space
**Check topic data:**
```bash
ros2 topic hz /occupied_voxels_cloud
ros2 topic echo /distance_field_cloud --once
```

**Fix:**
1. Verify Fixed Frame is `map`
2. Adjust point size (0.02-0.05)
3. Check Color Transformer = RGB8 for distance field

---

## File Structure

```
global_nav_ws/
├── src/
│   ├── nav_interfaces/          # Service definitions
│   │   └── srv/
│   │       ├── SetVoxelMap.srv
│   │       ├── PlanPath.srv
│   │       └── GetDistanceField.srv
│   │
│   ├── jps_planner_3d/          # C++ planner
│   │   ├── include/jps_planner_3d/
│   │   │   ├── voxel_map.hpp    # VoxelMap class
│   │   │   └── jps_3d.hpp       # JPS3D planner
│   │   └── src/
│   │       ├── voxel_map.cpp    # Surface extraction, distance field
│   │       ├── jps_3d.cpp       # A* implementation
│   │       └── jps_planner_node.cpp  # ROS 2 node
│   │
│   └── occmap_tools/            # Python tools
│       ├── occmap_tools/
│       │   ├── obj_mesh_loader.py         # OBJ → voxels
│       │   ├── distance_field_visualizer.py
│       │   └── plan_path_client.py
│       └── launch/
│           ├── load_obj_mesh.launch.py
│           └── visualize_distance_field.launch.py
│
└── README.md  # This file
```

---

## Development Notes

### Adding New Map Formats
Extend `occmap_tools` with new loader nodes following `obj_mesh_loader.py` pattern:
1. Parse format → extract occupied points
2. Call `/set_voxel_map` service
3. Publish PointCloud2 for visualization

### Custom Cost Functions
Modify `JPS3D::getCost()` in `jps_3d.cpp`:
```cpp
double cost = g_cost + h_cost * heuristic_weight_;
cost += z_penalty_weight_ * std::abs(dz);
cost += potential_weight_ / (distance * distance + 1e-6);
// Add your custom penalty here
return cost;
```

### Distance Field Algorithm
Located in `VoxelMap::computeDistanceFieldOnSurface()` (voxel_map.cpp:448-600):
- Forward pass: lines 470-520
- Backward pass: lines 523-573
- Z-connectivity handling: searches within threshold

---

## Citation & References

**Algorithm**: Surface-constrained A* (JPS - Just Planning on Surface)
- Similar to Jump Point Search but adapted for 3D terrain navigation
- Extracts walkable surfaces from voxel occupancy grids
- Plans only on surfaces to reduce search space

**Dataset**: Matterport3D
- Large-scale RGB-D dataset with 3D meshes
- OBJ format with texture coordinates (ignored by loader)
- Typical room: 50K-200K triangles → 10K-50K voxels (resolution 0.05m)

---

## License

Apache-2.0
