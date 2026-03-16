# topo_nav — 持久拓扑房间分解与分层导航

实现论文 **"Beyond Slicing: Persistent Topology-Grounded Room Decomposition in Multi-Story 3D Environments"** 的完整系统。

---

## 目录

- [系统概述](#系统概述)
- [编译](#编译)
- [使用方式](#使用方式)
  - [方式一：交互式 RViz 工具（推荐）](#方式一交互式-rviz-工具推荐)
  - [方式二：手动调用服务](#方式二手动调用服务)
- [服务接口](#服务接口)
- [参数说明](#参数说明)
- [常见问题](#常见问题)
- [文件结构](#文件结构)

---

## 系统概述

```
输入: 3D 占据栅格 (PCD / voxel list / OBJ mesh)
         │
         ▼
  遍历面 S + 距离场 D  (复用 jps_planner_3d_core)
         │
         ▼
  Algorithm 1: 持久同调 → Merge Tree
         │
         ▼
  Algorithm 2: Merge Tree Collapse → 房间分割
         │
         ▼
  楼层检测 + 楼梯/走廊分类
         │
         ▼
  分层 A*: 楼层图 → 房间图 → 体素级
```

---

## 编译

```bash
cd global_nav_ws

# 若之前编译过旧版 jps_planner_3d（未导出库），先清理
rm -rf build/jps_planner_3d install/jps_planner_3d

colcon build --packages-up-to topo_nav --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

---

## 使用方式

### 方式一：交互式 RViz 工具（推荐）

这是和前一工作 `rviz_benchmark.py` 相同的交互模式，适用于 topo_nav。

#### Terminal 1：启动节点

```bash
# 推荐：ros2 run（服务名不带 namespace 前缀）
ros2 run topo_nav topo_nav_node

# 或者启动节点 + room_visualizer 一体：
ros2 launch topo_nav topo_nav_with_viz.launch.py
# 此时服务名带 /topo_nav/ 前缀
```

#### Terminal 2：启动交互工具

```bash
python3 tools/benchmark/tools/rviz_topo_benchmark.py \
    --obj-path /path/to/scene.obj \
    --resolution 0.1
```

常用参数：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--obj-path` | — | OBJ 文件路径 |
| `--resolution` | 0.1 | 体素分辨率 (m) |
| `--z-conn` | 0.3 | Z 连通阈值 (m)，降低可排除床面等高台 |
| `--clearance` | 1.6 | 站立净空高度 (m)，增大可排除低矮空间 |
| `--tau` | 3.0 | 持久性阈值 τ (体素单位) |
| `--floor-sep` | 2.5 | 楼层间距 H_floor (m) |
| `--robot-diameter` | 0.4 | 机器人直径 (m) |
| `--node-name` | `topo_nav_node` | 配合 launch 启动时改为 `topo_nav/topo_nav_node` |

#### Terminal 3：启动 RViz2

RViz2 中添加：

| 话题 | 类型 | 说明 |
|------|------|------|
| `/topo_benchmark/pointcloud` | PointCloud2 | 原始占据体素（白色） |
| `/topo_benchmark/room_cloud` | PointCloud2 | 房间着色（Color: RGB8） |
| `/topo_benchmark/surface` | PointCloud2 | 可通行面（绿色） |
| `/topo_benchmark/path` | Path | 规划路径 |
| `/topo_benchmark/markers` | MarkerArray | 起/终/种子标记球 |

#### 操作流程

1. **点击种子点**（蓝色球）→ 自动建图 + 构建拓扑地图 + 显示房间着色点云
2. **点击起点**（绿色球）→ 必须在彩色点云上
3. **点击终点**（红色球）→ 自动分层 A* 规划，显示路径
4. 继续点击新的起/终点；Ctrl+C 退出并保存结果

---

### 方式二：手动调用服务

适用于脚本集成或调试。以下命令适用于 `ros2 run topo_nav topo_nav_node`（无 namespace）。

#### Step 1：加载地图

```bash
# 从 PCD 文件加载
ros2 service call /load_pcd_map nav_interfaces/srv/LoadPcdMap "{
  pcd_path: '/path/to/scene.pcd',
  resolution: 0.1,
  min_points_per_voxel: 1,
  max_abs_coord: 100.0,
  seed_point: {x: 0.0, y: 0.0, z: 0.0}
}"

# 或从 OBJ mesh 加载（通过 obj_mesh_loader，自动调用 /set_voxel_map）
ros2 run occmap_tools obj_mesh_loader \
  --ros-args -p obj_path:=/path/to/scene.obj -p resolution:=0.1
```

#### Step 2：构建拓扑地图

```bash
ros2 service call /build_topo_map nav_interfaces/srv/BuildTopoMap "{
  persistence_tau: 3.0,
  floor_separation_m: 2.5,
  corridor_aspect_ratio: 3.0,
  staircase_variance_m: 0.5,
  robot_diameter_m: 0.4
}"
```

#### Step 3：规划

```bash
ros2 service call /plan_hierarchical nav_interfaces/srv/PlanHierarchical "{
  start: {position: {x: 1.0, y: 2.0, z: 0.1}},
  goal:  {position: {x: 15.0, y: 8.0, z: 3.2}},
  snap_to_surface: true,
  robot_diameter_m: 0.4
}"
```

---

## 服务接口

| 服务名（短名） | 类型 | 说明 |
|--------|------|------|
| `set_voxel_map` | SetVoxelMap | 从体素点列表加载地图（含 seed_point） |
| `load_pcd_map` | LoadPcdMap | 从 PCD 文件加载地图 |
| `build_topo_map` | BuildTopoMap | 运行 Algorithm 1+2，构建拓扑地图 |
| `get_room_labels` | GetRoomLabels | 返回每个体素的房间/楼层标签 |
| `plan_hierarchical` | PlanHierarchical | 三级分层 A* 规划 |
| `get_distance_field` | GetDistanceField | 返回遍历面坐标 + 距离场（兼容 jps_planner_node） |

> 启动方式影响服务路径：
> - `ros2 run topo_nav topo_nav_node` → `/set_voxel_map`, `/build_topo_map` ...
> - `ros2 launch topo_nav topo_nav.launch.py` → `/topo_nav/set_voxel_map`, `/topo_nav/build_topo_map` ...

---

## 参数说明

`config/topo_nav.yaml`：

```yaml
topo_nav_node:
  ros__parameters:
    frame_id: "map"

    # 遍历面提取（与 jps_planner_3d 相同）
    connectivity_height_threshold: 0.3   # Z 连通阈值 (m)，降低可排除床等高台
    surface_clearance_height: 1.6        # 站立净空 (m)，增大可排除低矮空间
    distance_field_max_distance: 10      # 距离场截断 (体素)

    # 体素级 A* 权重（与 jps_planner_3d 相同）
    w_up:   2.0
    w_down: 1.0
    w_obs:  0.5

    # 拓扑参数（对应论文符号）
    persistence_tau: 3.0        # τ: 显著性阈值（体素单位）
    floor_separation_m: 2.5     # H_floor: 楼层间距阈值 (m)
    corridor_aspect_ratio: 3.0  # γ: 走廊长宽比阈值
    staircase_variance_m: 0.5   # σ_stair: 楼梯高度标准差 (m)
    robot_diameter_m: 0.4       # 机器人直径 (m)
```

**τ 调参指南**：
- `τ = 3.0`（默认）：适合分辨率 0.1m 的场景
- `τ` 过大 → 所有房间合并为 1 个 → 降低 τ
- `τ` 过小 → 噪声碎片化 → 增大 τ

---

## 常见问题

**Q: build_topo_map 返回 num_rooms: 1**
降低 `persistence_tau`（如 1.0 或 2.0）。

**Q: 床面/桌面被纳入遍历面**
降低 `connectivity_height_threshold`（如 0.15m），使 BFS 无法从地面爬上高台。

**Q: 规划失败 "No passable path"**
两房间间所有通道宽度 < robot_diameter_m。尝试减小 `robot_diameter_m`。

**Q: 编译报错 target 'jps_planner_3d::jps_planner_3d_core' not found**
```bash
rm -rf build/jps_planner_3d install/jps_planner_3d
colcon build --packages-select jps_planner_3d
colcon build --packages-select topo_nav
```

---

## 文件结构

```
topo_nav/
├── CMakeLists.txt
├── package.xml
├── setup.py / setup.cfg       # Python 打包
├── config/
│   └── topo_nav.yaml          # 默认参数
├── doc/
│   └── README.md              # 本文档
├── include/topo_nav/
│   ├── persistence.hpp
│   ├── room_segmenter.hpp
│   ├── floor_detector.hpp
│   └── hierarchical_planner.hpp
├── launch/
│   ├── topo_nav.launch.py            # 单独启动节点（带 namespace）
│   └── topo_nav_with_viz.launch.py  # 节点 + room_visualizer 一体启动
├── src/
│   ├── persistence.cpp        # Algorithm 1
│   ├── room_segmenter.cpp     # Algorithm 2
│   ├── floor_detector.cpp
│   ├── hierarchical_planner.cpp
│   └── topo_nav_node.cpp      # ROS2 服务节点（6 个服务）
├── topo_nav_tools/
│   ├── room_visualizer.py     # 持续发布房间着色点云（独立运行）
│   └── hierarchical_plan_client.py  # 命令行单次规划测试
└── resource/topo_nav
```

**工具对比**：

| 工具 | 用途 |
|------|------|
| `rviz_topo_benchmark.py` | 交互式 RViz 完整流程（加载→seed→规划），带统计 |
| `room_visualizer.py` | 只做持续可视化，不规划 |
| `hierarchical_plan_client.py` | 命令行单次规划，不需要 RViz |
