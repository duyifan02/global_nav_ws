# topo_nav — 持久拓扑房间分解与分层导航

实现论文 **"Beyond Slicing: Persistent Topology-Grounded Room Decomposition in Multi-Story 3D Environments"** 的完整系统。

---

## 目录

- [系统概述](#系统概述)
- [依赖关系](#依赖关系)
- [编译](#编译)
- [快速上手：完整流程](#快速上手完整流程)
- [服务接口详解](#服务接口详解)
- [参数说明](#参数说明)
- [RViz 可视化](#rviz-可视化)
- [Python 客户端工具](#python-客户端工具)
- [算法说明](#算法说明)
- [常见问题](#常见问题)

---

## 系统概述

```
输入: 3D 占据栅格 (PCD / voxel list)
         │
         ▼
[jps_planner_3d]  → 遍历面图 S + 距离场 D
         │
         ▼
[topo_nav]
  ├─ Algorithm 1: 持久同调 → Merge Tree
  ├─ Algorithm 2: Merge Tree Collapse → 房间分割
  ├─ 楼层检测 + 楼梯/走廊分类
  ├─ Level-1 房间图 G1 + Level-2 楼层图 G2
  └─ 分层 A* 规划 (楼层 → 房间 → 体素)
```

**与前一工作的关系**：`topo_nav` 直接复用 `jps_planner_3d_core` 库的 `VoxelMap`（遍历面提取）和 `JPS3D`（体素级 A*）。你不需要单独启动 `jps_planner_node`，`topo_nav_node` 自包含所有功能。

---

## 依赖关系

```
jps_planner_3d (编译为共享库)
nav_interfaces  (服务消息定义，含3个新 srv)
topo_nav        (本包)
```

编译顺序必须是：`nav_interfaces` → `jps_planner_3d` → `topo_nav`

---

## 编译

```bash
cd global_nav_ws
source /opt/ros/humble/setup.bash   # 或 jazzy

# 首次编译（全部）
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 仅编译本包及其依赖
colcon build --packages-up-to topo_nav --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
```

> **注意**：如果之前已编译过旧版 `jps_planner_3d`，需要先 `rm -rf build/jps_planner_3d install/jps_planner_3d` 再重编，因为 CMakeLists.txt 增加了库导出逻辑。

---

## 快速上手：完整流程

### Terminal 1：启动节点

```bash
source install/setup.bash
ros2 launch topo_nav topo_nav.launch.py
```

启动后看到：
```
[topo_nav_node]: TopoNav node ready. Call set_voxel_map or load_pcd_map, then build_topo_map.
```

---

### Step 1：加载地图

**方式 A — 从 PCD 文件加载**（推荐用于 Matterport3D 数据）

```bash
ros2 service call /topo_nav/load_pcd_map nav_interfaces/srv/LoadPcdMap "{
  pcd_path: '/path/to/scene.pcd',
  resolution: 0.1,
  min_points_per_voxel: 1,
  max_abs_coord: 100.0,
  seed_point: {x: 0.0, y: 0.0, z: 0.0}
}"
```

返回示例：
```yaml
success: true
message: PCD map loaded.
raw_point_count: 1500000
occupied_voxel_count: 45000
surface_voxel_count: 34530     # |S| 遍历面体素数
traversability_time_ms: 312.4
```

**方式 B — 直接传入占据体素列表**

```bash
ros2 service call /topo_nav/set_voxel_map nav_interfaces/srv/SetVoxelMap "{
  resolution: 0.1,
  occupied: [{x: 1.0, y: 0.0, z: 0.0}, {x: 1.0, y: 0.1, z: 0.0}],
  seed_point: {x: 0.0, y: 0.0, z: 0.0}
}"
```

**方式 C — 复用 occmap_tools 加载 OBJ mesh（推荐用于本地测试）**

```bash
# Terminal 2: 加载 OBJ 并发布体素到 /topo_nav/set_voxel_map
ros2 run occmap_tools obj_mesh_loader \
  --ros-args \
  -p obj_path:=/path/to/scene.obj \
  -p resolution:=0.1 \
  -p target_service:=/topo_nav/set_voxel_map
```

---

### Step 2：构建拓扑地图

```bash
ros2 service call /topo_nav/build_topo_map nav_interfaces/srv/BuildTopoMap "{
  persistence_tau: 3.0,
  floor_separation_m: 2.5,
  corridor_aspect_ratio: 3.0,
  staircase_variance_m: 0.5,
  robot_diameter_m: 0.4
}"
```

返回示例：
```yaml
success: true
message: Topo map built.
num_rooms: 18          # 识别到的房间数
num_floors: 2          # 楼层数
num_staircases: 1      # 楼梯间数
num_corridors: 3       # 走廊数
build_time_ms: 45.2    # 含 Algorithm 1 + 2 + 楼层检测
```

> **参数调整指南**：
> - `persistence_tau`：增大 → 更少房间（合并小区域）；减小 → 更多房间（对噪声更敏感）。推荐从 `3.0` 开始。
> - `floor_separation_m`：低层高建筑可降至 `2.0`。
> - `corridor_aspect_ratio`：房间长宽比超过此值判为走廊（默认 `3.0`）。
> - `staircase_variance_m`：房间内 z 坐标标准差超过此值判为楼梯（默认 `0.5`）。

---

### Step 3：分层路径规划

```bash
ros2 service call /topo_nav/plan_hierarchical nav_interfaces/srv/PlanHierarchical "{
  start: {position: {x: 1.0, y: 2.0, z: 0.1}},
  goal:  {position: {x: 15.0, y: 8.0, z: 3.2}},
  snap_to_surface: true,
  robot_diameter_m: 0.4
}"
```

返回示例（跨楼层规划）：
```yaml
success: true
message: OK
path_length_m: 23.4
planning_time_ms: 18.3
  floor_planning_ms: 0.02     # Step 1: 楼层图 A*
  room_planning_ms:  0.15     # Step 2: 房间图 A*
  voxel_planning_ms: 18.1     # Step 3: 体素级 A*（各房间分段）
room_sequence: [3, 7, 12, 15] # 经过的房间 id
floor_sequence: [0, 1]        # 经过的楼层 id
voxel_nodes_expanded: 4210
path: ...                     # nav_msgs/Path，直接在 RViz 中显示
```

---

## 服务接口详解

| 服务名 | 功能 |
|--------|------|
| `/topo_nav/set_voxel_map` | 从体素点列表加载地图 |
| `/topo_nav/load_pcd_map` | 从 PCD 文件加载地图 |
| `/topo_nav/build_topo_map` | 运行 Algorithm 1+2，构建拓扑地图 |
| `/topo_nav/get_room_labels` | 返回每个体素的房间/楼层标签（用于可视化） |
| `/topo_nav/plan_hierarchical` | 三级分层 A* 规划 |

> 前两个服务与 `jps_planner_node` 的接口完全相同，可直接复用现有的加载工具。

---

## 参数说明

参数文件位于 `config/topo_nav.yaml`：

```yaml
topo_nav_node:
  ros__parameters:
    frame_id: "map"

    # 遍历面提取（与 jps_planner_3d 相同）
    connectivity_height_threshold: 0.3   # 可跨越台阶高度 (m)
    surface_clearance_height: 1.6        # 机器人站立净空 (m)
    distance_field_max_distance: 10      # 距离场最大值 (体素)

    # 体素级 A* 代价权重
    w_up:   2.0   # 上坡惩罚
    w_down: 1.0   # 下坡惩罚
    w_obs:  0.5   # 贴墙惩罚

    # 拓扑地图参数
    persistence_tau: 3.0          # 显著性阈值 (体素)，对应论文 τ
    floor_separation_m: 2.5       # 楼层间距阈值 (m)，对应论文 H_floor
    corridor_aspect_ratio: 3.0    # 走廊长宽比阈值，对应论文 γ
    staircase_variance_m: 0.5     # 楼梯高度标准差 (m)，对应论文 σ_stair
    robot_diameter_m: 0.4         # 机器人直径 (m)，用于通道宽度过滤
```

---

## RViz 可视化

### 启动可视化节点

```bash
# Terminal 2: 启动房间标签可视化
ros2 run topo_nav room_visualizer \
  --ros-args -p publish_rate_hz:=2.0
```

发布以下话题（在 RViz 添加 PointCloud2）：

| 话题 | 内容 |
|------|------|
| `/topo_nav/room_cloud` | 按房间 ID 着色（每个房间一个颜色，黄金角间距） |
| `/topo_nav/floor_cloud` | 按楼层着色（0层红，1层绿，2层蓝…） |
| `/topo_nav/persist_cloud` | 按房间中心 D 值着色（蓝=低净空，红=高净空） |

### RViz 快速配置

1. Fixed Frame: `map`
2. Add → PointCloud2 → Topic: `/topo_nav/room_cloud`
   - Size: `0.05`, Color Transformer: `RGB8`
3. Add → Path → Topic: `/topo_nav/plan_hierarchical`（规划完成后自动更新）

规划路径话题在每次调用 `plan_hierarchical` 服务后会自动发布（通过 `response.path`，可在 Python 客户端订阅）。

---

## Python 客户端工具

### 分层规划一键测试

```bash
ros2 run topo_nav hierarchical_plan_client \
  --ros-args \
  -p start_x:=1.0 -p start_y:=2.0 -p start_z:=0.1 \
  -p goal_x:=15.0 -p goal_y:=8.0  -p goal_z:=3.2 \
  -p robot_diameter_m:=0.4

# 若想同时触发 BuildTopoMap（调试用）
ros2 run topo_nav hierarchical_plan_client \
  --ros-args \
  -p build_first:=true \
  -p persistence_tau:=3.0 \
  -p start_x:=1.0 -p start_y:=2.0 -p start_z:=0.1 \
  -p goal_x:=15.0 -p goal_y:=8.0  -p goal_z:=3.2
```

输出示例：
```
── Hierarchical Plan ──────────────────────────────
  Status       : OK
  Path length  : 23.4 m
  Waypoints    : 312
  Room sequence: [3, 7, 12, 15]
  Floor seq    : [0, 1]
  Time total   : 18.347 ms
    Floor step : 0.021 ms
    Room step  : 0.153 ms
    Voxel step : 18.173 ms
  Voxel nodes  : 4210
──────────────────────────────────────────────────
```

---

## 算法说明

### Algorithm 1 — 持久同调（persistence.cpp）

对遍历面 $\mathcal{S}$ 上的距离场 $D$ 做 0 维持久同调：

1. 按 $D$ 降序扫描（同值按坐标字典序排列，确保每个平台区只有一个极大值）
2. Union-Find 追踪超水平集连通分量（按出生值合并，高出生值保持根节点）
3. 新极大值 → Merge Tree 叶子；鞍点 → 合并事件，记录持久度 `pers = birth - death`
4. 每个体素记录 `raw_label` = 最先到达它的极大值的节点索引

时间复杂度：$O(|\mathcal{S}| \log |\mathcal{S}|)$（排序主导）

### Algorithm 2 — Merge Tree Collapse（room_segmenter.cpp）

1. 筛选显著极大值：$\mathrm{pers} > \tau$（含根节点，持久度 $= \infty$）
2. 从根节点出发 BFS 遍历 Merge Tree：显著节点 → 自己是一个房间；非显著节点 → 继承父节点的房间
3. 每个体素 $s$：`room_label[s] = room_of[raw_label[s]]`，边界自然落在距离场的鞍点处
4. 扫描所有边，计算房间邻接、通道宽度、过渡点（两侧最大 $D$ 体素的世界坐标中点）

### 楼层检测（floor_detector.cpp）

- 各房间计算中位高度 $\bar{z}_i$
- 对 $\bar{z}_i$ 排序后做 1D gap 检测（间距 $\geq H_{\text{floor}}$ → 新楼层）
- 楼梯间：高度方差 $> \sigma_{\text{stair}}^2$（优先判断）
- 走廊：PCA 长宽比 $> \gamma$（楼梯间判完再判）

### 分层 A*（hierarchical_planner.cpp）

```
Step 1: 楼层图 A*  → 选择经过哪些楼层/楼梯间
Step 2: 房间图 A*  → 选择房间序列，代价 = 质心→过渡点→质心（欧氏距离）
                     过滤通道宽度 < robot_diameter 的边
Step 3: 体素级 A*  → 在每个房间内部单独跑 A*，受限于该房间的体素子集
                     代价函数与 JPS3D 相同（欧氏 + 高度惩罚 + 贴墙惩罚）
```

**完备性**：房间划分是 $\mathcal{S}$ 的完全分割，过渡点在边界上，因此分层规划的完备性等价于平面 A* 的完备性。

---

## 常见问题

### Q: `build_topo_map` 返回 `num_rooms: 1`，整个场景只有一个房间

**原因**：`persistence_tau` 过大，所有次级极大值都被过滤掉了。

**解决**：降低 `persistence_tau`（如 `1.0` 或 `2.0`）再试。也可以先调用 `get_room_labels` 看持久度分布。

---

### Q: 规划失败："No passable path"

**原因**：所有连通两个房间的通道宽度都小于 `robot_diameter_m`。

**解决**：
1. 减小 `robot_diameter_m`
2. 检查 `get_room_labels` 的 `room_persistence` 字段，确认 D 场是否正确
3. 确认两点确实在同一连通分量内（`jps_planner_node` 的平面 A* 能规划通吗？）

---

### Q: 规划失败："Cannot snap start/goal to surface"

**原因**：起/终点距离最近遍历面超过 5m（snap 半径）。

**解决**：手动传入已在遍历面上的坐标，或先用 `get_room_labels` 返回的 `surface_points` 确认可用坐标范围。

---

### Q: 楼层检测错误（`num_floors: 1`，但应该是多层）

**原因**：`floor_separation_m` 过大，两层楼中位高度差不足。

**解决**：降低 `floor_separation_m`（如 `2.0`）。注意此参数比较的是**房间中位高度**，不是楼层底面高度差。

---

### Q: 编译报错 `target 'jps_planner_3d::jps_planner_3d_core' not found`

**原因**：`jps_planner_3d` 旧编译缓存没有导出库目标。

**解决**：
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
├── config/
│   └── topo_nav.yaml          # 默认参数
├── doc/
│   └── README.md              # 本文档
├── include/topo_nav/
│   ├── persistence.hpp        # PersistenceResult, MergeTreeNode
│   ├── room_segmenter.hpp     # RoomGraph, RoomNode, RoomEdge
│   ├── floor_detector.hpp     # FloorGraph, FloorNode, FloorEdge
│   └── hierarchical_planner.hpp  # HierarchicalPlanner, HierarchicalPath
├── launch/
│   └── topo_nav.launch.py
├── src/
│   ├── persistence.cpp        # Algorithm 1
│   ├── room_segmenter.cpp     # Algorithm 2
│   ├── floor_detector.cpp     # 楼层/楼梯/走廊检测
│   ├── hierarchical_planner.cpp  # 三级 A*
│   └── topo_nav_node.cpp      # ROS2 服务节点
├── topo_nav_tools/
│   ├── room_visualizer.py     # RViz 彩色房间点云
│   └── hierarchical_plan_client.py  # 一键规划测试客户端
└── setup.py
```
