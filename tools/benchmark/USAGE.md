# Benchmark 使用指南

本工具支持两套独立工作流：

| 工作流 | 场景来源 | 用途 |
|--------|----------|------|
| **Workflow A**: PCT 基线对比 | PCD 点云文件 (Spiral/Building/Plaza) | 与 PCT/3D2M 的定量对比测试 |
| **Workflow B**: MP3D 交互式会话 | Matterport3D OBJ 网格文件 | 自定义场景性能统计与截图 |

---

## 环境准备（首次）

```bash
# 编译工作空间
source /opt/ros/jazzy/setup.bash
cd /path/to/global_nav_ws
colcon build --symlink-install

# 以后每次开新终端都需要 source
source /opt/ros/jazzy/setup.bash
source /path/to/global_nav_ws/install/setup.bash
```

---

## Workflow A：PCT 基线对比（PCD 场景）

适用场景：Spiral (P1)、Building (P2)、Plaza (P3)，对应论文 Section IV-B。

### 场景参数

| 场景 | PCD 文件 | 分辨率 | 起点 | 终点 |
|------|---------|--------|------|------|
| `spiral` | `src/spiral0.3_2.pcd` | 0.20 m | (−16, −6, 0) | (−26, −5, 0) |
| `building` | `src/building2_9.pcd` | 0.10 m | (5, 5, 0) | (−6, −1, 0) |
| `plaza` | `src/plaza3_10.pcd` | 0.10 m | (0, 0, 0) | (23, 10, 0) |

### A-1：RViz2 交互式测试（推荐）

**终端 1 — 启动 JPS 节点**
```bash
source /opt/ros/jazzy/setup.bash
source /path/to/global_nav_ws/install/setup.bash
ros2 run jps_planner_3d jps_planner_node
```

**终端 2 — 启动 RViz2**
```bash
source /opt/ros/jazzy/setup.bash
source /path/to/global_nav_ws/install/setup.bash
rviz2
```

RViz2 配置（每次启动后设置一次）：
1. `Fixed Frame` → `map`
2. **Add** → PointCloud2 → Topic: `/benchmark/pointcloud`（原始点云，白色）
3. **Add** → PointCloud2 → Topic: `/benchmark/surface`（可通行区域，绿色）
4. **Add** → Path → Topic: `/benchmark/path`
5. **Add** → MarkerArray → Topic: `/benchmark/markers`
6. 工具栏选择 **Publish Point** 工具（快捷键 `P`）

**终端 3 — 运行 benchmark**
```bash
source /opt/ros/jazzy/setup.bash
source /path/to/global_nav_ws/install/setup.bash
cd /path/to/global_nav_ws/tools/benchmark/tools

# Spiral (r=0.2m)
python3 rviz_benchmark.py --scene spiral --resolution 0.2 --trials 50

# Building (r=0.1m)
python3 rviz_benchmark.py --scene building --resolution 0.1 --trials 50

# Plaza (r=0.1m)
python3 rviz_benchmark.py --scene plaza --resolution 0.1 --trials 50
```

**操作流程（Workflow A）：**
1. 点云自动加载并显示（白色）
2. 用 **Publish Point** 点击选择**建图种子点**（蓝色球）— 确定可行走区域
3. 等待绿色 surface 点云出现（建图完成）
4. 点击选择**导航起点**（绿色球）— 必须选在绿色 surface 上
5. 点击选择**导航终点**（红色球）— 必须选在绿色 surface 上
6. 自动运行 N 次规划，实时打印统计
7. 继续点击开始新一组测试；`Ctrl+C` 退出并保存结果

### A-2：自动化批量测试

```bash
cd /path/to/global_nav_ws/tools/benchmark
python3 run_benchmark.py --planners JPS --scenes spiral --trials 50
python3 run_benchmark.py --planners JPS PCT --scenes all --trials 50
```

| 参数 | 选项 | 默认值 |
|------|------|--------|
| `--planners` | `JPS`, `PCT`, `3D2M`, `all` | `JPS` |
| `--scenes` | `spiral`, `building`, `plaza`, `all` | `spiral` |
| `--trials` | 整数 | `50` |
| `--output` | 目录路径 | `./results` |

---

## Workflow B：MP3D OBJ 交互式会话

适用场景：Matterport3D 场景 S1–S5，直接加载 `.obj` 网格文件，无需 PCD 文件。

**特点**：
- `obj_mesh_loader` 节点自动完成：OBJ 解析 → 体素化 → 调用 `/set_voxel_map`
- `rviz_benchmark.py --preloaded` 直接连接已加载的地图，跳过种子点选择步骤

### B-1：启动（launch 一键启动 JPS + OBJ 加载）

```bash
source /opt/ros/jazzy/setup.bash
source /path/to/global_nav_ws/install/setup.bash

# 基本用法（r=0.1m，自动居中）
ros2 launch occmap_tools load_obj_mesh.launch.py \
    obj_path:=/path/to/scene.obj \
    resolution:=0.1

# 大场景（点数可能超过 500 万，开启截断）
ros2 launch occmap_tools load_obj_mesh.launch.py \
    obj_path:=/path/to/warehouse.obj \
    resolution:=0.1 \
    max_service_points:=5000000 \
    truncate_service_points:=true
```

> **注意**：`launch` 会同时启动 `jps_planner_node` 和 `obj_mesh_loader`，无需单独启动 JPS 节点。

Launch 参数说明：

| 参数 | 说明 | 默认值 |
|------|------|--------|
| `obj_path` | OBJ 文件路径（必填） | — |
| `resolution` | 体素分辨率 (m) | `0.05` |
| `auto_center` | 自动将网格居中到原点 | `true` |
| `free_space_margin` | 自由空间边距 (m) | `2.0` |
| `max_service_points` | 发送到服务的最大点数 | `5000000` |
| `truncate_service_points` | 超出时截断（否则报错退出） | `true` |

等待终端输出：
```
[obj_mesh_loader]: set_voxel_map success: ...
```
说明地图已加载完成，可以启动 benchmark 工具。

### B-2：启动 RViz2

```bash
# 新终端
source /opt/ros/jazzy/setup.bash
source /path/to/global_nav_ws/install/setup.bash
rviz2
```

RViz2 配置（与 Workflow A 相同，但不需要 `/benchmark/pointcloud`）：
1. `Fixed Frame` → `map`
2. **Add** → PointCloud2 → Topic: `/benchmark/surface`（可通行区域，绿色）
3. **Add** → PointCloud2 → Topic: `occupied_voxels`（OBJ 体素，可选）
4. **Add** → Path → Topic: `/benchmark/path`
5. **Add** → MarkerArray → Topic: `/benchmark/markers`
6. 工具栏选择 **Publish Point** 工具

### B-3：运行 benchmark（--preloaded 模式）

```bash
# 新终端
source /opt/ros/jazzy/setup.bash
source /path/to/global_nav_ws/install/setup.bash
cd /path/to/global_nav_ws/tools/benchmark/tools

# scene 参数仅用于结果文件命名，无功能影响
python3 rviz_benchmark.py --preloaded --scene s1 --resolution 0.1 --trials 50
```

**操作流程（Workflow B，比 A 少一步）：**
1. 绿色 surface 点云自动显示（从 JPS 节点拉取）
2. 点击选择**导航起点**（绿色球）— 选在绿色 surface 上
3. 点击选择**导航终点**（红色球）— 选在绿色 surface 上
4. 自动运行 N 次规划，实时打印统计
5. `Ctrl+C` 退出并保存结果

---

## 性能指标说明

| 指标 | 符号 | 说明 |
|------|------|------|
| 预处理时间 | Tp | PCD→体素化耗时（仅 Workflow A） |
| 可通行性估计时间 | Te | buildReachableMap + 距离场构建耗时 |
| 路径搜索时间 | Ts | A* 搜索耗时（C++ 端测量，不含 ROS 通信） |
| 总导航时间 | Tall | Tp + Te + Ts + To |
| 路径长度 | — | 三维欧氏路径长度 (m) |
| 扩展节点数 | Ns | A* 扩展的图节点数 |
| 内存使用 | Size | 体素数（字节估算） |

> `planning_time_ms`（即 Ts）由 C++ 节点内部计时，**不包含 ROS2 服务调用开销**。

---

## 输出文件

结果保存在 `benchmark/results/` 目录：

```
results/
└── jps_benchmark_{scene}_{timestamp}.json
```

JSON 结构：
```json
{
  "planner": "JPS_3D",
  "timestamp": "20260210_143022",
  "map_info": { "scene": "spiral", "resolution": 0.2, "num_voxels": 231884 },
  "config": { "trials_per_run": 50, "snap_to_surface": true },
  "cumulative_stats": {
    "total_runs": 5,
    "total_trials": 250,
    "success_rate": 1.0,
    "planning_time_ms": { "mean": 47.3, "std": 3.1, "min": 41.2, "max": 58.7, "median": 46.9 },
    "path_length_m": { "mean": 28.4, "std": 0.0 },
    "expanded_nodes": { "mean": 1842, "std": 0 }
  },
  "runs": [
    {
      "start": [-16.0, -6.0, 2.8],
      "goal": [-26.0, -5.0, 2.8],
      "euclidean_distance": 10.05,
      "trials": [ { "success": true, "planning_time_ms": 47.3, ... } ]
    }
  ]
}
```

---

## 文件结构

```
benchmark/
├── USAGE.md                   # 本文档
├── run_benchmark.py           # 自动化批量测试入口
├── config.py                  # 场景参数（PCD 路径、起终点、分辨率）
├── metrics.py                 # 指标计算工具
├── converters/
│   └── pcd_to_voxel.py        # PCD → 体素转换（ASCII/Binary PCD）
├── planners/
│   ├── pct_wrapper.py         # PCT 封装（ROS1）
│   ├── jps_wrapper.py         # JPS 封装（ROS2 服务客户端）
│   └── d2m_wrapper.py         # 3D2M 封装（ROS1）
├── tools/
│   ├── rviz_benchmark.py      # RViz2 交互式工具（支持 --preloaded）
│   ├── select_endpoints.py    # 端点选择工具
│   └── debug_conversion.py    # 体素转换调试工具
├── src/
│   ├── spiral0.3_2.pcd        # P1 Spiral 点云
│   ├── building2_9.pcd        # P2 Building 点云
│   └── plaza3_10.pcd          # P3 Plaza 点云
└── results/                   # 输出结果目录（自动创建）
```

对应 ROS2 工作空间节点：

```
global_nav_ws/src/
├── jps_planner_3d/            # JPS A* 规划器（/set_voxel_map, /plan_path 服务）
├── nav_interfaces/            # 服务定义（SetVoxelMap, PlanPath, GetDistanceField）
└── occmap_tools/
    ├── occmap_tools/obj_mesh_loader.py   # OBJ→体素→调用 /set_voxel_map
    └── launch/load_obj_mesh.launch.py    # 一键启动（JPS + OBJ 加载）
```

---

## 常见问题

**Q: `rviz_benchmark.py` 启动后提示服务不可用**
- 确保已启动 JPS 节点（Workflow A）或 launch 文件（Workflow B）
- 检查 `source install/setup.bash` 是否在正确的工作空间执行

**Q: 规划失败 "Planning was cancelled"**
- 起点或终点不在可通行 surface 上
- 将 `snap_to_surface=True`（默认）后仍失败，说明点选在距 surface 过远处
- 在 RViz2 中将点选在**绿色 surface 点云**上

**Q: Workflow B 的 OBJ 加载很慢**
- 大型 OBJ 文件体素化耗时较长（体素数 > 100 万时约需 10–60s）
- 适当增大 `resolution`（如 0.15 m）可显著减少体素数
- 一次加载后 JPS 节点保持地图，无需重复加载

**Q: 结果文件里 Tp=0**
- Preloaded 模式（Workflow B）下 `occupied_list` 来自外部节点，Tp 不记录为 0 是正常的

**Q: 注意事项**
- 使用系统 Python 3.12，**不要使用 conda** 环境（ROS2 依赖系统 Python）
- `planning_time_ms` 是 C++ 端测量的纯算法时间，不含 ROS 服务通信开销
- JPS 是确定性算法：同一起终点多次测试的路径和扩展节点数完全一致，规划时间的微小波动来自 CPU 调度
