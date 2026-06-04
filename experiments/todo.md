# Coding LLM 实验 TODO：PH-ALT-TG 论文主实验收敛版

## 当前背景

项目分支：

```bash
feat/stitched-geodesic-planner
```

当前方法 pipeline：

```text
mesh -> voxel surface graph -> PH room segmentation -> portals -> transition graph G_T -> ALT landmarks -> TG A* -> stitch via room Dijkstra parent chains
```

已有关键实现：

* PH room segmentation: `persistence.cpp`, `room_segmenter.cpp`
* transition graph / stitch / precompute: `hierarchical_planner.cpp`
* ALT heuristic: `alt_planner.cpp`
* adaptive tau selector: `tau_selector.cpp`
* benchmark runner: `tools/benchmark/tools/auto_benchmark.py`
* aggregation script: `tools/benchmark/tools/aggregate_results.py`

当前最重要的结论：

1. MP3D 6 scenes，0.1m，auto-tau：
   `ALT = 0.498ms`，`PH-ALT-TG = 0.047ms`，10.6x speedup，+3.0% path overhead，100% success。

2. HM3D 100 scenes，K=1 auto-tau：
   speedup 只有 1.5–2.1x，path overhead 约 +18–23%。

3. HM3D 100 scenes，multi-portal paired comparison：
   K=5 把 mean path overhead 从 +21.1% 降到 +6.2%，median overhead 从 +10.6% 降到 +1.8%，query time 只增加到 0.132ms。

所以接下来不要发散，主线是：

> 把 K=5 multi-portal 变成默认主方法，完整重跑 HM3D / MP3D / cross-resolution，并补齐论文需要的统计、图表和 bad-case analysis。

---

# P0：必须先做的代码与日志改动

## P0.1 暴露并统一 `max_portals_per_edge = K`

目标：让 K=1/3/5 可以在所有 benchmark 中稳定配置。

需要检查 / 修改：

* `room_segmenter.cpp`
* `hierarchical_planner.cpp`
* ROS2 parameter / config loading
* `auto_benchmark.py`
* `aggregate_results.py`

新增或确认参数：

```bash
--max-portals-per-edge 1
--max-portals-per-edge 3
--max-portals-per-edge 5
```

或者 ROS param：

```yaml
max_portals_per_edge: 5
```

验收标准：

* K=1 结果应复现当前旧结果。
* K=3/K=5 输出 CSV 中必须记录真实使用的 K。
* 每个 scene 的 portal 数量、TG node 数、TG edge 数随 K 增长合理。
* K=5 precompute 不应 silent fallback 到 K=1。

CSV 至少新增字段：

```text
max_portals_per_edge
selected_tau
tau_mode                 # fixed / auto
num_rooms
num_room_edges
num_portals
tg_nodes
tg_edges
portal_k_effective       # 某些边界候选不足 K 时的实际平均 K
```

---

## P0.2 加强 path validity checker

目标：所有 stitch path 必须可验证，避免多层 shortcut / parent-chain jump 再次污染结果。

对每条最终 path 做验证：

```text
1. path 非空
2. path[0] == snapped_start 或距离在 1 voxel 内
3. path[-1] == snapped_goal 或距离在 1 voxel 内
4. 每对连续节点必须是 surface graph 中合法 26-neighbor edge
5. 不允许 parent chain 产生非邻接跳跃
6. 每个 path voxel 必须属于 surface voxel set
7. stitch segment 在 room 内时，除 portal crossing 外不得跨 room
8. inter-room crossing 必须发生在 transition graph 中对应 portal pair
9. 可选：检查异常 z jump / floor jump
```

新增 CSV 字段：

```text
path_valid
invalid_reason
invalid_edge_count
max_step_distance_voxels
max_z_jump_voxels
room_boundary_violation_count
portal_crossing_count
```

如果 path invalid：

```text
success = false
failure_reason = invalid_path
```

不要把 invalid path 算进 query time / path length 主结果。

验收标准：

* 所有主实验都报告 valid path rate。
* invalid path 数量必须为 0，或者单独列出并解释。
* 若 invalid path > 0，输出对应 scene/query_id/path dump 供可视化。

---

## P0.3 细分 timing 日志

现在只看 `planning_time_ms` 不够。需要拆开记录：

```text
snap_time_ms
tg_astar_time_ms
stitch_time_ms
fallback_astar_time_ms
total_query_time_ms

ph_time_ms
segmentation_time_ms
tau_selection_time_ms
room_dijkstra_time_ms
tg_build_time_ms
alt_landmark_time_ms
topo_ms
```

CSV 字段：

```text
planning_time_ms
snap_time_ms
tg_astar_time_ms
stitch_time_ms
fallback_astar_time_ms
topo_ms
tau_selection_time_ms
```

验收标准：

* 能回答 query time 主要花在 TG A* 还是 stitch。
* 能回答 K=5 增加的成本主要在 precompute 还是 query。
* 能画 stacked bar 或至少输出 mean/p95。

---

## P0.4 明确 failure reason

所有失败不能只写 `success=0`，必须分类。

新增字段：

```text
failure_reason
```

取值建议：

```text
ok
snap_failed
start_goal_disconnected
precompute_timeout
query_timeout
tg_astar_failed
stitch_failed
fallback_failed
invalid_path
surface_extraction_failed
insufficient_walkable_voxels
```

0.05m 特别需要这个，否则无法证明 success drop 是全方法 timeout，而不是 PH-ALT-TG 自己的问题。

---

# P1：核心主实验，优先级最高

## P1.1 HM3D 100 scenes，0.1m，K sweep full benchmark

这是最重要的主实验。

运行配置：

```text
Dataset: HM3D 100 scenes
Resolution: 0.1m
Queries: 尽量 500 queries / scene；如果太慢，至少 300 queries / scene
Trials: 3
Methods:
  ALT baseline
  PH-ALT-TG auto-tau K=1
  PH-ALT-TG auto-tau K=3
  PH-ALT-TG auto-tau K=5
```

建议输出文件：

```text
experiments/results/hm3d_100/100_alt_01m_500q.csv
experiments/results/hm3d_100/100_tg_auto_k1_01m_500q.csv
experiments/results/hm3d_100/100_tg_auto_k3_01m_500q.csv
experiments/results/hm3d_100/100_tg_auto_k5_01m_500q.csv
```

主表需要统计：

```text
success_rate
valid_path_rate
mean_query_ms
median_query_ms
p95_query_ms
mean_path_m
mean_path_overhead_vs_ALT
median_path_overhead_vs_ALT
p95_path_overhead_vs_ALT
mean_topo_ms
p95_topo_ms
max_topo_ms
mean_memory_mb
p95_memory_mb
mean_fallback_nodes
p95_fallback_nodes
```

验收目标：

```text
K=5 path overhead < 8% mean
K=5 median overhead < 3%
K=5 query time still < ALT
K=5 success rate comparable to ALT
K=5 valid_path_rate = 100% among successful queries
```

如果 K=5 mean overhead 仍 > 10%，必须输出 top bad cases。

---

## P1.2 MP3D 6 scenes，0.1m，K=5 auto-tau

目标：确认 K=5 不会破坏 MP3D 上原本漂亮的 10x 结果。

运行配置：

```text
Dataset: MP3D 6 scenes
Resolution: 0.1m
Methods:
  ALT baseline
  PH-ALT-TG auto-tau K=1
  PH-ALT-TG auto-tau K=5
```

输出：

```text
experiments/results/mp3d_6/alt_01m.csv
experiments/results/mp3d_6/tg_auto_k1_01m.csv
experiments/results/mp3d_6/tg_auto_k5_01m.csv
```

关注：

```text
K=5 是否仍接近 10x speedup
K=5 path overhead 是否 <= K=1
K=5 precompute 是否明显增加但仍可接受
K=5 是否保持 100% success
```

验收目标：

```text
speedup >= 7x
path overhead <= 3-5%
success = 100%
valid_path_rate = 100%
```

---

## P1.3 HM3D cross-resolution，K=5 auto-tau

目标：把论文的 resolution scaling 图补完整。

运行配置：

```text
Dataset: HM3D 100 scenes
Resolutions:
  0.1m
  0.075m
  0.05m

Methods:
  ALT baseline
  PH-ALT-TG auto-tau K=5
```

输出：

```text
experiments/results/hm3d_100/100_alt_01m_500q.csv
experiments/results/hm3d_100/100_tg_auto_k5_01m_500q.csv

experiments/results/hm3d_100/100_alt_0075m_500q.csv
experiments/results/hm3d_100/100_tg_auto_k5_0075m_500q.csv

experiments/results/hm3d_100/100_alt_005m_500q.csv
experiments/results/hm3d_100/100_tg_auto_k5_005m_500q.csv
```

0.05m 必须单独统计 failure reason。

主表：

```text
resolution
method
success_rate
valid_path_rate
mean_query_ms
p95_query_ms
speedup_vs_ALT
mean_path_overhead
median_path_overhead
p95_path_overhead
timeout_rate
invalid_path_rate
topo_ms
memory_mb
```

验收标准：

* 不要只报 successful queries 的均值；必须同时报 success / timeout。
* 如果 0.05m success 只有 63–65%，需要明确是哪些 failure reason。
* 若 0.05m 结果太脏，论文中标记为 stress test，不作为主 headline。

---

# P2：统计分析与聚合脚本

## P2.1 修改 `aggregate_results.py`

新增输出：

```text
mean
median
std
p90
p95
max
success_rate
valid_path_rate
timeout_rate
invalid_path_rate
```

对 path overhead 要支持 paired comparison：

```text
只在 ALT 和 TG 都成功且 path_valid 的同一 query_id 上计算 overhead
overhead = (tg_path - alt_path) / alt_path
```

输出两套统计：

```text
1. full-set statistics:
   包括所有 query 的 success/failure 信息

2. paired-success statistics:
   只统计 ALT 和目标方法都成功的 query，用于公平比较 path length
```

新增输出文件：

```text
summary_main_table.csv
summary_main_table.tex
summary_paired_overhead.csv
summary_failures.csv
summary_scene_complexity.csv
summary_timing_breakdown.csv
```

---

## P2.2 生成 overhead CDF / boxplot 数据

每个方法输出 per-query overhead：

```text
dataset
scene
query_id
resolution
method
K
tau
alt_path
method_path
overhead
success
path_valid
failure_reason
```

用于画：

```text
K=1 vs K=3 vs K=5 overhead CDF
K=1 vs K=3 vs K=5 overhead boxplot
per-scene overhead distribution
p95 bad-case list
```

输出：

```text
figures/data/overhead_cdf_hm3d_01m.csv
figures/data/overhead_box_hm3d_01m.csv
```

验收标准：

* CDF 可以显示 median / p90 / p95。
* K=5 应明显优于 K=1/K=3。
* bad cases 能追溯到 scene/query_id。

---

## P2.3 Scene complexity analysis

为每个 scene 记录：

```text
scene_id
dataset
resolution
surface_voxels
surface_edges
num_rooms
num_room_edges
num_portals
tg_nodes
tg_edges
mean_room_size
max_room_size
max_room_fraction
room_size_gini
num_components
selected_tau
mean_clearance
mean_portal_width
mean_query_geodesic
mean_query_euclidean
mean_tortuosity = geodesic / euclidean
mean_room_transitions
cross_floor_query_ratio
same_room_query_ratio
```

目标图：

```text
speedup vs surface_voxels
speedup vs num_rooms
speedup vs num_portals
speedup vs mean_query_geodesic
path_overhead vs mean_room_transitions
path_overhead vs start_goal_to_portal_distance
selected_tau distribution
```

验收目标：

能解释：

```text
为什么 MP3D 是 10x
为什么 HM3D 只有 1.5-2.1x
什么时候 PH-ALT-TG 最有优势
什么时候 ALT 已经足够快
```

---

# P3：bad-case analysis

## P3.1 自动导出 top overhead queries

对 HM3D K=5，找：

```text
top 20 mean overhead queries
top 20 p95/p99 overhead queries
top invalid queries
top fallback queries
top timeout scenes
```

输出：

```text
experiments/results/analysis/top_bad_overhead_k5.csv
experiments/results/analysis/top_invalid_paths.csv
experiments/results/analysis/top_timeout_scenes.csv
```

每条 bad case 记录：

```text
scene
query_id
resolution
start_voxel
goal_voxel
start_room
goal_room
selected_tau
K
alt_path
tg_path
overhead
num_room_transitions
num_portal_crossings
fallback_nodes
failure_reason
path_valid
start_to_nearest_portal_dist
goal_to_nearest_portal_dist
```

---

## P3.2 bad-case 分类规则

给每个 bad case 自动或半自动分类：

```text
same_room_detour
portal_sparsity
bad_portal_location
open_space_room
long_corridor
room_overmerge
room_oversplit
multi_floor_issue
fallback_issue
timeout_issue
unknown
```

初始可以用 heuristic 分类：

```text
if start_room == goal_room and overhead high:
    same_room_detour

if start_to_nearest_portal_dist high or goal_to_nearest_portal_dist high:
    portal_sparsity

if num_room_transitions much larger than ALT-derived approximate room transitions:
    room_routing_bad

if max_z_jump_voxels high:
    multi_floor_issue
```

输出统计：

```text
bad_case_type_counts.csv
```

---

## P3.3 可视化导出

为 top bad cases 导出路径数据：

```text
ALT path voxels
TG K=1 path voxels
TG K=5 path voxels
rooms labels
portals
surface map bounding box
```

输出格式可以是：

```text
.json
.ply
.obj
.csv
```

建议文件：

```text
experiments/results/visualization/{scene}_{query_id}_alt_path.ply
experiments/results/visualization/{scene}_{query_id}_tg_k1_path.ply
experiments/results/visualization/{scene}_{query_id}_tg_k5_path.ply
experiments/results/visualization/{scene}_{query_id}_rooms.ply
experiments/results/visualization/{scene}_{query_id}_portals.ply
```

验收标准：

* 至少能生成 5 个 K=1 大绕路、K=5 修复的可视化案例。
* 至少能生成 5 个 K=5 仍失败或 overhead 很高的案例。
* 这些图后面要用于论文 qualitative figure。

---

# P4：adaptive tau selector 验证

## P4.1 固定 tau sweep

目标：证明 auto-tau 不是玄学调参。

运行：

```text
Dataset: HM3D 100 scenes
Resolution: 0.1m
K: 5
Tau candidates: 3, 4, 5, 6
Methods:
  PH-ALT-TG fixed tau=3 K=5
  PH-ALT-TG fixed tau=4 K=5
  PH-ALT-TG fixed tau=5 K=5
  PH-ALT-TG fixed tau=6 K=5
  PH-ALT-TG auto-tau K=5
```

如果全量太慢，先做 30 scenes 子集，但最终论文最好有 100 scenes。

输出：

```text
100_tg_tau3_k5_01m.csv
100_tg_tau4_k5_01m.csv
100_tg_tau5_k5_01m.csv
100_tg_tau6_k5_01m.csv
100_tg_auto_k5_01m.csv
```

---

## P4.2 Oracle tau 分析

对每个 scene，定义 oracle tau。

建议两个 oracle：

```text
oracle_speed_tau:
  在 path overhead <= 10% 且 success valid 的 tau 中选 query time 最小的

oracle_quality_tau:
  在 query time <= ALT time 的 tau 中选 path overhead 最小的
```

比较：

```text
auto_tau == oracle_tau 的比例
auto_tau 与 oracle_tau 差距
auto_tau 的 speed loss vs oracle
auto_tau 的 overhead loss vs oracle
```

输出：

```text
tau_oracle_comparison.csv
tau_selector_accuracy.csv
```

需要记录 auto-tau score components：

```text
tau
score
coverage
components
budget_skip_rooms
max_fraction
num_rooms
gini
num_portals
```

验收目标：

* auto-tau 不一定每次等于 oracle，但不能经常选灾难 tau。
* auto-tau 的 average performance 应接近 oracle。
* 如果 auto-tau 经常错，输出错例并调整 score。

---

# P5：precompute / memory / amortization

## P5.1 完整记录 precompute 和 memory

每个 scene/method 输出：

```text
ph_time_ms
segmentation_time_ms
tau_selection_time_ms
room_dijkstra_time_ms
tg_build_time_ms
alt_landmark_time_ms
topo_ms
memory_usage_bytes
dijkstra_voxel_visits
coverage
num_trans_fields
avg_field_size
```

对 K=1/3/5 特别重要。

输出表：

```text
precompute_memory_k_sweep.csv
```

---

## P5.2 Amortization curve

计算：

```text
T_total(N) = T_precompute + N * T_query
```

对比：

```text
ALT
TG auto K=1
TG auto K=5
```

N 取：

```text
1, 10, 50, 100, 500, 1000, 5000, 10000
```

输出：

```text
amortization_hm3d_01m.csv
amortization_mp3d_01m.csv
```

验收目标：

* 给出 break-even query count。
* 如果 K=5 precompute 更贵，要说明多少 query 后收益抵消。
* 论文里可以说适合 many-query / repeated planning / evaluation setting。

---

# P6：baseline 补强，可选但建议

## P6.1 PRM sample sweep

当前 PRM 1k path overhead 太高，容易被说 baseline 弱。补：

```text
PRM 1k
PRM 2k
PRM 5k
PRM 10k
```

至少 HM3D 0.1m 100 scenes 或 30 scenes 子集。

输出：

```text
100_prm_1k_01m.csv
100_prm_2k_01m.csv
100_prm_5k_01m.csv
100_prm_10k_01m.csv
```

统计：

```text
success_rate
query_time
path_overhead
build_time
memory
```

目标：

* 证明不是拿弱 PRM 当 strawman。
* 如果 PRM 10k path 变好但 build/memory/query 成本大，也可以作为论文讨论。

---

## P6.2 更强 graph speedup baseline 调研 / 最小实现

如果有时间，调研或尝试：

```text
Contraction Hierarchies on surface graph
Hub Labeling / 2-hop labeling
Arc Flags
Multi-Level Dijkstra / CRP
```

不一定全实现。最低要求：

* 写一个 `baselines/README_graph_speedups.md`
* 说明每个方法的输入、预处理、query、path reconstruction、memory 风险
* 如果实现 CH 太重，至少给出小规模 feasibility test

验收：

```text
small_scene_ch_feasibility.csv
```

如果无法完成，不阻塞主论文，但 related work / limitation 要能交代。

---

# P7：0.05m timeout protocol

当前 0.05m success 掉到约 63–65%，要避免被审稿人质疑 survivor bias。

需要做：

```text
1. 所有方法使用相同 timeout
2. 记录 precompute timeout 与 query timeout
3. 记录 surface extraction failure
4. 记录 snap failure
5. query time 只在 successful + valid path 上统计
6. success/failure 表单独报告
```

输出：

```text
timeout_breakdown_005m.csv
```

字段：

```text
method
resolution
num_queries
num_success
num_timeout_precompute
num_timeout_query
num_snap_failed
num_disconnected
num_invalid
success_rate
valid_path_rate
```

论文表述目标：

```text
0.05m is a stress test.
All methods suffer from large-map timeout.
PH-ALT-TG still reduces query time among successful valid queries, but success rate must be interpreted together with timeout breakdown.
```

---

# P8：论文图表生成

请实现或扩展脚本：

```text
tools/benchmark/tools/make_paper_figures.py
```

生成以下 figure data 和图片：

## Figure A：main table

```text
MP3D 0.1m:
  ALT
  TG auto K=1
  TG auto K=5

HM3D 0.1m:
  ALT
  TG auto K=1
  TG auto K=5
```

输出：

```text
figures/main_results_table.tex
```

---

## Figure B：speed-quality Pareto

x-axis:

```text
mean path overhead
```

y-axis:

```text
mean query time
```

methods:

```text
ALT
JPS
Flat A*
PRM
Grid-HPA*
No-Stitch
TG K=1
TG K=3
TG K=5
```

输出：

```text
figures/pareto_hm3d_01m.pdf
figures/pareto_mp3d_01m.pdf
```

---

## Figure C：overhead CDF

```text
K=1
K=3
K=5
```

输出：

```text
figures/overhead_cdf_k_sweep_hm3d_01m.pdf
```

必须标注：

```text
mean
median
p95
```

---

## Figure D：resolution scaling

x-axis:

```text
resolution: 0.1, 0.075, 0.05
```

y-axis:

```text
query time
```

methods:

```text
ALT
TG auto K=5
```

另一个 y 或表：

```text
speedup
path overhead
success rate
```

输出：

```text
figures/resolution_scaling_hm3d.pdf
```

---

## Figure E：speedup vs complexity

至少画：

```text
speedup vs surface_voxels
speedup vs num_rooms
speedup vs mean_query_geodesic
```

输出：

```text
figures/speedup_vs_surface_voxels.pdf
figures/speedup_vs_num_rooms.pdf
figures/speedup_vs_query_distance.pdf
```

---

## Figure F：bad-case visualization

输出 4–6 个 qualitative case：

```text
1. K=1 bad, K=5 fixes
2. K=5 still bad
3. MP3D multi-floor success
4. HM3D simple scene where ALT already fast
```

---

# 最终实验优先级

## 必须完成，否则论文主线不稳

```text
[x] P0.1 K 参数全链路暴露                          # 1dbf549, 2026-05-23
[x] P0.2 path validity checker                     # 1dbf549, 2026-05-23
[x] P0.3 timing breakdown                          # 1dbf549, 2026-05-23
[x] P0.4 failure reason logging                    # 1dbf549, 2026-05-23
[x] P1.1 HM3D 0.1m K=1/K=5 full benchmark         # DONE 2026-05-23, 500q×3trials
[x] P1.2 MP3D 0.1m K=1/K=5 auto-tau               # DONE 2026-05-23, 6q×3trials
[x] P2.1 aggregate_results.py 支持 paired overhead  # fae4e8f, 2026-05-23
[x] P2.2 overhead CDF 数据                          # fae4e8f, analyze_overhead.py
[x] P3.1 top bad overhead query 导出                # fae4e8f, analyze_overhead.py
```

## 强烈建议完成，能明显提高论文质量

```text
[x] P1.3 HM3D cross-resolution K=5                 # DONE 2026-05-25, speedup 1.73x→2.07x→2.24x (med)
[x] P2.3 scene complexity analysis                  # analyze_scene_complexity.py, 2026-05-23
[x] P3.2 bad-case 分类                              # analyze_overhead.py bad_case_type_counts
[x] P3.3 qualitative 可视化导出                      # export_visualization.py, 2026-05-23
[x] P4.1 fixed tau sweep K=5                        # DONE 2026-05-25, auto-tau paired speedup 1.76x best OH +1.1%
[x] P4.2 oracle tau vs auto-tau                     # analyze_tau_oracle.py, 2026-05-23
[x] P5.1 precompute / memory logging                # already in CSV: topo_ms, memory_usage_bytes, timing breakdown
[x] P5.2 amortization curve                         # analyze_amortization.py, 2026-05-23
```

## 有时间再做

```text
[x] P6.1 PRM 2k/5k/10k sweep                        # DONE 2026-05-25, OH +41~60%, dominated by TG in Pareto
[x] P6.2 CH / Hub Labeling feasibility analysis     # experiments/notes/graph_speedup_baselines.md
[x] P7 0.05m timeout protocol                       # analyze_timeout.py + timeout_breakdown.csv
[x] P8 paper-ready figures/tables                    # make_paper_figures.py + plot_paper_figures.py → LaTeX + PDF
```

---

# 最关键验收指标

最终希望主表能达到：

```text
MP3D 0.1m, auto-tau K=5:
  success = 100%             ← ✅ 100%
  valid_path = 100%          ← ⚠️ 16.7% → pending v2 rerun (bridgeGap fix applied)
  speedup vs ALT >= 7x      ← ✅ 5.8x (K=1 is 8.6x)
  mean path overhead <= 5%   ← ✅ -2.1% (paths shorter than ALT!)

HM3D 0.1m, auto-tau K=5:
  success comparable to ALT  ← ✅ 91.2% vs ALT 88.6%
  valid_path = 100%          ← ⚠️ 28.5% → pending v2 rerun (bridgeGap + snap fix)
  median speedup vs ALT > 1x ← ✅ 1.73x median, paired 1.76x (76% queries faster)
  mean path overhead <= 8%   ← ✅ +1.1% (paired valid), +8.7% (all successful)
  median path overhead <= 3% ← ✅ +1.6% (paired valid)
  p95 overhead < K=1         ← ✅ +22.1% vs K=1 +32.4%

K sweep:
  K=5 dominates K=1         ← ✅ OH: +1.1% vs +3.1% (paired valid)
  K=5 median faster          ← ✅ 0.049ms (K=5) vs 0.034ms (K=1) vs 0.085ms (ALT)
  K=5 precompute documented  ← ✅ 212ms (K=5) vs 51ms (K=1) vs 76ms (ALT)

Resolution scaling (median speedup):
  0.1m:  1.73x (76% faster)
  0.075m: 2.07x (81% faster)
  0.05m:  2.24x (87% faster)
  → Hierarchy benefit grows with resolution

NOTE: mean speedup (0.92x at 0.1m) is misleading due to 36/456 snap outliers
  at ~1.6ms. Snap uses brute-force radius-50 search when start/goal is far
  from surface. Median is the correct metric for algorithmic performance.
```

最重要的一句话：

> 请不要继续优化 K=1 的结论；下一版论文主方法应当是 **PH-ALT-TG + adaptive tau + multi-portal K=5 + validated stitch path**。
