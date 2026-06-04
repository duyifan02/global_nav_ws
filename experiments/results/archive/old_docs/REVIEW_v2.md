下面只收敛到 **PH-ALT-TG 这条线**。本周不要做 FDM / MPPI 集成；FDM 属于平台动力学和局部安全控制，和当前 bottleneck（τ、fallback、path overhead、ALT 对比）不是同一个层级问题。你现在最该把“per-scene oracle τ 能让 TG 在 83% HM3D scenes 胜 ALT”变成一个可解释、非作弊、可复现实验结论；同时把 fallback 和 +16–18% path overhead 降下来。这个优先级基于你当前 PH-ALT-TG 结果、旧版 full method 被 ALT 压制的 ablation、以及你给出的论文问题设定。    

# 一、接下来一周最应该做的 5 件事

## Priority 1 — 先把 fallback、timing、heuristic 全部可解释化

**要改哪段代码 / 算法**

改 query pipeline 和 stitch pipeline 的日志接口，不先改大算法。重点插桩：

* `queryPHAltTG(...)`
* virtual start / goal connection
* `stitchPath(...)`
* room transfer table lookup
* fallback voxel A* call site
* transition-graph A* open/closed bookkeeping
* landmark heuristic evaluation
* floor-constrained room Dijkstra

同时修两个 correctness 问题：

1. **0.05m multi-floor stitch bug**：room Dijkstra 必须限制在同一 room 且同一 floor-valid component 内，不能通过 voxel adjacency 走跨楼层 shortcut。
2. **landmark count anomaly**：同一 τ 下 L=8 比 L=4/L=16 慢很多，说明 A* tie-breaking、heuristic consistency、fallback interaction 或 goal landmark distance 处理有 bug 风险。

**要记录哪些新日志**

每个 query 一行，必须包含：

```text
scene_id, query_id, tau, landmarks, resolution
|V_surface|, |E_surface|
rooms, portals, |V_T|, |E_T|
start_room, goal_room, same_room
room_hop_distance, transition_hops
tg_astar_expanded, tg_astar_reopened
surface_alt_expanded
fallback_called, fallback_nodes, fallback_ms
fallback_reason
  = missing_transfer_pair
  | skipped_room_budget
  | endpoint_connection_missing
  | disconnected_TG
  | same_room_no_direct_edge
  | floor_boundary_violation
  | path_reconstruction_failure
stitch_segments, stitch_ms
start_connect_ms, goal_connect_ms
tg_astar_ms, total_query_ms
path_len, path_cost
abstract_TG_cost, stitched_surface_cost
cache_hit_all_segments
h_start, true_TG_dist_start_goal_if_available
h_start / true_TG_dist
selected_portal_count_start_room
selected_portal_count_goal_room
```

额外记录每个 τ 的 scene-level precompute logs：

```text
max_room_size
max_room_portals k_max
max k^2 |room|
num_budget_skipped_rooms
skipped_room_voxel_fraction
transfer_pair_coverage
TG_connected_components
query_pair_reachability
```

**要重跑哪些实验**

只重跑最小诊断集：

* MP3D 6 scenes, 0.1m, τ={3,4,5}, landmarks={4,8,16}
* HM3D slow 12 scenes, τ={3,5}, landmarks=8 或 16
* HM3D 10 minival scenes, 0.05m, τ={3,4,5,6}

**要画哪些图**

1. `fallback_nodes` vs `query_time` scatter
2. 每个 scene 的 stacked timing bar：start connection / TG A* / stitch / fallback
3. landmark count vs expanded nodes vs fallback nodes
4. τ vs `transfer_pair_coverage` / `skipped_room_fraction` / query time
5. multi-floor bug 修复前后路径 overlay，只画 S4/S6 或 HM3D multi-floor case

**失败判据：说明这条路线不值得继续的实验**

如果完成插桩后仍然出现：

* L=8 比 L=4/L=16 慢，但 `tg_astar_expanded`、fallback、tie-breaking 都解释不了；
* 或者 `cache_hit_all_segments=true` 时 query time 仍然经常 > ALT；
* 或者 floor fix 后 0.05m 多楼层 speedup 大幅消失；

那说明当前实现结果不可信，不能继续往论文主线推。

**好结果：可以直接成为论文核心卖点**

如果日志显示：

* slow cases 主要由少数 fallback cause 解释；
* τ=3 能救 10/12 slow cases 的原因可由 `skipped_room_fraction` / `max k²|room|` 预测；
* cache-hit 时 PH-ALT-TG 稳定 <0.1ms；

那可以写成核心诊断图：**PH-ALT-TG 的优势不是偶然，而是来自 transition graph search + cache-hit stitch；失败来自可预测的 cache coverage risk。**

---

## Priority 2 — 重写 transfer cache budget，先消灭 fallback

**要改哪段代码 / 算法**

当前最大风险是“large room budget skip → online voxel fallback”。本周最值得改的是 cache policy。

把现在的：

[
\text{skip room if } k^2 |R_i| > B
]

改成两层策略：

### 2.1 不再因为 pair path storage 跳过整间 room

不要 materialize 所有 portal-pair paths。只存每个 portal-root 的 Dijkstra distance / parent tree：

[
\text{storage}(R_i) \approx k_i |R_i|
]

而不是：

[
k_i^2 |R_i|
]

portal-pair path 在 stitch 时从 parent tree traceback 得到：

[
P(p_a,p_b)=TraceBack_{\pi_{p_a}}(p_b)
]

这会直接降低 `skipped_room_budget`。

### 2.2 对仍然超 budget 的房间做 bounded fallback，而不是 full voxel fallback

对超大 room：

* 至少预计算 top-M portals 的 root Dijkstra；
* 其他 portal pair 用 lazy cache：第一次遇到时计算一次并写入 cache；
* query time 表里单独报告 lazy miss，不和正常 online query 混在一起；
* paper 主结果用 warmed cache 或 report cold/warm 两列。

**要记录哪些新日志**

```text
room_id
room_size |R_i|
incident_portals k_i
old_budget_ratio = k_i^2 |R_i| / B
new_budget_ratio = k_i |R_i| / B_root
root_dijkstra_count
pair_paths_materialized_count
lazy_cache_miss_count
lazy_cache_hit_count
skipped_by_old_policy
skipped_by_new_policy
```

**要重跑哪些实验**

* HM3D 12 slow cases, τ=5 old cache vs new cache
* HM3D 100 scenes, τ=5 old cache vs new cache
* MP3D τ sweep, especially τ=1,3,4,5,8
* 0.05m minival τ=5，因为这里 path overhead 和 large room 问题最明显

**要画哪些图**

1. old budget ratio vs fallback nodes
2. old cache vs new cache query-time CDF
3. fallback rate before/after
4. p95 query time before/after
5. memory / precompute increase vs query-time reduction

**失败判据**

如果新 cache policy 后：

* fallback rate 没有明显下降；
* slow 12 scenes 仍然慢；
* 或 memory/precompute 增长超过 ALT 很多，但 query time 仍不优；

那说明 slow case 不是 budget skip 主导，而是 transition abstraction 本身有问题。

**好结果：论文核心卖点**

如果新 cache policy 达到：

```text
cache-hit rate ≥ 98–99%
fallback_nodes ≈ 0
HM3D p95 query time 显著下降
τ=5 slow cases 大部分恢复到 ALT-comparable 或 better
```

就可以主打：

> PH-ALT-TG gives predictable sub-millisecond query latency by eliminating online voxel search in the cache-hit regime.

---

## Priority 3 — 实现 adaptive τ selector，把 oracle τ 变成方法

**要改哪段代码 / 算法**

实现一个 `TauSelector`，输入 scene-level dry stats，不使用 ALT 的真实 query time。

接口建议：

```cpp
TauChoice selectTau(
    const SurfaceGraph& Gs,
    const MergeTree& ph_tree,
    const vector<Query>& query_distribution_optional,
    const SelectorMode mode  // scene_only | query_aware | learned
);
```

候选只允许：

[
\tau \in {3,4,5,6}
]

流程：

1. 对四个 τ collapse PH merge tree；
2. 只构建 lightweight room / portal / TG stats；
3. 估计 fallback risk、large-room risk、graph size、connectedness；
4. scene-only rule 选 τ；
5. query-aware mode 额外使用 start-goal distribution；
6. 只对最终 τ 做正式 full precompute。

**要记录哪些新日志**

每个 scene、每个 τ：

```text
R_tau
P_tau = |V_T|
E_tau = |E_T|
TG_connected_components
query_pair_reach_rate
max_room_size_fraction
room_size_gini
max_portals_per_room
mean_portals_per_room
max_budget_ratio
skipped_room_fraction
transfer_coverage_est
endpoint_skipped_rate
same_room_rate
near_room_rate
mean_room_hops
p90_endpoint_to_portal_ratio
abstract_detour_proxy
selector_score
selected_tau
oracle_tau_after_eval
selector_regret
```

**要重跑哪些实验**

* HM3D 100 scenes, τ={3,4,5,6}, fixed τ vs adaptive τ
* MP3D 6 scenes, τ={3,4,5,6}, fixed τ vs adaptive τ
* HM3D 12 slow cases, specifically check whether selector picks τ=3 or τ=4 instead of τ=5/6
* 10 minival scenes at 0.05m, because resolution scaling是核心卖点

**要画哪些图**

1. selected τ distribution vs oracle τ distribution
2. selector regret CDF：

[
\frac{T_{\text{selected}}-T_{\text{oracle}}}{T_{\text{oracle}}}
]

3. speedup CDF：ALT vs fixed τ vs adaptive τ
4. fallback risk score vs actual fallback nodes
5. max budget ratio / skipped fraction vs selected τ

**失败判据**

如果 adaptive selector：

* 不能在 ≥70% scenes 内选到 oracle τ 的 10% query time 范围；
* 或 adaptive τ 的 HM3D mean/median 仍不如 fixed τ=5；
* 或 win rate vs ALT 仍 <70%；
* 或 path overhead 更差；

那 adaptive τ 不能作为论文主线，只能放 appendix。

**好结果：论文核心卖点**

如果 adaptive τ 达到：

```text
win rate vs ALT ≥ 75–80%
median speedup vs ALT ≥ 1.5x
oracle regret ≤ 10–15%
path overhead 不高于 fixed τ=5
fallback rate 明显低于 fixed τ=5
```

这可以直接成为论文核心卖点：

> A simple map-statistics-based τ selector recovers most of the per-scene oracle benefit without using baseline timings.

---

## Priority 4 — 降低 HM3D path overhead：same-room / near-room direct path + multi-portal

**要改哪段代码 / 算法**

先做两个低风险改动。

### 4.1 same-room / near-room direct shortcut

如果：

[
r_s = r_g
]

不要强迫路径绕 portal。直接用 room-local exact A* / ALT / cached Dijkstra：

```cpp
if (start_room == goal_room) {
    return localRoomALT(start, goal, room);
}
```

如果：

[
\text{room_hop_distance}(r_s,r_g) \le 1
]

也跑一个 cheap local / corridor ALT repair，和 TG route 比较 cost，取短者。

### 4.2 每个 room boundary 至少 K=2 portals

当前 single highest-clearance portal 会造成 detour。改成 doorway component clustering + K medoids：

[
P_{ij}^{K}
==========

\arg\min_{P\subset B_{ij}, |P|=K}
\sum_{q\in B_{ij}} \min_{p\in P} d_{B_{ij}}(p,q)
]

本周只做：

[
K \in {1,2,4}
]

不要一上来做复杂 optimization。

**要记录哪些新日志**

```text
same_room_query
near_room_query
used_direct_shortcut
direct_shortcut_ms
direct_shortcut_path_len
tg_path_len
chosen_direct_or_tg

boundary_id
boundary_component_count
portal_K
portal_spread
endpoint_to_nearest_portal_dist
portal_detour_ratio
path_overhead_by_query_type:
  same_room
  near_room
  cross_room
  multi_floor
```

**要重跑哪些实验**

* HM3D 100 scenes, adaptive τ, K={1,2,4}, with/without same-room direct
* MP3D 6 scenes, τ=4, K={1,2,4}
* 0.05m minival, τ=5, because path overhead +32% 是最大 reviewer 风险
* 单独统计 same-room / near-room / cross-room path overhead

**要画哪些图**

1. path overhead vs speedup Pareto：K=1/2/4 + direct shortcut
2. path overhead broken down by query type
3. endpoint-to-portal detour histogram before/after
4. example path overlay：single-portal detour vs multi-portal repaired path
5. query time increase vs path overhead decrease

**失败判据**

如果 K=2/4 + direct shortcut 后：

* HM3D path overhead 仍 >15%；
* 或 path overhead 降了但 median speedup vs ALT <1；
* 或 portal count 让 query time 接近 surface ALT；

那说明 portal abstraction 本身损失太大，论文必须弱化 path-quality claim。

**好结果：论文核心卖点**

如果达到：

```text
HM3D path overhead ≤ 8–10%
median speedup vs ALT ≥ 1.5x
cache-hit rate ≥ 98%
```

这就是非常强的主图：

> PH-ALT-TG offers a controllable speed–quality Pareto frontier via portal budget.

---

## Priority 5 — 做最终 paper-core rerun：adaptive τ + cache fix + path fix 对 ALT

**要改哪段代码 / 算法**

冻结一个本周版本：

```text
PH-ALT-TG-adaptive
= new cache policy
+ adaptive τ selector
+ same-room / near-room direct shortcut
+ K=2 portals default
+ fixed timing protocol
```

同时保留 ablations：

```text
PH-TG no ALT
PH-TG Euclidean
PH-ALT-TG fixed τ=3
PH-ALT-TG fixed τ=5
PH-ALT-TG adaptive τ
grid-TG + ALT
surface ALT
old full HPA*
```

**要记录哪些新日志**

除了前面所有日志，再加 timing methodology：

```text
warmup_runs
repeat_count
median_query_ms
mean_query_ms
p90_query_ms
p95_query_ms
p99_query_ms
std_query_ms
cpu_pinned
batch_size
precompute_ms_including_selector
memory_bytes
```

**要重跑哪些实验**

最低限度：

1. **HM3D 100 scenes, 0.1m**
   固定 τ、adaptive τ、ALT、old full、grid-TG+ALT。

2. **MP3D 6 scenes, 0.1m and 0.05m**
   证明 resolution scaling。

3. **HM3D 10 minival, 0.05m**
   证明高分辨率优势不是 MP3D 特例。

4. **更多 query per scene**
   当前 HM3D 3 queries/scene 太弱。至少做：

```text
10 queries / scene = 1000 queries
```

如果时间不够，先对 30 scenes 做 20 queries/scene。

**要画哪些图**

1. Main table：ALT vs fixed τ vs adaptive τ
2. speedup CDF on HM3D
3. resolution scaling：0.1m / 0.075m / 0.05m
4. path overhead vs speedup Pareto
5. fallback rate before/after cache fix
6. selector regret CDF
7. PH vs grid ablation：证明 PH 不是装饰

**失败判据**

如果最终版本在 HM3D 100 上：

```text
adaptive PH-ALT-TG median speedup vs ALT < 1.2x
win rate vs ALT < 65%
path overhead > 15%
fallback rate > 5%
```

那这条路线不适合作为“强速度论文”投，只能改成 topology representation / interpretability paper。

**好结果：论文核心卖点**

如果最终版本达到：

```text
HM3D win rate vs ALT ≥ 75–80%
median speedup ≥ 1.5x
p95 latency lower than ALT
path overhead ≤ 10–12%
precompute ≤ or comparable to ALT
0.05m speedup ≥ 3x
```

主 claim 可以非常清楚：

> PH-ALT-TG transfers ALT from the full voxel graph to a topology-derived transition graph, giving most of the shortest-path guidance at much smaller online search cost, with automatic τ selection and bounded fallback.

---

# 二、Adaptive τ selection for PH-ALT-TG

## 0. 统一定义

候选集合固定：

[
\mathcal T={3,4,5,6}.
]

对每个 scene 和每个 τ，先从 PH merge tree collapse 得到 room partition：

[
P_\tau = {R_1^\tau,\dots,R_{N_\tau}^\tau}.
]

记录：

[
R_\tau = |P_\tau|,
]

[
P_\tau^{portal}=|V_T^\tau|,
]

[
E_\tau^{TG}=|E_T^\tau|.
]

对每个 room (r)：

[
n_r = |R_r|,
\qquad
k_r = #\text{incident portals}.
]

当前 fallback 风险的关键 proxy：

[
B_\tau^{max}
============

\max_r
\frac{k_r^2 n_r}{B}.
]

更推荐的新 cache policy 下也记录：

[
B_{\tau,root}^{max}
===================

\max_r
\frac{k_r n_r}{B_{root}}.
]

skipped mass：

[
M_\tau^{skip}
=============

\frac{
\sum_r n_r \cdot \mathbf 1[k_r^2 n_r>B]
}{
\sum_r n_r
}.
]

transfer coverage：

[
C_\tau^{transfer}
=================

\frac{
#\text{cached transfer pairs}
}{
#\text{required transfer pairs}
}.
]

largest-room fraction：

[
L_\tau
======

\max_r
\frac{n_r}{|V_S|}.
]

room-size imbalance：

[
Gini_\tau
=========

Gini({n_r}*{r=1}^{R*\tau}).
]

connectedness：

[
Conn_\tau
=========

\frac{
#\text{reachable start-goal pairs in }G_T^\tau
}{
#\text{start-goal pairs}
}.
]

对每个 query (q=(s,g))，如果 query distribution 已知，记录：

[
Same_\tau(q)=\mathbf 1[\rho_\tau(s)=\rho_\tau(g)].
]

endpoint-to-portal risk：

[
EP_\tau(q)
==========

\frac{
d_{\rho(s)}(s,T_{\rho(s)})
+
d_{\rho(g)}(g,T_{\rho(g)})
}{
|s-g|_2+\epsilon
}.
]

abstract detour proxy：

[
AD_\tau(q)
==========

\frac{
\hat d_T^\tau(s,g)
}{
LB(s,g)+\epsilon
},
]

其中：

[
LB(s,g)=|s-g|_2+\omega_z|\Delta z|.
]

(\hat d_T^\tau(s,g)) 是在 transition graph 上的 cheap dry-run shortest path cost，不使用 ALT baseline timing。

为了避免不同 scene 尺度不同，所有 scene 内候选 τ 的特征做 min-max normalization：

[
\widetilde x_\tau
=================

\frac{
x_\tau-\min_{\tau'\in\mathcal T}x_{\tau'}
}{
\max_{\tau'\in\mathcal T}x_{\tau'}-\min_{\tau'\in\mathcal T}x_{\tau'}+\epsilon
}.
]

---

# 1. Rule-based τ selector

## 1.1 设计目标

scene-only，不使用 query time，不使用 ALT baseline。只看 map / PH / room graph / cache-risk stats。

直觉：

* τ 太小：rooms / portals 太多，TG search 和 precompute 变大；
* τ 太大：large rooms、poor portal coverage、budget skip、fallback risk 上升；
* 最优 τ 应该是 **fallback risk 低 + transition graph 不太大 + largest room 不过分大** 的折中点。

## 1.2 特征

对每个 τ：

```text
R_tau
P_tau = |V_T|
E_tau = |E_T|
max_room_fraction L_tau
room_size_gini Gini_tau
max_budget_ratio Bmax_tau
skipped_room_fraction Mskip_tau
transfer_coverage Ctransfer_tau
TG_connected_components
Conn_tau
mean_portals_per_room
max_portals_per_room
```

## 1.3 公式

hard penalty：

[
H_\tau
======

100\cdot
\mathbf 1[
Conn_\tau < 0.99
\ \lor
C_\tau^{transfer}<0.95
].
]

fallback risk：

[
F_\tau
======

0.40\widetilde{M_\tau^{skip}}
+
0.30\widetilde{B_\tau^{max}}
+
0.20(1-C_\tau^{transfer})
+
0.10\widetilde{L_\tau}.
]

graph work proxy：

[
G_\tau
======

0.45\widetilde{P_\tau^{portal}}
+
0.35\widetilde{E_\tau^{TG}}
+
0.20\widetilde{R_\tau}.
]

coarseness / path-risk proxy：

[
Q_\tau
======

0.50\widetilde{L_\tau}
+
0.30\widetilde{Gini_\tau}
+
0.20\widetilde{B_\tau^{max}}.
]

final score：

[
S_{\text{rule}}(\tau)
=====================

H_\tau
+
0.45F_\tau
+
0.35G_\tau
+
0.20Q_\tau.
]

选择：

[
\tau^*
======

\arg\min_{\tau\in{3,4,5,6}}
S_{\text{rule}}(\tau).
]

tie-breaking：

```text
if scores within 0.03:
    choose larger τ
```

理由：风险接近时选更 coarse 的 graph，降低 query cost。

## 1.4 伪代码

```python
def select_tau_rule(scene):
    candidates = [3, 4, 5, 6]
    stats = {}

    for tau in candidates:
        rooms = collapse_ph_tree(scene.merge_tree, tau)
        tg_stats = build_lightweight_tg_stats(rooms)
        cache_stats = estimate_cache_coverage(rooms, tg_stats)

        stats[tau] = {
            "R": num_rooms(rooms),
            "P": tg_stats.num_portal_nodes,
            "E": tg_stats.num_edges,
            "L": max_room_fraction(rooms),
            "gini": room_size_gini(rooms),
            "Bmax": max_budget_ratio(rooms),
            "Mskip": skipped_room_fraction(rooms),
            "Ctransfer": cache_stats.transfer_coverage,
            "Conn": tg_stats.reachable_pair_rate,
        }

    norm = minmax_normalize_per_feature(stats)

    best_tau = None
    best_score = float("inf")

    for tau in candidates:
        H = 100.0 if (
            stats[tau]["Conn"] < 0.99 or
            stats[tau]["Ctransfer"] < 0.95
        ) else 0.0

        F = (
            0.40 * norm[tau]["Mskip"] +
            0.30 * norm[tau]["Bmax"] +
            0.20 * (1.0 - stats[tau]["Ctransfer"]) +
            0.10 * norm[tau]["L"]
        )

        G = (
            0.45 * norm[tau]["P"] +
            0.35 * norm[tau]["E"] +
            0.20 * norm[tau]["R"]
        )

        Q = (
            0.50 * norm[tau]["L"] +
            0.30 * norm[tau]["gini"] +
            0.20 * norm[tau]["Bmax"]
        )

        score = H + 0.45 * F + 0.35 * G + 0.20 * Q

        if score < best_score - 0.03:
            best_tau = tau
            best_score = score
        elif abs(score - best_score) <= 0.03:
            best_tau = max(best_tau, tau)

    return best_tau, stats
```

## 1.5 预期行为

* τ=5 / τ=6：在 large room 不危险、coverage 高时被选中；
* τ=3 / τ=4：当 τ=5 触发 large-room budget skip、connectivity risk、fallback risk 时被选中；
* MP3D 应该多选 τ=4；
* HM3D 应该多数选 τ=5，slow cases 选 τ=3/4。

---

# 2. Query-aware τ selector

## 2.1 使用场景

当 start-goal workload 已知，比如 PointNav episodes、固定 patrol routes、benchmark query set，可以用 query distribution。仍然不能用 ALT timing，也不能用跑完 TG 后的真实 query time 来挑。

它只用：

* start / goal 落在哪些 room；
* query 是否 same-room / near-room；
* endpoint 到 portal 的距离；
* transition graph dry-run path cost；
* cache-hit estimate。

## 2.2 特征

在 rule-based 特征之外，增加：

```text
same_room_rate
near_room_rate
mean_room_hops
p90_room_hops
query_reach_rate
query_cache_hit_est
endpoint_skipped_rate
p50_endpoint_to_portal_ratio
p90_endpoint_to_portal_ratio
mean_abstract_detour_proxy
p90_abstract_detour_proxy
```

其中：

[
EP_\tau
=======

p90_q\ EP_\tau(q).
]

[
AD_\tau
=======

median_q\ AD_\tau(q).
]

[
Hops_\tau
=========

mean_q\ \text{transition hops}_\tau(q).
]

query fallback risk：

[
F_\tau^Q
========

1-
\frac{1}{Q}
\sum_{q=1}^{Q}
\mathbf 1[\text{all segments cache-hit under }\tau].
]

same-room waste：

[
W_\tau
======

\frac{1}{Q}
\sum_q
\mathbf 1[\rho_\tau(s_q)=\rho_\tau(g_q)]
\cdot
EP_\tau(q).
]

## 2.3 公式

[
S_{\text{query}}(\tau)
======================

H_\tau
+
0.25G_\tau
+
0.30F_\tau^Q
+
0.20\widetilde{AD_\tau}
+
0.15\widetilde{EP_\tau}
+
0.10\widetilde{Hops_\tau}.
]

如果实现了 same-room direct shortcut，则把 (W_\tau) 权重降到 0；否则加：

[
+0.10\widetilde{W_\tau}.
]

选择：

[
\tau^*
======

\arg\min_\tau S_{\text{query}}(\tau).
]

## 2.4 伪代码

```python
def select_tau_query_aware(scene, queries):
    candidates = [3, 4, 5, 6]
    stats = {}

    for tau in candidates:
        rooms = collapse_ph_tree(scene.merge_tree, tau)
        tg = build_lightweight_transition_graph(rooms)

        per_query = []

        for s, g in queries:
            rs = room_of(s, rooms)
            rg = room_of(g, rooms)

            reachable = tg_rooms_reachable(tg, rs, rg)
            cache_hit_est = estimate_cache_hit_for_query(tg, rs, rg)

            endpoint_ratio = estimate_endpoint_to_portal_ratio(
                s, g, rs, rg, rooms
            )

            abstract_cost, hops = dry_run_tg_shortest_path(
                tg, s, g, use_estimated_edges=True
            )

            lower_bound = euclidean_lb(s, g)
            abstract_detour = abstract_cost / max(lower_bound, 1e-6)

            per_query.append({
                "same_room": rs == rg,
                "reachable": reachable,
                "cache_hit_est": cache_hit_est,
                "endpoint_ratio": endpoint_ratio,
                "abstract_detour": abstract_detour,
                "hops": hops,
            })

        stats[tau] = aggregate_scene_and_query_stats(
            rooms, tg, per_query
        )

    norm = minmax_normalize_per_feature(stats)

    best_tau = None
    best_score = float("inf")

    for tau in candidates:
        H = 100.0 if stats[tau]["query_reach_rate"] < 0.99 else 0.0

        score = (
            H
            + 0.25 * norm[tau]["graph_work"]
            + 0.30 * (1.0 - stats[tau]["query_cache_hit_est"])
            + 0.20 * norm[tau]["abstract_detour"]
            + 0.15 * norm[tau]["endpoint_ratio_p90"]
            + 0.10 * norm[tau]["mean_hops"]
        )

        if not SAME_ROOM_DIRECT_ENABLED:
            score += 0.10 * norm[tau]["same_room_waste"]

        if score < best_score - 0.03:
            best_tau = tau
            best_score = score
        elif abs(score - best_score) <= 0.03:
            # query-aware tie: prefer lower detour, not necessarily larger tau
            if stats[tau]["abstract_detour"] < stats[best_tau]["abstract_detour"]:
                best_tau = tau

    return best_tau, stats
```

## 2.5 预期行为

* 如果 queries 多为短程 / same-room：选更细 τ 或触发 direct shortcut；
* 如果 queries 多为 long-range cross-room：选更粗 τ，只要 fallback risk 低；
* 如果 τ=5 large rooms 会导致 endpoint-to-portal detour 或 cache miss：选 τ=3/4；
* 如果 τ=3 rooms 过碎导致 hops 过多：选 τ=5/6。

---

# 3. Lightweight learned selector

## 3.1 定位

这个不要放 main claim 第一位。它适合 appendix 或补充结果。主文优先放 rule-based，因为 reviewer 更容易接受。

learned selector 只学习：

[
\text{map/query stats} \rightarrow \tau
]

不能把 ALT timing 作为 feature 或 label。

## 3.2 特征

每个 scene-candidate pair ((s,\tau)) 的 feature vector：

[
\phi(s,\tau)
============

[
\widetilde R_\tau,
\widetilde P_\tau,
\widetilde E_\tau,
\widetilde L_\tau,
\widetilde{Gini}*\tau,
\widetilde B*\tau^{max},
M_\tau^{skip},
C_\tau^{transfer},
Conn_\tau,
same_\tau,
near_\tau,
\widetilde{EP_\tau},
\widetilde{AD_\tau},
\widetilde{Hops_\tau}
].
]

如果是 scene-only learned selector，去掉 query-specific features：

```text
same_tau, near_tau, EP_tau, AD_tau, Hops_tau
```

## 3.3 训练标签

定义 training-only oracle objective：

[
J_{\text{train}}(s,\tau)
========================

\log T_{\text{TG}}(s,\tau)
+
\eta \max(0,\Delta_{\text{path}}(s,\tau)-\Delta_0)
+
\lambda(1-Success(s,\tau)).
]

建议：

[
\Delta_0 = 0.10
]

或如果你想保守：

[
\Delta_0 = 0.15.
]

oracle label：

[
y_s
===

\arg\min_{\tau\in{3,4,5,6}}
J_{\text{train}}(s,\tau).
]

注意：这里不含 ALT timing。ALT 只在最终比较中出现。

## 3.4 模型 1：ridge regression objective predictor

[
\hat J(s,\tau)
==============

w^\top \phi(s,\tau)+b.
]

选择：

[
\tau^*
======

\arg\min_\tau \hat J(s,\tau).
]

训练：

[
\min_w
\sum_{s,\tau}
(\hat J(s,\tau)-J_{\text{train}}(s,\tau))^2
+
\alpha|w|_2^2.
]

## 3.5 模型 2：pairwise ranking logistic

对同一个 scene 的两个 τ：

[
P(\tau_a \prec \tau_b)
======================

\sigma
\left(
w^\top
(\phi(s,\tau_a)-\phi(s,\tau_b))
\right).
]

训练目标：

```text
τ_a better than τ_b iff J_train(s,τ_a) < J_train(s,τ_b)
```

推理时：

```python
score[tau] = number_of_pairwise_wins(tau)
choose tau with max score
```

这个比直接分类更适合，因为 τ 是 candidate ranking 问题。

## 3.6 伪代码

```python
def train_learned_tau_selector(train_scenes):
    X = []
    y = []

    for scene in train_scenes:
        candidate_features = {}
        candidate_objectives = {}

        for tau in [3, 4, 5, 6]:
            features = compute_selector_features(scene, tau)
            tg_metrics = run_tg_only_training_eval(scene, tau)

            J = (
                math.log(tg_metrics.median_query_ms)
                + ETA * max(0.0, tg_metrics.path_overhead - PATH_CAP)
                + LAMBDA * (1.0 - tg_metrics.success_rate)
            )

            candidate_features[tau] = features
            candidate_objectives[tau] = J

        for tau in [3, 4, 5, 6]:
            X.append(candidate_features[tau])
            y.append(candidate_objectives[tau])

    model = RidgeRegression(alpha=1e-2)
    model.fit(X, y)
    return model


def select_tau_learned(scene, model):
    scores = {}

    for tau in [3, 4, 5, 6]:
        features = compute_selector_features(scene, tau)
        scores[tau] = model.predict(features)

    return min(scores, key=scores.get)
```

## 3.7 预期行为

如果 rule-based 已经接近 oracle，learned selector 不一定值得放主文。
如果 learned selector 明显降低 oracle regret，放 appendix：

> A lightweight selector trained only on map-derived statistics further improves τ selection, but our main results use the transparent rule-based selector.

---

# 三、Adaptive τ 的实验表格设计

## Table 1 — Main selector comparison

| Method            | τ source    | Mean ms | Median ms | p95 ms | Path Δ | Success | Fallback rate | Win vs ALT | Median speedup vs ALT | Oracle regret | Precompute incl. selector |
| ----------------- | ----------- | ------: | --------: | -----: | -----: | ------: | ------------: | ---------: | --------------------: | ------------: | ------------------------: |
| ALT A*            | —           |         |           |        |    ref |         |             — |          — |                  1.0x |             — |                           |
| TG fixed τ=3      | fixed       |         |           |        |        |         |               |            |                       |               |                           |
| TG fixed τ=4      | fixed       |         |           |        |        |         |               |            |                       |               |                           |
| TG fixed τ=5      | fixed       |         |           |        |        |         |               |            |                       |               |                           |
| TG fixed τ=6      | fixed       |         |           |        |        |         |               |            |                       |               |                           |
| TG adaptive-rule  | scene stats |         |           |        |        |         |               |            |                       |               |                           |
| TG adaptive-query | query stats |         |           |        |        |         |               |            |                       |               |                           |
| TG learned        | learned     |         |           |        |        |         |               |            |                       |               |                           |
| TG oracle         | post-hoc    |         |           |        |        |         |               |            |                       |             0 |                           |

Main paper 只放：

```text
ALT
fixed τ=3
fixed τ=5
adaptive-rule
adaptive-query
oracle
```

learned selector 可以 appendix。

---

## Table 2 — Selector accuracy against oracle

| Selector    | Exact oracle τ match | Within 5% oracle time | Within 10% oracle time | Mean regret | Median regret | Slow-case rescue rate |
| ----------- | -------------------: | --------------------: | ---------------------: | ----------: | ------------: | --------------------: |
| fixed τ=3   |                      |                       |                        |             |               |                       |
| fixed τ=5   |                      |                       |                        |             |               |                       |
| rule        |                      |                       |                        |             |               |                       |
| query-aware |                      |                       |                        |             |               |                       |
| learned     |                      |                       |                        |             |               |                       |

重点看：

[
\text{regret}
=============

\frac{
T_{\text{selected}}-T_{\text{oracle}}
}{
T_{\text{oracle}}
}.
]

---

## Table 3 — Chosen τ distribution

| Selector    | τ=3 | τ=4 | τ=5 | τ=6 |
| ----------- | --: | --: | --: | --: |
| Oracle      |     |     |     |     |
| Rule        |     |     |     |     |
| Query-aware |     |     |     |     |
| Learned     |     |     |     |     |

这张表很重要。它能证明 selector 不是“永远选 τ=5”。

---

## Table 4 — Feature ablation for selector

| Selector variant          | Mean regret | Win vs ALT | Fallback rate | Path Δ |
| ------------------------- | ----------: | ---------: | ------------: | -----: |
| full rule                 |             |            |               |        |
| no fallback-risk features |             |            |               |        |
| no graph-size features    |             |            |               |        |
| no large-room features    |             |            |               |        |
| no query features         |             |            |               |        |

如果 removing fallback-risk features 让 slow cases 回来，就能证明 selector 的机制合理。

---

## Table 5 — Split protocol

| Train split    | Test split     | Selector         | Mean regret | Win vs ALT | Notes         |
| -------------- | -------------- | ---------------- | ----------: | ---------: | ------------- |
| none           | HM3D 100       | rule             |             |            | no learning   |
| HM3D train     | HM3D val       | learned          |             |            | clean         |
| HM3D 80 scenes | HM3D 20 scenes | learned CV       |             |            | diagnostic    |
| HM3D train     | MP3D 6         | learned transfer |             |            | cross-dataset |
| none           | MP3D 6         | rule             |             |            | no learning   |

---

# 四、如何避免 reviewer 认为 adaptive τ 是调参作弊

必须在论文里明确写：

1. **τ candidate set 固定**
   [
   \tau\in{3,4,5,6}
   ]
   不是 per scene 手工扫很多 τ。

2. **selector 不使用 ALT baseline timing**
   Features 只能来自 map、PH segmentation、transition graph stats、cache coverage estimate、query distribution。

3. **selector 在 query timing 前运行**
   不能先跑四个 τ 的真实 query time 再挑最优。
   如果某个 variant 真的 full-precompute 四个 τ，必须把四个 τ 的 precompute 全算进方法成本，并标为 ensemble/oracle-like variant。

4. **main method 用 rule-based selector**
   Rule 写成公式，放进 paper。不要只说“we adaptively select τ”。

5. **oracle τ 只作为 upper bound**
   表格中明确：

   ```text
   Oracle τ is post-hoc and not used by the method.
   ```

6. **learned selector 需要 train/test split**
   不能在 HM3D 100 val 上训练又测试。
   最好 learned selector 放 appendix，主结果用 rule-based。

7. **同时报告 fixed τ baselines**
   必须保留 τ=3、τ=5。这样 reviewer 能看到 adaptive 的增益，而不是隐藏 sensitivity。

8. **报告 selected τ per scene distribution**
   防止 reviewer 怀疑手动选择。

9. **报告 selector failure cases**
   至少列 5 个 selector 选错的 scene，说明原因。

10. **比较目标不进入 selector**
    selector objective 不能含：

[
T_{\text{ALT}}
]

也不要含：

[
\mathbf 1[T_{\text{TG}}<T_{\text{ALT}}].
]

ALT 只能在最终 evaluation table 出现。

---

# 五、本周最关键的 go / no-go 结论

你这周真正要回答 4 个问题：

1. **fallback 是否可预测、可消除？**
   否则 sub-ms claim 不稳。

2. **adaptive τ 是否能接近 oracle？**
   否则 HM3D 只能说 fixed τ 和 ALT 打平，论文力度不够。

3. **path overhead 是否能从 +16–18% 降到 ≤10–12%？**
   否则 reviewer 会说你只是用长路换速度。

4. **PH 是否比 grid partition + ALT 更有用？**
   如果 grid-TG+ALT 和 PH-TG+ALT 一样好，persistent homology 贡献会被攻击。

最理想的一周结果是：

```text
adaptive τ selector:
  ≥75% HM3D scenes beat ALT
  median speedup ≥1.5x
  oracle regret ≤10–15%

cache fix:
  fallback rate ≤1–2%
  p95 latency below ALT

path fix:
  overhead ≤10–12%
  speed still >ALT median

ablation:
  PH-ALT-TG > grid-TG+ALT
  PH-ALT-TG > PH-TG Euclidean
```

如果这些成立，这篇论文的核心卖点就非常清楚：**PH 提供 topology-derived compression，ALT 提供 strong heuristic，adaptive τ 和 cache coverage 让它在 HM3D 上不再只是 per-scene oracle，而是可部署的自动方法。**
