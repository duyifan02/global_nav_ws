# 1. Executive summary

* **当前 full method 在 0.1m 实验里确实被 ALT A* 打败**：full 平均查询时间 0.734ms，ALT A* 0.425ms；full 比 ALT 慢约 **1.73×**，路径长度高约 **16.5%**，节点展开约 **2.50×**。这不是小噪声，而是方法结构层面的信号。
* **你的方法不应再主打“比所有 graph-search baselines 更快”**。更稳的定位是：从 raw 3D occupancy 中构造 **seed-reachable traversable surface + PH-derived room topology + cacheable transition overlay**，服务于可解释、多层级、可复用、可与任务规划结合的室内导航。
* **ALT 赢的核心原因**是它仍然在原始 surface graph 上搜索，保留最优性；landmark heuristic 是 graph-distance lower bound，比你的 Euclidean room heuristic 强得多；它没有 room abstraction、portal pruning、stitch detour 和 fallback 的误差。ALT 的原始思想就是用 landmark 和 triangle inequality 给 A* 提供强下界，并保持最短路最优性。([cs.princeton.edu][1])
* **当前 full method 的主要失败点不是 PH segmentation 本身，而是 lossy overlay**：先选 room sequence，再 stitch；portal/transition 表达不充分；heuristic 弱；τ 导致 room count、fallback 和 path quality 强烈波动。你的实验中 tau=8 时 S4/S6 出现从 stitch 到 corridor A* fallback 的双峰行为，说明 cache coverage 和 abstraction connectivity 还不稳定。
* **最优先的改进方向**是不要把 ALT 当敌人，而是把它吸收到方法里：构建 **PH-room transition graph**，在 transition graph 上运行 ALT A*。这样保留 PH room/hierarchy novelty，同时用 graph-distance landmark heuristic 替代 Euclidean room heuristic。
* **论文叙事应该改成“topology-aware planner + strong heuristic integration”**：ALT 是强 baseline，也可以是你的模块。你要证明的是 PH hierarchy 提供了 raw 3D traversability、可解释 room route、局部更新、多查询缓存、任务层接口，而不是单纯在所有场景下秒杀 ALT。
* **下周最值得做的实验**：实现 `PH-ALT-TG`，补充 portal count、τ、landmark count、resolution、多查询 amortization、fallback rate、abstraction gap、memory-speed Pareto 曲线。

---

# 2. Why ALT A* beats the current method

## 2.1 现象先定量：ALT 不是偶然赢

你当前 0.1m ablation/baseline 结果里：

| Metric                |    full |  ALT A* | 结论                |
| --------------------- | ------: | ------: | ----------------- |
| Mean query time       | 0.734ms | 0.425ms | full 慢约 1.73×     |
| Mean path length      | 40.076m | 34.388m | full 路径长约 16.5%   |
| Mean nodes expanded   |   8,746 |   3,494 | full 展开约 2.50× 节点 |
| Speedup vs Surface A* |    5.7× |    9.8× | ALT 更强            |

这些数字说明：在当前 graph size、resolution、τ、portal 和 stitch 设置下，ALT A* 同时在速度和路径质量上占优。尤其 S3 和 S6 的 full path overhead 很高，S1/S4/S5 又有 fallback，导致 full 的“理论 O(1) stitch”没有稳定兑现。

---

## 2.2 Search space comparison：ALT 搜索大图，但 heuristic 强；ours 搜索小图，但 abstraction 有损

设 surface graph 为：

[
G_S=(V_S,E_S),\qquad |V_S|=n,\ |E_S|=m .
]

ALT A* 在 (G_S) 上搜索，使用 landmark set (\mathcal L)。对每个 landmark (L)，预计算：

[
d_S(L,v),\quad \forall v\in V_S .
]

undirected 情况下：

[
h_{\text{ALT}}(v,t)
===================

\max_{L\in\mathcal L}
\left|
d_S(L,t)-d_S(L,v)
\right|.
]

directed 情况下：

[
h_{\text{ALT}}(v,t)
===================

\max_{L\in\mathcal L}
\left{
d_S(L,t)-d_S(L,v),\
d_S(v,L)-d_S(t,L)
\right}.
]

复杂度近似为：

[
T_{\text{pre}}^{\text{ALT}}
===========================

O!\left(
|\mathcal L| \cdot (|E_S|\log |V_S|)
\right),
]

[
T_{\text{query}}^{\text{ALT}}
=============================

O(N_{\text{ALT}}\log |V_S|),
]

其中 (N_{\text{ALT}}) 是 A* 实际展开节点数。

你的 full method 先把 (V_S) partition 成 PH rooms：

[
\rho:V_S\rightarrow {1,\dots,R}.
]

每个 room (r) 有 transition set (T_r)。预计算近似为：

[
T_{\text{pre}}^{\text{ours}}
============================

\sum_{r=1}^{R}
|T_r|\cdot O(|E_r|\log |V_r|),
]

查询近似为：

[
T_{\text{query}}^{\text{ours}}
==============================

O(|E_R^{\text{expanded}}|\log |V_R|)
+
O(|P_{\text{stitch}}|),
]

如果 stitch hit，则没有 voxel-level priority queue；如果 fallback，则还会加上 corridor/local A*：

[
+
O(N_{\text{fallback}}\log |V_S|).
]

所以 ours 赢 ALT 的条件不是“room graph 小于 surface graph”这么简单，而是：

[
|E_R^{\text{expanded}}|\log |V_R| + |P_{\text{stitch}}|
\ll
N_{\text{ALT}}\log |V_S|.
]

但在你当前实验里，ALT 已经把 (N_{\text{ALT}}) 降到了平均 3,494，而 full 平均仍有 8,746 个 expanded nodes；这说明 room abstraction 没有稳定减少实际搜索，反而由于 fallback 或 poor abstraction 增加了展开。

---

## 2.3 Heuristic strength：Euclidean room heuristic 太弱

你的 room-level heuristic 当前大致是 transition point 的 Euclidean distance：

[
h_E(e_i,e_g)=|x_{e_i}-x_{e_g}|_2 .
]

如果 edge cost 至少包含几何长度，且所有 penalty 非负，那么：

[
h_E(v,t)\le d_G(v,t).
]

ALT 的 graph-distance 下界也是 admissible：

[
h_{\text{ALT}}(v,t)\le d_G(v,t).
]

但通常在室内环境中：

[
h_E(v,t)\ll h_{\text{ALT}}(v,t)\lesssim d_G(v,t).
]

原因很直接：Euclidean distance 不知道墙、门、楼梯、绕行、房间连通性；ALT 的 landmark distance 是在 graph 上算出来的，landmark 到目标和当前点的距离差会编码“必须绕墙走”的拓扑信息。A* 展开的状态集合近似是：

[
{v\mid g(v)+h(v)\le C^*}.
]

当 (h) 太弱时，这个集合会膨胀。你的 room graph 虽然比 surface graph 小，但 heuristic 不够强，且 state 还带 incoming transition，实际展开未必少。

这就是为什么 ALT 在 surface graph 上仍能快：它没有降低 graph resolution，却大幅提高了 search guidance。ALT 原始论文也强调这种方法用 preprocessing + triangle inequality lower bounds 加速 shortest path search，同时仍计算最短路径。([cs.princeton.edu][1])

---

## 2.4 Abstraction error：问题不是“房间分割”，而是“有损的 overlay”

定义真实 surface shortest path cost：

[
d_S(s,g)=
\min_{P:s\leadsto g}
\sum_{(u,v)\in P}c(u,v).
]

你的 hierarchical/stitch path 可以写成：

[
d_H(s,g)
========

\min_{\pi}
\left[
d_{r_s}(s,t_0)
+
\sum_{k=0}^{K-1}
\left(
c(t_k,t_{k+1})
+
d_{r_k}(t_k,t_{k+1})
\right)
+
d_{r_g}(t_K,g)
\right],
]

其中 (\pi) 是某个 room/transition sequence。

abstraction gap：

[
\Delta_{\text{abs}}(s,g)
========================

d_H(s,g)-d_S(s,g).
]

如果满足以下理想条件：

1. 所有 room boundary crossing voxels 都作为 portals；
2. 每个 room 内任意 portal-to-portal distance 都是 exact；
3. start/goal 到 room portals 的连接 exact；
4. 查询直接在 transition overlay graph 上找最短路；
5. cost metric 完全一致；

那么 PH room partition 本身不一定引入路径误差。任意 surface path 都可以被分解成 room 内段和 boundary crossing 段。

当前误差主要来自这些地方：

### A. Wrong room sequence

先在 floor/room graph 上选 sequence，再 stitch，可能错过需要暂时绕到另一个 room 或选择不同 doorway 的最优路径。

### B. Transition bottleneck

如果 room boundary 有多个门或宽门被压成一个 representative portal，最短路可能被迫绕向错误 portal。

### C. Over-segmentation

τ 太小会产生太多小 rooms，room graph 中状态数、transition 数和 stitching 段数上升。S4 tau=1 的 path length 达 89.32m，是明显的 fragmentation pathology。

### D. Under-segmentation

τ 太大时，rooms 合并，doorway/passages 结构被抹掉，导致 transfer table 覆盖变差或 fallback 增加。你的 tau sweep 里 S4/S6 在 tau=8/10 出现约 2ms 级 fallback，而 tau=4 时 S6 仍约 0.062ms。

### E. Portal discretization error

doorway 是一段 boundary component，不是单个 voxel。选错 representative 会带来 detour：

[
\Delta_{\text{portal}}
======================

d_S(s,p_{\text{chosen}})
+
d_S(p_{\text{chosen}},g)
------------------------

\min_{p\in B_{ij}}
\left[
d_S(s,p)+d_S(p,g)
\right].
]

### F. Precomputed tree rigidity

你当前 full method 用 transition-rooted Dijkstra tree stitch。对某些 start/goal，tree path 是从 portal root 到 endpoint 的最短路，但整体组合可能不是 global optimal。你的实验已经显示：no_stitch 平均路径 33.910m，而 full 是 40.076m；也就是说，online local voxel A* 更灵活，stitch precompute 牺牲了路径质量。

### G. Objective mismatch

你表中有 flat_surface/no_stitch 相对 Surface A* 出现负 overhead。严格来说，如果 Surface A* 是同一 graph、同一 cost、同一 endpoints 的 exact optimum，就不应出现负 gap。建议以后同时报告：

[
\text{CostGap}
==============

\frac{C(P)-d_S^c(s,g)}{d_S^c(s,g)}
]

和

[
\text{LengthGap}
================

\frac{\ell(P)-\ell(P^*)}{\ell(P^*)},
]

并用同一个 snapping、同一个 edge cost、同一个 exact Dijkstra/A* reference。否则 reviewer 会质疑“optimality gap”的定义。

---

## 2.5 Precomputation mismatch：ALT 存的是全局距离结构；ours 存的是局部 transfer

ALT 的 distance table：

[
{d(L,v): L\in\mathcal L,\ v\in V_S}
]

是全局结构摘要。一个 landmark 在房屋另一侧时，(|d(L,t)-d(L,v)|) 会隐式表达墙、走廊、楼梯、绕行等 topology。

你的 transfer table：

[
{d_{t_i}(s): t_i\in T_r,\ s\in S_r}
]

主要表达 room-local geometry。它回答的是：

> “从这个 doorway 到 room 内这个 voxel 怎么走？”

但它没有直接回答：

> “从当前 doorway 到目标 doorway，全局上应该穿过哪串 rooms？”

这正是 ALT 强的地方：它用少量 landmarks 编码全局 shortest-path geometry；而当前 full method 的 global heuristic 仍是 Euclidean。

---

## 2.6 Resolution scaling：为什么理论优势没在 0.1m 出现

你当前 6 个 Matterport scenes 的 surface nodes 约为 1.2k–14k。这个规模对 16-landmark ALT 来说还不大，预计算和内存都可控。ALT 在这样的 graph 上是很强的 baseline。

你的方法理论优势更可能出现在：

[
|V_S|\rightarrow 10^5,\ 10^6
]

并且：

[
Q\gg 1
]

的 static multi-query setting。也就是 amortized cost：

[
T_{\text{amort}}(Q)
===================

\frac{T_{\text{pre}}}{Q}
+
T_{\text{query}}.
]

但是如果 high-resolution 下仍然有 fallback、portal detour、weak heuristic，那么 ALT 仍可能赢。因此必须先解决 cache hit、transition graph heuristic 和 path gap。

---

# 3. What the current method is still good for

你的方法真正有价值的地方不是“room A* 一定比 ALT A* 快”，而是以下几件 ALT 本身不解决或不直接解决的事。

## 3.1 Seed-reachable traversability extraction

你的 surface extraction 从 seed 出发取 reachable component：

[
S=CC_C(s_0).
]

这对 3D indoor robot navigation 很重要：它过滤 tabletop、cabinet top、ceiling cavity、floating reconstruction artifacts 等“几何上像 surface，但机器人不可达”的结构。这是 raw 3D occupancy 到 robot-traversable graph 的关键步骤。

ALT 只是给定 graph 后的 shortest-path accelerator；它不解决 graph 是否物理可通行。

## 3.2 Semantic-free topology

PH room segmentation 不依赖人工语义标签，而是从 obstacle distance field 的 maxima/saddles 中提取 room-like components：

[
\text{local maxima of }D \approx \text{room interiors},
]

[
\text{saddles of }D \approx \text{doorways/passages}.
]

这可以作为 semantic-free topology layer，用于：

* room-level route explanation；
* task planner 的 symbolic interface；
* frontier exploration；
* “go to room/stair/exit”类高层规划；
* multi-floor navigation；
* dynamic door/blockage reasoning。

## 3.3 可解释路径表示

ALT 输出的是 surface path；你的 hierarchy 可以输出：

[
\text{floor sequence}
\rightarrow
\text{room sequence}
\rightarrow
\text{transition sequence}
\rightarrow
\text{voxel path}.
]

这对机器人系统很有价值，因为实际导航栈经常需要解释、监控和局部修复路径，而不仅仅是得到一串 voxels。

## 3.4 Dynamic local updates

如果一个 doorway 被堵住，ALT 的 landmark distances 理论上可能全局失效；你的 hierarchy 可以只更新受影响 room boundary、transition edge 或局部 transfer table。这个优势需要实验支持，但叙事上很合理。

## 3.5 Memory-bounded / cache-budgeted planning

ALT memory 是：

[
O(|\mathcal L|\cdot |V_S|).
]

PH transition graph ALT memory 可以变成：

[
O(|\mathcal L_T|\cdot |V_T|)
+
\text{local transfer cache},
\qquad |V_T|\ll |V_S|.
]

这不是免费优势，因为 local transfer cache 也可能大；但它提供了更可控的 budget knob：portal count、room merge、landmark count、shortcut count。

## 3.6 论文主线建议

推荐主线：

> We propose a topology-aware navigation representation for raw 3D occupancy maps. The method extracts the seed-reachable traversable surface, decomposes it into persistent-homology rooms, and builds a reusable transition overlay for interpretable and cacheable planning. Rather than competing with landmark heuristics, the hierarchy is complementary to them: graph-distance landmark lower bounds can be applied on the PH-derived transition graph, yielding a planner that combines semantic-free topological abstraction with strong goal-directed search.

弱化：

* “fastest planner”；
* “always outperforms ALT”；
* “PH hierarchy alone gives near-optimal paths”；
* “constant-time query in all scenes”。

强化：

* raw 3D traversability extraction；
* semantic-free topological decomposition；
* interpretable room/transition route；
* no online full-surface priority queue when cache hit；
* reusable local geodesic cache；
* compatibility with ALT / shortcut / hub heuristics；
* dynamic local update potential；
* multi-query amortization。

---

# 4. Highest-priority fixes

下面按优先级排序，偏向 1–2 周内可实现和验证的方案。

| Priority | 改进方向                                                    | 数学/算法改动                                                                                          | 预期提升                                           | 副作用                               | 实验                                              | 论文可写性                                  |                                                     |    |
| -------: | ------------------------------------------------------- | ------------------------------------------------------------------------------------------------ | ---------------------------------------------- | --------------------------------- | ----------------------------------------------- | -------------------------------------- | --------------------------------------------------- | -- |
|        1 | **PH transition graph + ALT heuristic**                 | 构造 (G_T=(V_T,E_T))，在 transitions/portals 上预计算 (d_T(L,v))，查询用 (h_T(v,g)=\max_L                    | d_T(L,g)-d_T(L,v)                              | )                                 | 大幅增强 global heuristic，减少 room/transition A* 展开  | 需要维护 transition graph 和 landmark table | full vs roomALT vs transitionALT；landmark 4/8/16/32 | 很高 |
|        2 | **从 room-sequence search 改成 transition-overlay search** | 不再先固定 room sequence；state 直接是 portal/transition node；edge 为 intra-room exact transfer + crossing | 同时选择 room sequence 和 doorway，减少 wrong sequence | (E_T) 可能变稠密                       | 比较 path gap、expanded transition nodes           | 很高                                     |                                                     |    |
|        3 | **exact dynamic start/goal connection**                 | 对 endpoint room (r_s,r_g)，用 cached portal-root distances 连接：(w(s,p)=d_p(s))                      | 减少 start/goal stitch detour；避免 fallback        | same-room query 可能仍需 local repair | endpoint detour ratio，same-room query benchmark | 高                                      |                                                     |    |
|        4 | **portal clustering + multi-portal medoids**            | Boundary (B_{ij}) 聚类；选 (K) 个 medoids：(p^*=\arg\min_p\sum_{q\in B}d(p,q))                         | 降低 portal discretization error                 | portal 多会增大 (G_T)                 | (K=1,2,4,8,\text{all}) Pareto                   | 高                                      |                                                     |    |
|        5 | **adaptive PH room merge**                              | score: (\alpha pers+\beta\log area+\gamma \bar D-\delta degree)；低 score 或 wide boundary merge    | 降低 over-segmentation 和 fallback                | 可能损失 interpretability             | τ fixed vs adaptive；room count vs gap           | 高                                      |                                                     |    |
|        6 | **cost-consistent evaluation**                          | 所有 baselines 用同一 (c(u,v))，同时报告 cost gap 和 length gap                                             | 消除负 overhead 和 reviewer 质疑                     | 需要重跑 reference exact A*/Dijkstra  | 对每个 query 保存 same endpoints/cost                | 很高                                     |                                                     |    |
|        7 | **Hybrid ALT fallback/repair**                          | 当 (h) 弱、portal gap 高或 fallback scene 时，用 surface ALT 修复局部段                                       | 保证 worst-case path quality                     | 会引入在线 surface search              | fallback frequency、repair rate、latency tail     | 中高                                     |                                                     |    |
|        8 | **shortcut / contraction edges on (G_T)**               | 为常用 transition sequence 加 shortcut：(w(u,v)=d_T(u,v))，存 path witness                              | 降低 transition graph search depth               | shortcut 过多会内存爆炸                  | shortcut budget 0/1k/5k/10k                     | 中                                      |                                                     |    |
|        9 | **query/task-aware hubs**                               | stairs、exits、high-betweenness rooms、frequent goals 作为 hubs/landmarks                             | 多查询和任务导航更强                                     | workload-dependent                | random queries vs goal-biased queries           | 中                                      |                                                     |    |
|       10 | **fallback instrumentation and cache-hit objective**    | 记录 (\chi=#\text{no-voxel-search}/Q)、fallback reason、missing transfer pair                        | 快速定位 S1/S4/S5/S6 问题                            | 不是算法本身                            | 每 scene 的 fallback taxonomy                     | 很高                                     |                                                     |    |

---

# 5. Recommended new method: PH-ALT-TG

我建议把改进版方法命名为：

## **PH-ALT-TG: Persistent-Homology Transition-Graph ALT Planner**

核心思想：

> 用 PH rooms 保留你的 topology novelty，但不要在 coarse room graph 上用 Euclidean heuristic 搜索。改为构造一个 sparse transition graph (G_T)，其节点是 doorway/transition representatives，边是 exact cached intra-room geodesics 和 inter-room crossings；然后在 (G_T) 上运行 ALT A*。

这等价于把你的 hierarchy 从“可能有损的 room sequence prior”变成“PH-induced sparse overlay graph”。HPA* 的经典思想也是将地图抽象成 clusters，缓存 cluster crossing distances，再在 abstract graph 上规划；你的版本应当把 cluster 从 uniform grid 换成 PH topology，并用 ALT 加强 abstract search。([core.ac.uk][2])

---

## 5.1 Graph construction

PH segmentation 给出：

[
\rho:V_S\rightarrow {1,\dots,R}.
]

两个 rooms (i,j) 的 boundary crossing set：

[
B_{ij}
======

{(u,v)\in E_S:
\rho(u)=i,\ \rho(v)=j}.
]

对 (B_{ij}) 做 connected-component clustering，得到 doorway components：

[
B_{ij}^1,\dots,B_{ij}^{M_{ij}}.
]

每个 doorway component 选一个或多个 portal representatives。单 portal medoid：

[
p_{ij}^{m}
==========

\arg\min_{p\in B_{ij}^{m}}
\sum_{q\in B_{ij}^{m}}
d_{S_i}(p,q).
]

多 portal 可以用 (K)-medoids：

[
P_{ij}^{m,K}
============

\arg\min_{P\subset B_{ij}^{m}, |P|=K}
\sum_{q\in B_{ij}^{m}}
\min_{p\in P}
d_{S_i}(p,q).
]

定义 transition graph：

[
G_T=(V_T,E_T).
]

### Nodes

[
V_T
===

\bigcup_{r=1}^{R}T_r,
]

其中 (T_r) 是 room (r) 内所有 portal-side nodes。注意同一个 doorway 最好在两侧各有一个 node：

[
p_{i\rightarrow j},\quad p_{j\rightarrow i}.
]

### Edges

#### 1. Intra-room transfer edges

对同一 room 内的 portals (p,q\in T_r)：

[
(p,q)\in E_T,
]

[
w_T(p,q)=d_{S_r}(p,q).
]

这里 (d_{S_r}) 用你已有的 room-local Dijkstra transfer table。

#### 2. Inter-room crossing edges

如果 (p_{i\rightarrow j}) 和 (p_{j\rightarrow i}) 是同一 doorway 两侧：

[
(p_{i\rightarrow j},p_{j\rightarrow i})\in E_T,
]

[
w_T(p_{i\rightarrow j},p_{j\rightarrow i})
==========================================

c(p_i,p_j).
]

#### 3. Optional shortcut edges

对常用或高 betweenness transition pairs (u,v)：

[
(u,v)\in E_T^{\text{shortcut}},
]

[
w_T(u,v)=d_T(u,v),
]

并存 witness path：

[
W(u,v)=\langle u,\dots,v\rangle.
]

这类似 contraction hierarchies 的思想：用 shortcut 表示被压缩的中间路径，但必须控制 shortcut budget。Contraction hierarchies 的基本思路是按节点重要性收缩节点并加入 shortcut edges，以加速 routing。([Springer][3])

---

## 5.2 Landmark heuristic on transition graph

选 transition landmarks：

[
\mathcal L_T\subset V_T.
]

选择策略优先级：

1. farthest-point sampling over (G_T)；
2. stairs / floor connectors；
3. high-degree doorway nodes；
4. high-betweenness transition nodes；
5. PH maxima 所在 room 的主要 portals。

预计算：

[
d_T(L,v),\quad \forall L\in \mathcal L_T,\ v\in V_T.
]

undirected transition graph：

[
h_T(v,g)
========

\max_{L\in\mathcal L_T}
\left|
d_T(L,g)-d_T(L,v)
\right|.
]

directed graph：

[
h_T(v,g)
========

\max_{L\in\mathcal L_T}
\max
\left{
d_T(L,g)-d_T(L,v),
d_T(v,L)-d_T(g,L)
\right}.
]

如果 (g) 是 temporary goal，不在 (V_T)，用 goal-room portal connections 计算：

[
d_T(L,g)
========

\min_{p\in T_{r_g}}
\left[
d_T(L,p)+d_{S_{r_g}}(p,g)
\right].
]

同理：

[
d_T(g,L)
========

\min_{p\in T_{r_g}}
\left[
d_{S_{r_g}}(g,p)+d_T(p,L)
\right].
]

只要 endpoint connection costs 是 exact 或 conservative，ALT heuristic 对 augmented transition graph 仍然 admissible。

---

## 5.3 Why admissible and stronger

对 undirected graph，triangle inequality 给出：

[
d_T(L,g)
\le
d_T(L,v)+d_T(v,g),
]

因此：

[
d_T(L,g)-d_T(L,v)\le d_T(v,g).
]

同理：

[
d_T(L,v)-d_T(L,g)\le d_T(v,g).
]

所以：

[
|d_T(L,g)-d_T(L,v)|\le d_T(v,g),
]

[
h_T(v,g)\le d_T(v,g).
]

因此 (h_T) admissible。若距离表 exact 且 edge weight 非负，它也 consistent。

它通常强于 Euclidean room heuristic，因为：

[
h_E(v,g)=|x_v-x_g|_2
]

只看几何直线，而：

[
h_T(v,g)
]

编码了 transition graph 上的 doorway、corridor、stairs、room connectivity。

注意它不一定严格 domination：

[
h_T(v,g)\ge h_E(v,g)
]

并不总是数学保证成立。实践中可以用：

[
h(v,g)=\max{h_T(v,g),h_E(v,g)}.
]

如果两者都 admissible，max 仍 admissible，而且永不弱于任一单独 heuristic。

---

## 5.4 与 surface-level ALT 的信息损失

surface ALT 的 heuristic 是在 (G_S) 上算的：

[
d_S(L,v).
]

PH-ALT-TG 是在 (G_T) 上算的：

[
d_T(L,v).
]

信息损失来自：

1. 非 portal surface voxels 被压缩；
2. boundary crossing 被 doorway representatives 近似；
3. room 内路径只通过 transfer table 表达；
4. start/goal 到 portals 的连接可能有 detour；
5. 如果 portal pruning 太激进，真实 shortest path 的 crossing point 不在 (V_T)。

但如果把所有 boundary crossing voxels 都作为 portals，且 intra-room distances exact，则 (G_T) 可以保留 surface shortest paths。实际中用 portal clustering 是为了 Pareto tradeoff：

[
\text{memory/query speed}
\quad \leftrightarrow \quad
\text{path optimality}.
]

这正好可以成为论文贡献：**controllable topology-aware compression**。

---

## 5.5 Query procedure

给定 start (s)、goal (g)。

1. snap 到 surface nodes；
2. 找 rooms (r_s=\rho(s))、(r_g=\rho(g))；
3. 添加 temporary nodes (s,g)；
4. 对 (p\in T_{r_s}) 添加：

[
w(s,p)=d_{S_{r_s}}(s,p).
]

5. 对 (q\in T_{r_g}) 添加：

[
w(q,g)=d_{S_{r_g}}(q,g).
]

6. 如果 (r_s=r_g)，可添加 direct local edge：

[
w(s,g)=d_{S_{r_s}}(s,g),
]

这个可以用 bounded local A* 或 room-local ALT 计算。若坚持 no online voxel queue，则可以不加 direct edge，但 same-room query path quality 会受影响。

7. 在 augmented (G_T^+) 上运行 A*：

[
f(v)=g(v)+h_T(v,g).
]

8. 得到 transition sequence 后，用 parent trees stitch voxel path。

---

## 5.6 Complexity

设：

[
n_T=|V_T|,\quad m_T=|E_T|,\quad L_T=|\mathcal L_T|.
]

Transition graph ALT precompute：

[
O(L_T\cdot m_T\log n_T).
]

Landmark memory：

[
O(L_T\cdot n_T).
]

Room transfer precompute：

[
\sum_{r=1}^{R}
|T_r|\cdot O(|E_r|\log |V_r|).
]

Query：

[
O(N_T\log n_T + |P|),
]

其中 (N_T) 是 transition graph A* expanded nodes。

如果 (n_T\ll |V_S|)，则 transition ALT 的 landmark table 比 surface ALT 小得多；同时 heuristic 又比 Euclidean room heuristic 强得多。

---

## 5.7 Expected advantages

* 比 current full 更少 room/transition expansions；
* 比 current full 更低 path overhead，因为不再固定 coarse room sequence；
* 比 surface ALT 更可解释，因为输出 transition/room route；
* 比 surface ALT 更适合动态局部更新；
* 保留 PH segmentation novelty；
* 可自然扩展到 multi-floor、task planning、frequent-goal routing；
* 可用 portal/landmark/shortcut budget 控制 memory-speed-quality Pareto。

---

## 5.8 Potential failures

* Portal count 过少：path gap 仍高；
* Portal count 过多：room clique edges (\sum_r |T_r|^2) 爆炸；
* Endpoint rooms 很大：start/goal connection detour 明显；
* Same-room queries 若不做 direct local edge，会很差；
* Dynamic obstacles 改变 room 内 shortest path，cached transfer 失效；
* 如果 (G_T) 太小，ALT heuristic 再强也只能找到 overlay-optimal path，不是 surface-optimal path。

---

## 5.9 Paper contribution statement

可以写成：

> We introduce a persistent-homology transition overlay for 3D indoor navigation. Unlike standard HPA*-style room search, the proposed overlay represents doorway-side transition states and exact room-local geodesic transfers, allowing global planning directly over transitions rather than over a lossy room sequence. We further equip this topology-derived overlay with landmark-based triangle-inequality lower bounds, combining the interpretability and local update structure of PH rooms with the goal-directed strength of ALT search.

---

# 6. Mathematical formulation

## 6.1 Surface graph and cost

[
G_S=(V_S,E_S),
]

[
c(u,v)
======

r|u-v|*2
+
\omega_z r|\Delta z|
+
\frac{\omega*{\text{obs}}r}{D(v)+1}.
]

True shortest-path cost:

[
d_S(s,g)
========

\min_{P:s\leadsto g}
\sum_{(u,v)\in P}c(u,v).
]

## 6.2 PH room partition

[
\rho:V_S\rightarrow {1,\dots,R}.
]

Room subgraph:

[
G_r=(V_r,E_r),
\quad
V_r={v\in V_S:\rho(v)=r}.
]

## 6.3 Boundary and portal set

[
B_{ij}
======

{(u,v)\in E_S:\rho(u)=i,\rho(v)=j}.
]

Doorway components:

[
B_{ij}=\bigcup_{m=1}^{M_{ij}}B_{ij}^m.
]

Portal medoid:

[
p_{ij}^{m}
==========

\arg\min_{p\in B_{ij}^{m}}
\sum_{q\in B_{ij}^{m}}
d_{S_i}(p,q).
]

Room portal set:

[
T_r={p:\ p\text{ lies on boundary of room }r}.
]

## 6.4 Room-local transfer

[
C_r(p,q)=d_{S_r}(p,q),
\qquad p,q\in T_r.
]

Store predecessor tree:

[
\pi_p^r(v),
\quad
v\in V_r.
]

Path reconstruction:

[
P_r(p,q)=\text{TraceBack}_{\pi_p^r}(q).
]

## 6.5 Transition graph

[
G_T=(V_T,E_T),
]

[
V_T=\bigcup_{r=1}^{R}T_r.
]

[
E_T
===

E_{\text{intra}}
\cup
E_{\text{cross}}
\cup
E_{\text{shortcut}}.
]

Intra-room edge:

[
w_T(p,q)=C_r(p,q).
]

Crossing edge:

[
w_T(p_{i\rightarrow j},p_{j\rightarrow i})
==========================================

c(p_i,p_j).
]

Shortcut:

[
w_T(u,v)=d_T(u,v).
]

## 6.6 ALT on transition graph

[
h_T(v,g)
========

\max_{L\in\mathcal L_T}
|d_T(L,g)-d_T(L,v)|.
]

Directed version:

[
h_T(v,g)
========

\max_{L\in\mathcal L_T}
\max
{
d_T(L,g)-d_T(L,v),
d_T(v,L)-d_T(g,L)
}.
]

Combined admissible heuristic:

[
h(v,g)=\max{h_T(v,g),h_E(v,g)}.
]

## 6.7 Abstraction gap

[
\Delta_{\text{cost}}(s,g)
=========================

\frac{d_T^+(s,g)-d_S(s,g)}{d_S(s,g)}.
]

Path length gap:

[
\Delta_{\text{len}}(s,g)
========================

\frac{\ell(P_T)-\ell(P_S^*)}{\ell(P_S^*)}.
]

## 6.8 Amortized cost

[
T_{\text{amort}}(Q)
===================

\frac{T_{\text{pre}}}{Q}
+
T_{\text{query}}.
]

## 6.9 Cache hit rate

[
\chi
====

\frac{
#{\text{queries with no online voxel-level search}}
}{
Q
}.
]

## 6.10 Adaptive room significance

[
score(r)
========

\alpha,pers(r)
+
\beta\log |V_r|
+
\gamma,\overline D_r
--------------------

\delta,deg(r).
]

Merge if:

[
score(r)<\theta
]

or for adjacent rooms:

[
merge(i,j)
\quad\text{if}\quad
width(B_{ij})>w_{\min}
\ \land
\Delta_{\text{merge}}(i,j)<\epsilon.
]

---

# 7. Algorithm pseudocode

## 7.1 Preprocessing

```text
Algorithm PREPROCESS_PH_ALT_TG(O, seed, r, tau, K_portal, L_T)

1. Extract surface candidates C from occupancy grid O.
2. Keep seed-reachable component:
       V_S = CC_C(seed)
3. Build surface graph G_S=(V_S,E_S).
4. Compute obstacle distance field D(v).
5. Run persistent-homology merge tree on superlevel sets of D.
6. Keep maxima with persistence > tau.
7. Assign room labels:
       rho: V_S -> {1,...,R}

8. For every adjacent room pair (i,j):
       B_ij = {(u,v) in E_S | rho(u)=i, rho(v)=j}
       Cluster B_ij into doorway components.
       For each doorway component:
            select K_portal medoids or representatives.
            create portal-side nodes p_{i->j}, p_{j->i}.

9. For each room r:
       T_r = all portal-side nodes in room r.
       For each portal p in T_r:
            Run Dijkstra on room subgraph G_r from p.
            Store distances d_p(v), v in V_r.
            Store parent pointers pi_p^r(v).
       For each portal pair p,q in T_r:
            Add intra-room transition edge:
                w_T(p,q)=d_p(q)

10. For each doorway portal pair across rooms:
        Add crossing edge with surface edge cost.

11. Optionally add shortcut edges under a memory budget.

12. Select transition landmarks L_T:
        farthest-point sampling + stairs/exits/high-degree portals.

13. For each landmark L in L_T:
        Run Dijkstra on G_T.
        Store d_T(L,v) for all v in V_T.
        If directed, also store reverse distances.

14. Return:
        surface graph, room labels, portal sets,
        transfer tables, transition graph, landmark tables.
```

## 7.2 Online query

```text
Algorithm QUERY_PH_ALT_TG(s_raw, g_raw)

1. Snap:
       s = nearest surface voxel to s_raw
       g = nearest surface voxel to g_raw

2. Get endpoint rooms:
       r_s = rho(s)
       r_g = rho(g)

3. Create temporary nodes s and g.

4. Connect start:
       For each portal p in T_{r_s}:
            w(s,p) = d_p(s)
            store reconstruction pointer p -> s

5. Connect goal:
       For each portal q in T_{r_g}:
            w(q,g) = d_q(g)
            store reconstruction pointer q -> g

6. If r_s == r_g:
       Option A, quality mode:
            compute local d_{r_s}(s,g) with bounded local A*/ALT;
            add edge s -> g.
       Option B, cache-only mode:
            skip direct edge and use portal-mediated route.

7. Define heuristic for any transition node v:
       h(v,g)=max_L |d_T(L,g)-d_T(L,v)|

   where:
       d_T(L,g)=min_{q in T_{r_g}} d_T(L,q)+d_q(g)

8. Run A* on augmented transition graph G_T^+.

9. Obtain transition path:
       s, p_1, p_2, ..., p_k, g

10. Stitch voxel path:
       For every intra-room edge (p_i,p_j):
            append TraceBack_{pi_{p_i}}(p_j)
       For every crossing edge:
            append crossing voxel edge
       For endpoint edges:
            append cached endpoint tracebacks.

11. Return:
       voxel path,
       room sequence,
       transition sequence,
       timing,
       expanded nodes,
       cache-hit/fallback diagnostics.
```

---

# 8. Experimental plan

## 8.1 Datasets

| Dataset type                    | Purpose                                                | Expected insight                                                        |
| ------------------------------- | ------------------------------------------------------ | ----------------------------------------------------------------------- |
| Current 6 Matterport3D scenes   | Direct comparability with current ablation             | Show PH-ALT-TG fixes S1/S3/S4/S6 failure modes                          |
| More Matterport3D scenes        | Generality                                             | Avoid overfitting to 6 scenes                                           |
| Multi-floor scenes              | Test floor graph/stairs topology                       | PH hierarchy should be more useful than flat heuristics for explanation |
| Cluttered furniture-rich scenes | Test seed-reachable extraction and clearance cost      | Surface extraction novelty becomes visible                              |
| Narrow-passage scenes           | Test PH saddles/doorways                               | Portal and τ sensitivity                                                |
| Synthetic mazes                 | Stress graph topology and Euclidean heuristic weakness | ALT and transition ALT should dominate Euclidean                        |
| Synthetic open spaces           | Honest failure case                                    | ALT/flat A* may be sufficient; hierarchy less useful                    |
| Dynamic blockage scenes         | Local update advantage                                 | PH transition graph can update local edges                              |

## 8.2 Methods

| Category                  | Methods                                                                                            |
| ------------------------- | -------------------------------------------------------------------------------------------------- |
| Exact / near-exact search | flat_surface A*, exact Dijkstra subset, Surface ALT A* with 4/8/16/32 landmarks                    |
| Current methods           | grid_partition HPA*, PH no_stitch, current full                                                    |
| Baselines                 | PRM 1k/2k/5k samples                                                                               |
| Improved variants         | Room-level ALT, Transition-level ALT, PH-ALT-TG, PH-ALT-TG + portal medoids, PH-ALT-TG + shortcuts |
| Hybrid methods            | PH-ALT-TG + surface ALT repair, PH-ALT-TG + local room ALT                                         |

## 8.3 Metrics

| Metric                                    | Why it matters                        |   |                  |
| ----------------------------------------- | ------------------------------------- | - | ---------------- |
| success rate                              | Detect disconnected/fallback failures |   |                  |
| query time                                | Main latency metric                   |   |                  |
| p50/p90/p99 query time                    | Predictable latency claim             |   |                  |
| expanded surface nodes                    | Compare with flat/ALT                 |   |                  |
| expanded transition nodes                 | Diagnose abstract search              |   |                  |
| path length                               | Navigation distance                   |   |                  |
| path cost under (c(u,v))                  | Must match planner objective          |   |                  |
| optimality gap vs exact surface reference | Honest quality metric                 |   |                  |
| preprocessing time                        | Amortization                          |   |                  |
| memory                                    | ALT vs transition ALT tradeoff        |   |                  |
| cache hit rate (\chi)                     | Validate no online voxel queue claim  |   |                  |
| fallback frequency                        | Identify hierarchy failures           |   |                  |
| room count (R)                            | τ sensitivity                         |   |                  |
| portal count (                            | V_T                                   | ) | Overlay size     |
| transition edge count (                   | E_T                                   | ) | Query complexity |
| abstraction gap (\Delta_{\text{cost}})    | Measures overlay quality              |   |                  |
| stitch time                               | Separate reconstruction cost          |   |                  |
| endpoint connection time                  | Detect start/goal bottleneck          |   |                  |
| room/transition sequence length           | Interpretability and detour           |   |                  |

## 8.4 Ablations

| Ablation                 | Values                                                              |
| ------------------------ | ------------------------------------------------------------------- |
| τ threshold              | 1,2,3,4,5,6,8,10, adaptive                                          |
| landmarks                | 0,4,8,16,32                                                         |
| landmark location        | random, farthest, high-degree, stairs/exits, hybrid                 |
| portal count per doorway | 1,2,4,8, all boundary portals                                       |
| portal selection         | center voxel, Euclidean medoid, geodesic medoid, k-medoids          |
| heuristic                | Euclidean, room ALT, transition ALT, max(Euclidean, transition ALT) |
| transfer exactness       | exact Dijkstra, approximate, compressed                             |
| shortcut budget          | 0, 1k, 5k, 10k                                                      |
| resolution               | 0.1m, 0.075m, 0.05m, 0.03m if feasible                              |
| query distance           | short, medium, long, cross-floor                                    |
| endpoint mode            | cache-only, bounded local A*, room ALT                              |
| dynamic update           | blocked doorway, changed obstacle, changed cost                     |

---

## 8.5 Core experiments

### Experiment 1 — Reproduce current result with richer diagnostics

Purpose: explain current failure, not just report averages.

Add per-query logs:

[
R,\ |V_T|,\ |E_T|,\ \chi,\ N_R,\ N_T,\ N_{\text{fallback}},\ \Delta_{\text{cost}}.
]

Expected result: S1/S4/S5 fallback and S3/S6 path detours become attributable to missing portals, wrong room sequence, or τ effects.

Interpretation: gives reviewer confidence that failure is understood.

---

### Experiment 2 — Surface ALT landmark scaling

Run Surface ALT with:

[
|\mathcal L|=4,8,16,32.
]

Purpose: measure how hard ALT is to beat and where memory/query Pareto lies.

Expected result: 16 likely near-saturates current 0.1m scenes; 32 may improve modestly but memory rises.

Interpretation: establishes ALT as serious main baseline, not a strawman.

---

### Experiment 3 — Room ALT vs Transition ALT

Compare:

1. current Euclidean room heuristic;
2. room-level ALT;
3. transition-level ALT.

Purpose: isolate heuristic strength.

Expected result:

[
N_{\text{Euclid}} > N_{\text{RoomALT}} > N_{\text{TransitionALT}}.
]

Interpretation: shows ALT is complementary to hierarchy.

---

### Experiment 4 — Transition overlay path quality

Compare current full vs PH-ALT-TG with identical rooms/portals.

Purpose: test whether path overhead comes from room sequence abstraction.

Expected result: PH-ALT-TG should reduce S3/S6 overhead by choosing better portals/room sequences.

Interpretation: if path improves, paper can claim lossy room-sequence planning was fixed.

---

### Experiment 5 — Portal count Pareto

Run:

[
K_{\text{portal}}=1,2,4,8,\text{all}.
]

Plot:

[
|V_T|,\ |E_T|,\ T_{\text{query}},\ \Delta_{\text{cost}},\ memory.
]

Expected result: K=2 or K=4 likely gives large path improvement with moderate overhead.

Interpretation: creates a strong figure: controllable compression-quality tradeoff.

---

### Experiment 6 — Adaptive τ vs fixed τ

Compare fixed τ values with adaptive room merge.

Purpose: reduce tau sensitivity.

Expected result: adaptive τ avoids S4 tau=1 fragmentation and S4/S6 tau=8 fallback.

Interpretation: strengthens robustness claim.

---

### Experiment 7 — Resolution scaling

Run 0.1m, 0.075m, 0.05m, maybe 0.03m subset.

Report:

[
|V_S|,\ |V_T|,\ T_{\text{pre}},\ T_{\text{query}},\ memory,\ \Delta.
]

Expected result: surface A* and surface ALT query/memory grow with (|V_S|); PH-ALT-TG grows more with (|V_T|), which should be slower.

Interpretation: this is where hierarchy can honestly show value.

---

### Experiment 8 — Multi-query amortization

For each scene sample:

[
Q=1,10,50,100,500,1000.
]

Report:

[
T_{\text{amort}}(Q)=T_{\text{pre}}/Q+T_{\text{query}}.
]

Expected result: single-query may favor ALT; many-query static scene may favor cached transition overlay depending on memory and query time.

Interpretation: reframes paper from single-query race to reusable navigation representation.

---

### Experiment 9 — Dynamic local blockage

Randomly close 5%, 10%, 20% of doorways or add local obstacles.

Compare update cost:

* Surface ALT recompute landmarks;
* PH-ALT-TG update affected room/door edges;
* current full recompute transfer for affected rooms.

Expected result: PH hierarchy should update locally.

Interpretation: gives ALT cannot simply “solve everything” response.

---

### Experiment 10 — Task-level route benchmark

Queries like:

* start → nearest stair → target floor room;
* visit rooms A, B, C;
* avoid room type or blocked doorway;
* route explanation quality.

Expected result: PH hierarchy provides symbolic route and constraints naturally; ALT alone needs extra structure.

Interpretation: supports novelty beyond shortest path latency.

---

## 8.6 Figures and tables to add

1. **Pipeline figure**: occupancy → reachable surface → distance field → PH merge tree → rooms → transition graph → path.
2. **Distance field + PH merge tree**: show maxima, saddles, persistence threshold.
3. **Room segmentation overlay**: fixed τ vs adaptive τ.
4. **ALT vs current full expansion heatmap**: where ALT avoids search, where full falls back.
5. **Transition graph visualization**: portals, intra-room edges, crossing edges, landmarks.
6. **τ vs path gap / query time / fallback rate**.
7. **Portal count Pareto**: path gap vs query time vs memory.
8. **Resolution scaling**: (|V_S|), (|V_T|), query time.
9. **Multi-query amortization curve**.
10. **Failure case visualization**: S3/S6 current detour; PH-ALT-TG repaired route.
11. **Memory-speed Pareto**: Surface ALT, PH-ALT-TG, PRM, grid HPA*.
12. **Dynamic update plot**: update time vs percentage blocked doorways.

---

# 9. Paper revision strategy

## 9.1 Abstract

不要写：

> Our method outperforms all graph-search baselines in query time.

改成：

> We present a topology-aware navigation representation for raw 3D occupancy maps that extracts the seed-reachable traversable surface, decomposes it into persistent-homology rooms, and constructs a reusable transition overlay for interpretable and cacheable path planning. We further show that landmark-based graph heuristics can be applied on this topology-derived overlay, combining the goal-directed efficiency of ALT search with room-level structure useful for multi-query and task-level navigation.

## 9.2 Introduction

强调问题不是只有 shortest path search：

1. raw 3D occupancy 有不可达伪 surface；
2. robot navigation 需要可解释、可局部更新、可任务规划的 topology；
3. flat graph speedups 如 ALT 很强，但不产生 room-level abstraction；
4. 本文贡献是 topology extraction + transition overlay + heuristic integration。

## 9.3 Related work

ALT 必须放 main related work 和 main baseline。不能藏到 supplementary。ALT 原始工作明确是 preprocessing + landmark lower bounds + A* shortest path acceleration。([cs.princeton.edu][1])

HPA* 也要诚实讨论：它将地图抽象为 clusters，缓存 crossing distances，并在 abstract graph 上规划；你的差异是 PH-derived rooms、3D reachable surface、clearance field 和 transition-ALT integration。([core.ac.uk][2])

Contraction/hub methods可以作为未来或附加模块讨论，不要过度承诺。Hub labeling 是另一类强 shortest-path preprocessing 方法，在 road network 上非常强，但 memory/preprocessing tradeoff 不同。([Microsoft][4])

## 9.4 Method

把 current full method 改为：

1. traversable surface extraction；
2. PH room decomposition；
3. portal/transition graph construction；
4. local geodesic transfer cache；
5. ALT heuristic on transition graph；
6. stitch reconstruction；
7. optional local repair / shortcut.

这样 ALT 不再是外部威胁，而是方法中的 goal-directed component。

## 9.5 Experiments

Main table 应该包含：

* Surface A*；
* Surface ALT 4/8/16/32；
* current full；
* PH-ALT-TG；
* grid HPA*；
* PRM；
* no_stitch。

主结果不要只放 mean。放：

* query p50/p90/p99；
* cost gap；
* fallback rate；
* memory；
* preprocessing；
* amortized Q=1/10/100/1000。

## 9.6 Discussion

诚实承认：

* At coarse resolution and single-query settings, surface ALT is extremely competitive.
* Current hierarchy alone can be worse due to abstraction and weak heuristic.
* The improved method integrates ALT on the PH transition graph.
* The value of PH is topology, interpretability, local update, task interface, and controllable compression.

## 9.7 Reviewer risk response

Reviewer 可能说：

> ALT already solves the problem.

回应方式：

1. ALT solves shortest path acceleration on a given graph, not raw 3D traversability extraction.
2. ALT gives no room/floor/doorway abstraction.
3. ALT does not naturally support task-level constraints over rooms.
4. ALT landmark tables change globally under edge updates; PH overlay can update locally.
5. The proposed transition ALT uses ALT’s strength but on a topology-derived graph with much smaller state and interpretable route.
6. Experiments include ALT as main baseline and show where each method wins.

---

# 10. Discussion paragraph draft

> **Discussion.** Landmark-based A* is a particularly strong baseline for our setting. Because ALT performs search directly on the original surface graph, it preserves shortest-path optimality while using triangle-inequality lower bounds to greatly reduce node expansions. In our 0.1m experiments, this makes ALT faster and more accurate than our initial room-level planner, whose Euclidean abstract heuristic and fixed room-sequence stitching introduce both search overhead and abstraction error. We therefore do not view ALT as an orthogonal competitor to be ignored, but as a complementary goal-directed search principle. The revised planner applies landmark lower bounds on the persistent-homology transition graph, preserving the key advantages of our representation—seed-reachable 3D traversability extraction, semantic-free room decomposition, interpretable room/doorway routes, reusable local geodesic caches, and local update structure—while replacing the weak Euclidean room heuristic with graph-distance estimates. This reframes the method from a pure fastest-query planner into a topology-aware navigation representation whose benefits are most pronounced in multi-query, high-resolution, multi-floor, and task-constrained indoor navigation.

---

# 11. Concrete next-week action plan

## Day 1 — Instrument current code

* Add per-query logging:
  [
  R,\ |T|,\ |E_T|,\ \chi,\ N_R,\ N_T,\ N_{\text{fallback}},\ T_{\text{stitch}},\ T_{\text{fallback}}.
  ]
* Log fallback reason:

  * missing transfer pair；
  * no room path；
  * same-room special case；
  * floor graph failure；
  * portal/path reconstruction failure。
* Fix metric consistency:

  * same endpoints；
  * same snapping；
  * same edge cost；
  * report both length and cost。

## Day 2 — Build transition graph

* Implement boundary extraction:
  [
  B_{ij}={(u,v)\in E_S:\rho(u)=i,\rho(v)=j}.
  ]
* Cluster boundary components.
* Implement portal representatives:

  * center voxel；
  * geodesic medoid；
  * (K)-medoids later。
* Build (V_T,E_T):

  * intra-room transfer edges；
  * crossing edges；
  * optional direct same-room handling。

## Day 3 — Add ALT on transition graph

* Implement landmark selection:

  * random；
  * farthest；
  * high-degree/stair-biased。
* Precompute:
  [
  d_T(L,v).
  ]
* Implement heuristic:
  [
  h_T(v,g)=\max_L |d_T(L,g)-d_T(L,v)|.
  ]
* Add temporary start/goal connection:
  [
  d_T(L,g)=\min_{p\in T_{r_g}}d_T(L,p)+d_p(g).
  ]

## Day 4 — Integrate stitch reconstruction

* Reconstruct intra-room edges from existing parent trees.
* Reconstruct crossing edges.
* Add validation:

  * stitched path is connected；
  * all voxels are in reachable surface；
  * computed path cost matches sum of transition edge weights within tolerance。

## Day 5 — Run core ablations on 6 scenes

Run:

* current full；
* room Euclidean；
* room ALT；
* PH-ALT-TG with landmarks 4/8/16；
* portal count 1/2/4；
* tau 4/8/10；
* Surface ALT 16 as reference。

Report:

* query time；
* expanded nodes；
* cost gap；
* length gap；
* fallback rate；
* memory；
* preprocessing。

## Day 6 — Make figures

Prioritize these plots:

1. current full vs PH-ALT-TG vs Surface ALT scatter: query time vs cost gap；
2. portal count Pareto；
3. τ vs fallback rate；
4. transition node expansions；
5. failure case visualization for S3/S6；
6. amortized cost for Q=1/10/100/1000。

## Day 7 — Rewrite paper sections

* Abstract: remove overclaim, add topology-aware transition overlay.
* Method: add PH-ALT-TG.
* Experiments: promote ALT to main baseline.
* Discussion: use the paragraph above.
* Limitations: explicitly discuss abstraction error, portal budget, dynamic update assumptions.
* Add a short “Why ALT is strong and how we integrate it” subsection.

The fastest path to a stronger paper is: **keep PH as the structural contribution, replace weak room-level Euclidean search with transition-graph ALT, and evaluate the method as a topology-aware reusable navigation representation rather than a one-off shortest-path speed trick.**

[1]: https://www.cs.princeton.edu/courses/archive/spr06/cos423/Handouts/GH05.pdf?utm_source=chatgpt.com "Computing the Shortest Path: A Search Meets Graph Theory"
[2]: https://core.ac.uk/display/24526413?utm_source=chatgpt.com "Near optimal hierarchical path-finding - CORE"
[3]: https://link.springer.com/chapter/10.1007/978-3-540-68552-4_24?utm_source=chatgpt.com "Contraction Hierarchies: Faster and Simpler Hierarchical Routing in ..."
[4]: https://www.microsoft.com/en-us/research/publication/a-hub-based-labeling-algorithm-for-shortest-paths-on-road-networks-2/?utm_source=chatgpt.com "A Hub-Based Labeling Algorithm for Shortest Paths on Road Networks"
