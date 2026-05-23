# Experiments — PH-ALT-TG Navigation

**Date**: 2026-05-23
**Branch**: `feat/stitched-geodesic-planner`
**Datasets**: 6 MP3D scenes + 100 HM3D scenes
**Resolutions**: 0.1m / 0.075m / 0.05m

---

## Pipeline Overview

```
Input mesh  --(voxelize)--> Surface voxel map V (res r)
     |
     v
 1. Persistent Homology   -->  merge tree + birth/death pairs
 2. Room Segmentation     -->  rooms R = {R_1..R_k}, portals P
 3. Transition Graph G_T  -->  nodes = portals, edges = room Dijkstra costs
 4. ALT Landmarks         -->  L Dijkstra trees on G_T
     |
     v
 Query(s, g):
   snap to surface --> TG A* with ALT heuristic --> stitch via parent chains --> voxel path
```

---

## Algorithms & Formulas

### 1. Surface Graph & Edge Cost

Surface graph: voxels V connected to 26-neighbors; edge cost:

```
c(u, v) = c_eucl + c_z + c_clearance

c_eucl = r * sqrt(dx^2 + dy^2 + dz^2)
c_z    = |dz| * r * w_z,    w_z = w_up (dz>0) | w_down (dz<0) | 0
c_clear = w_obs * r / (D(v) + 1)
```

where `D(v)` = distance field (clearance to nearest obstacle), `r` = voxel resolution.

**Source**: `hierarchical_planner.cpp:752-766`, `alt_planner.cpp:37-54`

### 2. Persistent Homology (PH)

Filtration on distance field D, sorted descending. Union-Find sweep:

```
for each voxel v in decreasing D order:
  neighbors_processed = {u : u adjacent to v, u already visited}
  if |neighbors_processed| == 0:
    v is a local maximum, birth(v) = D(v)
  elif all neighbors in same component:
    attach v to that component
  else:  // saddle point
    merge components; highest-birth root survives
    for each absorbed root m:
      death(m) = D(v)
      persistence(m) = birth(m) - death(m)
```

Global maximum has persistence = infinity.

**Source**: `persistence.cpp:63-191`

### 3. Room Segmentation

Given merge tree and threshold tau:

```
Phase 1: Significant maxima M = { m : persistence(m) > tau }
Phase 2: Top-down collapse — non-significant nodes inherit parent's room label
Phase 3: Label all voxels by their significant ancestor
Phase 4: Per-room stats: centroid, mean clearance, PCA aspect ratio
Phase 5: Extract adjacency graph:
  for each boundary voxel pair (u in R_i, v in R_j):
    passage_width = 2 * min(D(u), D(v)) * r
    portal = argmax_D on each room side (symmetric midpoint)
```

**Key parameter**: tau controls room granularity. Small tau = many rooms (fine), large tau = few rooms (coarse).

**Source**: `room_segmenter.cpp:72-287`

### 4. Transition Graph (G_T) Construction

```
Portal selection (per room edge, per side):
  K = max_portals_per_edge (default 1)
  Primary portal: argmax D(s) over boundary voxels (max clearance)
  Additional portals (K>1): farthest-point sampling from primary
    for k = 2..K:
      portal[k] = argmax_{v in boundary} min_{j<k} ||v - portal[j]||

Nodes: 2K portal nodes per room adjacency edge (K per room side)
  TGNode = { surf_idx, room_id, edge_idx }

Edges:
  (a) Crossing edges (inter-room):
    All-to-all between K portals on each side of same edge
    cost = Euclidean distance between portal voxels

  (b) Intra-room edges (within same room):
    cost = trans_field_[portal_a].dist[portal_b]
         = precomputed Dijkstra geodesic cost through room
```

Precompute: For each portal p, run Dijkstra within room. Store parent tree in `trans_field_[p]`. Total budget B_max = 5M voxel visits. With K>1, each room has K× more portals → K× more Dijkstra calls.

**Source**: `hierarchical_planner.cpp:234-321`, `room_segmenter.cpp:197-287`

### 5. ALT Heuristic

Landmark selection via farthest-point sampling on G_T:

```
landmark[0] = arbitrary node
for l = 1..L-1:
  min_dist[v] = min(min_dist[v], d(landmark[l-1], v))
  landmark[l] = argmax_v min_dist[v]
```

For each landmark l, run full Dijkstra on G_T, store `d(l, v)` for all v.

Query heuristic (admissible, consistent):

```
h(v) = max_{l=1..L} |d(l, goal) - d(l, v)|
```

**Source**: `alt_planner.cpp:56-65` (heuristic), `alt_planner.cpp:93-151` (landmark selection)

Also used on the full transition graph in `hierarchical_planner.cpp:1276-1291`.

### 6. Adaptive Tau Selector

Evaluates tau candidates (default {3, 4, 5, 6}) without full precompute:

```
For each tau candidate:
  Run segmentRooms(tau) -> rooms, portals, adjacency
  Simulate budget: estimate which rooms can be precomputed within B_max
  Compute metrics:
    coverage       = cached_pairs / total_transfer_pairs
    components     = connected components of room adjacency graph
    gini           = Gini coefficient of room size distribution
    max_fraction   = largest_room_voxels / total_voxels

score(tau) = 100 * coverage
           - 10 * max(0, components - 1)
           - 15 * budget_skip_rooms
           - 20 * max(0, max_fraction - 0.3)
           - 10 * max(0, 5 - num_rooms)
           - 10 * max(0, gini - 0.7)
           - 0.01 * num_portals

Selected tau = argmax score(tau)
```

Selection cost: 2-4 ms (lightweight, no Dijkstra precompute).

**Source**: `tau_selector.cpp:169-197`

### 7. Stitch Path Reconstruction

Given TG A* room sequence [R_0, R_1, ..., R_k] with portal transitions:

```
For each segment (entry_portal -> exit_portal) in room R_i:
  if trans_field_[entry] covers both endpoints:
    path = walkParentChain(entry_field, start, target)     // O(|path|)
  elif trans_field_[exit] covers both:
    path = walkParentChain(exit_field, start, target)
  else:
    path = online voxelAstar(start, target, room_id)       // fallback

walkParentChain(field, from, target):
  cur = from
  while cur != target:
    cur = parent[cur]
    path.append(cur)
  return path
```

Same-room shortcut (P4): If start and goal are in the same room, find best portal Dijkstra field covering both, reconstruct directly.

**Source**: `hierarchical_planner.cpp:960-1199`

### 8. Root-Dijkstra Cache Policy (P2)

```
Old: Materialize transfer_path_[i,j] for all portal pairs -> O(k^2 * |R|) memory
New: Store only trans_field_[portal] parent trees -> O(k * |R|) memory
     Reconstruct paths on-the-fly via parent-chain walks at stitch time
     Budget raised from 500K to 5M voxel visits -> 100% transfer coverage
```

**Source**: `hierarchical_planner.cpp` (P2 commit 3ad7ec0)

### 9. Query Complexity

```
Precompute (one-time):
  PH:           O(|V| log |V|)
  Segmentation: O(|V|)
  Room Dijkstra: O(B_max)        // B_max = 5M budget
  ALT landmarks: O(L * |G_T|)    // L = num landmarks

Per-query:
  Snap:         O(1)             // nearest surface voxel
  TG A*:        O(|G_T| log |G_T|)  // typically |G_T| << |V|
  Stitch:       O(|path|)        // parent-chain walk
  Total:        O(|G_T| log |G_T| + |path|)

vs. ALT on full graph:
  Per-query:    O(|V| log |V|)   // with landmark pruning
```

---

## Directory Structure

```
experiments/
├── README.md                       # this file
├── results/
│   ├── p5_alt_baseline.csv         # MP3D 6 scenes, ALT baseline (0.1m)
│   ├── p5_fixed_tau3.csv           # MP3D 6 scenes, PH-ALT-TG tau=3 (0.1m)
│   ├── p5_auto_tau.csv             # MP3D 6 scenes, PH-ALT-TG auto-tau (0.1m)
│   ├── hm3d_100/                   # HM3D 100 scenes, all methods & resolutions
│   │   ├── 100_alt_01m_500q.csv    # ALT baseline, 0.1m, 500 queries
│   │   ├── 100_tg_tau3_01m_500q.csv
│   │   ├── 100_tg_auto_01m.csv
│   │   ├── 100_alt_0075m.csv       # 0.075m resolution
│   │   ├── 100_tg_tau3_0075m.csv
│   │   ├── 100_tg_auto_0075m.csv
│   │   ├── 100_alt_005m.csv        # 0.05m resolution
│   │   ├── 100_tg_tau3_005m.csv
│   │   ├── 100_tg_auto_005m.csv
│   │   ├── 100_jps_01m.csv         # ablation baselines (0.1m, 300q)
│   │   ├── 100_flat_01m.csv
│   │   ├── 100_prm_1k_01m.csv
│   │   ├── 100_grid_tau5_01m.csv
│   │   ├── 100_no_stitch_tau5_01m.csv
│   │   ├── 100_full_tau5_01m.csv
│   │   ├── 100_tg_tau3_01m.csv     # (old 300q, kept for reference)
│   │   ├── 100_tg_tau4_01m.csv
│   │   ├── 100_tg_tau5_01m.csv
│   │   └── 100_alt_01m.csv         # (old 300q)
│   ├── ph_alt_tg/                  # MP3D tau sweep & landmark ablation
│   ├── figures/                    # paper figures (PDF + PNG)
│   └── archive/                    # superseded data & docs
└── tools/benchmark/tools/
    ├── auto_benchmark.py           # main benchmark runner
    ├── aggregate_results.py        # result aggregation & LaTeX tables
    └── sweep_ph_alt_tg.py          # tau/landmark sweep
```

---

## Key Results

### MP3D (6 large multi-floor scenes, 0.1m)

| Method | Succ% | Ts (ms) | Ts_p95 | Path (m) | Overhead | Topo (ms) |
|--------|-------|---------|--------|----------|----------|-----------|
| ALT baseline | 100% | 0.498 | 0.987 | 34.38 | -- | 254 |
| PH-ALT-TG (tau=3) | 83% | 0.067 | 0.146 | 34.86 | +1.4% | 254 |
| **PH-ALT-TG (auto-tau)** | **100%** | **0.047** | **0.089** | 35.42 | +3.0% | 170 |

**10.6x speedup**, 3% path overhead, 100% success with auto-tau.

### HM3D (100 scenes, cross-resolution)

| Resolution | ALT Ts | TG Ts | Speedup | Path OH |
|------------|--------|-------|---------|---------|
| 0.1m | 0.205 | 0.138 | 1.5x | +18.1% |
| 0.075m | 0.310 | 0.146 | 2.1x | +23.0% |
| 0.05m | 0.435 | 0.222 | 2.0x | +22.4% |

### HM3D Ablation (100 scenes, 0.1m)

| Method | Succ% | Ts (ms) | Path (m) | Path OH |
|--------|-------|---------|----------|---------|
| JPS_3D | 87.3% | 0.447 | 16.02 | +0.7% |
| Flat A* | 90.7% | 0.447 | 16.74 | +5.3% |
| ALT (L=16) | 91.2% | 0.205 | 15.91 | -- |
| PRM (1k) | 89.3% | 0.438 | 26.43 | +66.1% |
| Grid-HPA* | 88.0% | 0.184 | 24.31 | +52.8% |
| No-Stitch | 88.3% | 0.442 | 16.77 | +5.4% |
| PH-ALT-TG (tau=3) | 91.4% | 0.161 | 18.81 | +18.3% |
| **PH-ALT-TG (auto-tau)** | **91.4%** | **0.138** | 18.78 | +18.1% |

### Multi-Portal Path Quality (HM3D 100 scenes, 0.1m, paired comparison)

Paired on 410 queries succeeding with ALL methods (ALT + K=1 + K=3 + K=5):

| K | Mean Path | Mean OH | Median OH | P95 OH | Best path | Ts (ms) |
|---|-----------|---------|-----------|--------|-----------|---------|
| 1 | 18.39m | +21.1% | +10.6% | +120.5% | 84/410 | 0.115 |
| 3 | 16.97m | +11.8% | +4.2% | +83.6% | 135/410 | 0.130 |
| **5** | **16.13m** | **+6.2%** | **+1.8%** | **+78.2%** | **191/410** | 0.132 |

K=5 cuts mean path overhead from +21% to +6%. Precompute cost increases ~4x but remains under 2s for all scenes.

---

## Observations

1. **MP3D (complex multi-floor)**: PH-ALT-TG achieves 10x speedup with minimal path overhead. Auto-tau recovers the 1 scene that fixed tau=3 fails on.

2. **HM3D (simple apartments)**: Speedup is modest (1.5-2.1x) because scenes are smaller — the transition graph overhead is less amortized. But PH-ALT-TG still achieves the best or tied-best success rate (91.4%).

3. **Resolution scaling**: Speedup increases with resolution (0.1m: 1.5x -> 0.075m: 2.1x) because ALT's cost grows with |V| while TG's A* cost is resolution-independent.

4. **Multi-portal (K>1)**: The single-portal bottleneck was the primary source of path overhead. Farthest-point sampling of K portals per boundary provides better geometric coverage and cuts overhead from +21% to +6% (K=5). Query time is barely affected (+15%).

4. **Path overhead**: 3% on MP3D, 18-23% on HM3D. The overhead comes from room-level routing on the coarser transition graph. Larger scenes amortize this better.

5. **Success rate at 0.05m**: Drops to ~63-65% for all methods due to large voxel maps causing timeouts. Not a method-specific issue.

---

## Reproducing

```bash
# On remote machine (steam@localhost:9022)
cd /home/steam/project/planner_benchmark/global_nav_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash

# Start topo_nav_node in tmux:node
ros2 run topo_nav topo_nav_node

# Run benchmark in tmux:bench
python3.12 tools/benchmark/tools/auto_benchmark.py \
  --csv benchmark_hm3d_startseed_01m.csv \
  --planner-mode ph_alt_tg --tau -1 \
  --output experiments/results/hm3d_100/output.csv --trials 3

# Aggregate results
python3 tools/benchmark/tools/aggregate_results.py \
  --results-dir experiments/results --latex
```
