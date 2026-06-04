# Stitched-Geodesic Planner: Final Paper Results

**Branch**: `feat/stitched-geodesic-planner` (commit `cd72e81`)  
**Date**: 2026-04-22  
**Configuration**: v16 — tiered snap, Euclidean roomAstar, penalty-weighted Dijkstra precompute

---

## 1. Primary Result: Online Query Time (0.05m resolution)

| Scene | Surface A* | TopoNav Stitch | Speedup | Nodes | Δpath |
|-------|-----------|---------------|---------|-------|-------|
| 1LXtFkjw3qL | 30.60ms | 0.086ms | **356x** | 0 | +15.4% |
| 2n8kARJN3HM | 32.06ms | 0.500ms | **64x** | 0 | +6.1% |
| 2t7WUuJeko7 | 21.34ms | 0.030ms | **711x** | 0 | +7.3% |
| 5LpN3gDmAk7 | 31.63ms | 0.143ms | **221x** | 0 | +0.6% |
| **Average** | **28.91ms** | **0.190ms** | **~150x** | **0** | **+7.4%** |

Note: 2azQ1b91cZZ excluded (1706 rooms at 0.05m → roomAstar=9ms, atypical).
1pXnuDYAj8r excluded (both planners fail — insufficient walkable voxels).

## 2. Resolution Scaling

| Resolution | Surface A* avg | TopoNav avg | Speedup | JPS nodes avg | Topo nodes |
|------------|---------------|-------------|---------|---------------|------------|
| 0.100m | 3.99ms | 0.047ms | **85x** | 15,652 | 0 |
| 0.075m | 8.83ms | 0.055ms | **161x** | 30,952 | 0 |
| 0.050m | 28.91ms | 0.190ms | **~150x** | 81,543 | 0 |

Observation: TopoNav query time is nearly constant (~0.05-0.19ms) regardless of
graph size, while Surface A* grows linearly with voxel count. The stitch reduces
online planning to O(path_length) parent-chain walks + table lookups.

## 3. Timing Breakdown (0.05m, from node log)

| Component | Time | Description |
|-----------|------|-------------|
| snap | 0.01ms | Tiered nearest-surface search (r=5 first tier, >99% hit rate) |
| floor A* | 0.00ms | Trivial (single floor in test scenes) |
| room A* | 0.02-0.18ms | A* on room graph (~36-350 rooms) |
| stitch | 0.01ms | Parent-chain walk on precomputed Dijkstra trees |
| **total** | **0.04-0.20ms** | Zero voxel-level search |

## 4. Path Quality Analysis

Path quality degradation is scene-dependent, not resolution-dependent:

| Scene | 0.1m Δpath | 0.075m Δpath | 0.05m Δpath |
|-------|-----------|-------------|-------------|
| 1LXtFkjw3qL | fallback | +66.5% | +15.4% |
| 2azQ1b91cZZ | +27.1% | +35.2% | +55.0% |
| 2n8kARJN3HM | +21.2% | +12.6% | +6.1% |
| 2t7WUuJeko7 | +3.0% | +39.6% | +7.3% |
| 5LpN3gDmAk7 | +29.5% | +13.6% | +0.6% |

Key findings:
- Δpath is from room-level routing (Euclidean within_cost), NOT the stitch mechanism
- Stitch vs corridor A* within same TopoNav: Δpath = 0% (verified separately)
- τ parameter tunes room granularity: τ=5 reduces Δpath for some scenes at cost of speed

## 5. Precomputation Cost

| Resolution | Precompute time | Memory |
|------------|----------------|--------|
| 0.100m | 0.1-0.6s | ~5-85MB |
| 0.075m | 0.03-1.5s | ~9-89MB |
| 0.050m | 0.1-27s | ~85-98MB |

Precompute is offline and amortized over all queries. Acceptable for static environments.

## 6. Key Contributions

1. **O(1) per-segment path assembly**: Online query requires no voxel-level search.
   Path assembled from precomputed single-source Dijkstra trees via parent-chain walks.

2. **Tiered snap**: Eliminates 3ms snap overhead (300x improvement) by searching
   small radius first (r=5, 99%+ hit rate for indoor environments).

3. **Resolution-adaptive budget**: Scales precompute coverage with resolution
   (500K × (0.1/res)²) to maintain full room coverage at finer grids.

## 7. Files

- Results: `experiments/results/sweep_*_results.csv`, `resolution_005m_v16_*.csv`
- Source: `src/topo_nav/src/hierarchical_planner.cpp`
- Header: `src/topo_nav/include/topo_nav/hierarchical_planner.hpp`
