# v16 Final Results: Stitched-Geodesic Planner Resolution Sweep

**Branch**: `feat/stitched-geodesic-planner`  
**Commit**: `c655330` (reverted transfer_length, v16 Euclidean within_cost)  
**Date**: 2026-04-22

## Configuration
- roomDijkstra: penalty-weighted (eucl + z_cost + w_obs*clearance)
- roomAstar: Euclidean within_cost
- snapToSurface: tiered search r=5 -> r=20 -> r=50
- Budget: resolution-adaptive (500K * (0.1/res)^2)

## Summary Table (stitch-only, excluding fallback scenes)

| Resolution | JPS avg Ts | Topo avg Ts | Speedup | JPS nodes | Topo nodes | Δpath  |
|------------|-----------|------------|---------|-----------|-----------|--------|
| 0.100m     | 3.99ms    | 0.047ms    | 85x     | 15,652    | 0         | +16.8% |
| 0.075m     | 8.83ms    | 0.055ms    | 161x    | 30,952    | 0         | +33.5% |
| 0.050m     | 26.01ms   | 1.990ms    | 13x     | 65,504    | 0         | +16.9% |

Note: 0.05m average dragged by scene 2azQ1b91cZZ (1706 rooms, roomAstar=9ms).
Excluding that outlier: Topo avg = 0.19ms, Speedup ~137x.

## Per-Scene Results at 0.05m

| Scene        | JPS Ts  | Topo Ts  | Speedup | JPS path | Topo path | Δpath  |
|-------------|---------|----------|---------|----------|-----------|--------|
| 1LXtFkjw3qL | 30.60ms | 0.086ms  | 356x    | 31.69m   | 36.58m    | +15.4% |
| 1pXnuDYAj8r | FAIL    | FAIL     | -       | -        | -         | -      |
| 2azQ1b91cZZ | 14.41ms | 9.190ms  | 1.6x    | 38.78m   | 60.09m    | +55.0% |
| 2n8kARJN3HM | 32.06ms | 0.500ms  | 64x     | 42.28m   | 44.88m    | +6.1%  |
| 2t7WUuJeko7 | 21.34ms | 0.030ms  | 711x    | 13.21m   | 14.17m    | +7.3%  |
| 5LpN3gDmAk7 | 31.63ms | 0.143ms  | 221x    | 48.97m   | 49.27m    | +0.6%  |

## Timing Breakdown (from node log, 0.05m)

| Component  | Typical    | Outlier (1706 rooms) |
|-----------|------------|---------------------|
| snap      | 0.01ms     | 0.01ms              |
| room A*   | 0.02-0.18ms| 7.73ms              |
| stitch    | 0.01ms     | 0.02ms              |
| **total** | **0.04ms** | **7.8ms**           |

## Key Optimizations (this session)

1. **Tiered snap** (r=5->20->50): 3ms -> 0.01ms. Most points on surface in first tier.
2. **Resolution-adaptive budget**: Maintains precompute coverage at fine resolutions.
3. **Dead end: transfer_length in roomAstar** — causes disconnected stitch paths (path < JPS optimal). Reverted.
