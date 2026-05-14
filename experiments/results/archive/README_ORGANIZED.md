# Experiment Results — Organized Index

## Final Results (Paper-Ready)

| File | Description |
|------|-------------|
| `PAPER_RESULTS_FINAL.md` | v16 最终论文结果总结（速度、路径质量、分辨率扩展性） |
| `v16_final_sweep_summary.md` | v16 详细技术分析（含 timing breakdown、per-scene） |

## v16 Resolution Sweep (Final Configuration)

### 0.05m Resolution
| File | Planner | Scenes |
|------|---------|--------|
| `resolution_005m_v16_jps_results.csv` | Surface A* | S1-S6 |
| `resolution_005m_v16_topo_results.csv` | TopoNav Stitch | S1-S6 |
| `resolution_005m_tau5_topo_results.csv` | TopoNav (tau=5) | S1-S6 |

### 0.075m Resolution
| File | Planner | Scenes |
|------|---------|--------|
| `sweep_0075m_jps_results.csv` | Surface A* | S1-S6 |
| `sweep_0075m_topo_results.csv` | TopoNav Stitch | S1-S6 |
| `sweep_0075m_tau5_topo_results.csv` | TopoNav (tau=5) | S1-S6 |

### 0.10m Resolution
| File | Planner | Scenes |
|------|---------|--------|
| `sweep_01m_jps_results.csv` | Surface A* | S1-S6 |
| `sweep_01m_topo_results.csv` | TopoNav Stitch | S1-S6 |

### 0.03m Resolution (JPS only, TopoNav OOM)
| File | Planner | Scenes |
|------|---------|--------|
| `sweep_003m_jps_results.csv` | Surface A* | S1-S6 |

## Historical Versions (Archived)

| File | Version | Notes |
|------|---------|-------|
| `summary_comparison.csv` | v1/v2 | 0.1m baseline, 6 scenes |
| `timing_breakdown.csv` | v1 | JPS vs TopoNav timing components |
| `algorithm_versions.csv` | v1/v2 | Algorithm evolution metadata |
| `topo_v3_results.csv` | v3 | HPA* cap fix |
| `topo_v4_results.csv` | v4 | Corridor A* |
| `topo_v4b_results.csv` | v4b | Dijkstra optimization (non-det failure) |
| `topo_v4b_det_results.csv` | v4b-det | Deterministic occmaps, 6/6 success |
| `resolution_0.05m_results.csv` | pre-v16 | Early 0.05m test |
| `multi_query_50_results.csv` | v4b | 50-query amortization test |
| `sweep_01m_v17_topo_results.csv` | v17 | transfer_length (reverted) |

## Reports
| File | Language | Purpose |
|------|----------|---------|
| `汇报文档_Stitched_Geodesic_Planner.md` | 中文 | 领导汇报用技术文档 |
