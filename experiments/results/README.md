# Experiment Results Index

## Quick Reference

| Analysis | File | Status |
|----------|------|--------|
| **Ablation + Baselines (0.1m)** | [`ABLATION_AND_BASELINE_ANALYSIS.md`](ABLATION_AND_BASELINE_ANALYSIS.md) | 2026-05-13, 7 configs |
| Resolution Sweep (0.05-0.1m) | [`PAPER_RESULTS_FINAL.md`](PAPER_RESULTS_FINAL.md) | 2026-04-22, v16 final |
| v16 Technical Details | [`v16_final_sweep_summary.md`](v16_final_sweep_summary.md) | 2026-04-22 |

## Data Files

### Ablation & Baseline (2026-05-13, 0.1m, tau=8, 5 trials)

| File | Config |
|------|--------|
| `ablation_flat_surface_results.csv` | Config B: unrestricted surface A* |
| `ablation_no_stitch_results.csv` | Config D: PH rooms, online A* |
| `ablation_grid_partition_results.csv` | Config C: uniform grid + HPA* |
| `ablation_full_results.csv` | Config E: full method (PH + stitch) |
| `baseline_prm_results.csv` | PRM baseline (1000 samples) |
| `baseline_alt_results.csv` | ALT A* baseline (16 landmarks) |
| `tau_sweep_01m_results.csv` | Tau sensitivity (1-10, 6 scenes) |

### Resolution Sweep (2026-04-22, v16)

| File | Resolution | Planner |
|------|-----------|---------|
| `sweep_01m_jps_results.csv` | 0.10m | Surface A* |
| `sweep_01m_topo_results.csv` | 0.10m | TopoNav |
| `sweep_0075m_jps_results.csv` | 0.075m | Surface A* |
| `sweep_0075m_topo_results.csv` | 0.075m | TopoNav |
| `sweep_0075m_tau5_topo_results.csv` | 0.075m | TopoNav (tau=5) |
| `resolution_005m_v16_jps_results.csv` | 0.05m | Surface A* |
| `resolution_005m_v16_topo_results.csv` | 0.05m | TopoNav |
| `resolution_005m_tau5_topo_results.csv` | 0.05m | TopoNav (tau=5) |
| `sweep_003m_jps_results.csv` | 0.03m | Surface A* (JPS only) |

### Archive

Historical version results (v1-v4b) in `archive/`. See [`ABLATION_AND_BASELINE_ANALYSIS.md` Section 8](ABLATION_AND_BASELINE_ANALYSIS.md#8-data-files) for full listing.
