# Experiments — TopoNav vs JPS_3D

## Directory Structure

```
experiments/
├── notes/
│   └── 2026-03-31_benchmark_notes.md  — 详细实验记录、参数配置、根因分析
└── results/
    ├── README.md                       — 高层摘要：各版本跨场景对比表
    ├── algorithm_versions.csv          — 算法版本变更日志
    ├── summary_comparison.csv          — 所有版本 × 场景：Ts / path / nodes
    ├── timing_breakdown.csv            — 详细时序分解：te_ms / topo_ms / Ts
    ├── topo_v3_results.csv             — v3 原始数据（roomAstar v2 + HPA* cap）
    ├── topo_v4_results.csv             — v4 原始数据（Corridor A* 初版）
    └── topo_v4b_results.csv            — v4b 原始数据（Corridor A* + roomDijkstra）
```

JPS baseline 原始数据在工作区根目录的 `benchmark.csv` 中（JPS_3D 行）。

## Algorithm Version Summary

| 版本 | 关键改动 | path Δ vs JPS | topo_ms 均值 | 节点数均值 |
|------|---------|--------------|-------------|-----------|
| v1 | roomAstar v1（inadmissible h，min_exit_cost） | +7.5% | ~43ms（含201ms异常） | ~361（统计 bug） |
| v3 | roomAstar v2（admissible h，精确HPA*代价），cap=90 | +6.5% | 7.7ms | ~389（仍为统计 bug） |
| v4 | Corridor A*，roomDijkstra，正确节点统计 | +1.0%（5/6场景） | 10ms | ~17k（与JPS相当） |
| **v4b** | kMaxEdgesForDijkstra 15→10 | **+1.0%** | **10ms** | **~18k** |

详细对比见 `results/README.md`。
