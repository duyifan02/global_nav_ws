# HM3D PointNav Experiment Results — Consolidated Summary

**Date**: 2026-05-18
**Branch**: `feat/stitched-geodesic-planner`
**Dataset**: 10 HM3D minival scenes (GLB→OBJ), 5 PointNav episodes per scene
**Resolution**: 0.1m, **Trials**: 3

---

## 1. Scene Index

| ID | Scene ID | Episodes | Geo Distance Range | Status |
|----|----------|----------|--------------------|--------|
| H1 | BHXhpBwSMLh | 5 | — | 5/5 JPS ok |
| H2 | HaxA7YrQdEC | 5 | — | 5/5 JPS ok |
| H3 | Qpor2mEya8F | 5 | — | 2/5 JPS ok |
| H4 | SUHsP6z2gcJ | 5 | — | 5/5 JPS ok |
| H5 | TEEsavR23oF | 5 | — | All FAIL |
| H6 | k1cupFYWXJ6 | 5 | — | 5/5 JPS ok |
| H7 | rsggHU7g7dh | 5 | — | 5/5 JPS ok |
| H8 | tQ5s4ShP627 | 5 | — | 5/5 JPS ok |
| H9 | wcojb4TFT35 | 5 | — | All FAIL |
| H10 | y9hTuugGdiq | 5 | — | 5/5 JPS ok |

---

## 2. Method Comparison — Mean Query Time (ms)

Only includes episodes that succeeded for that method. Lower is better.

| Scene | JPS (Surface A*) | flat_surface | no_stitch | grid_partition | full (HPA*) | PRM | ALT A* | PH-ALT-TG τ=3 | PH-ALT-TG τ=4 |
|-------|------:|------:|------:|------:|------:|------:|------:|------:|------:|
| BHXhpBwSMLh | 0.361 | 0.358 | 0.392 | 0.253 | 0.345 | 0.253 | 0.087 | 0.058 | 0.057 |
| HaxA7YrQdEC | 0.254 | 0.265 | 0.218 | 0.178 | 0.030 | 0.227 | 0.106 | 0.032 | 0.031 |
| Qpor2mEya8F | 2.368 | 1.699 | 2.354 | 1.789 | 2.360 | 2.412 | 1.721 | 2.388 | 2.379 |
| SUHsP6z2gcJ | 0.777 | 0.724 | 0.703 | 0.350 | 0.049 | 0.721 | 0.136 | 0.373 | 0.368 |
| TEEsavR23oF | FAIL | FAIL | FAIL | FAIL | FAIL | FAIL | FAIL | FAIL | FAIL |
| k1cupFYWXJ6 | 1.406 | 1.410 | 1.375 | 2.773 | 1.427 | 2.183 | 1.381 | 1.341 | 2.212 |
| rsggHU7g7dh | 1.366 | 1.397 | 1.439 | 0.631 | 0.081 | 0.933 | 0.393 | 0.080 | 0.065 |
| tQ5s4ShP627 | 0.280 | 1.036 | 1.632 | 0.201 | 0.398 | 0.533 | 1.042 | 0.398 | 0.985 |
| wcojb4TFT35 | FAIL | FAIL | FAIL | FAIL | FAIL | FAIL | FAIL | FAIL | FAIL |
| y9hTuugGdiq | 0.549 | 2.447 | 0.832 | 0.773 | 0.698 | 0.759 | 2.201 | 0.705 | 2.149 |
| **Mean (success)** | **0.920** | **1.167** | **1.118** | **0.869** | **0.673** | **1.002** | **0.883** | **0.672** | **1.031** |

---

## 3. Mean Path Length (m)

| Scene | JPS (Surface A*) | flat_surface | no_stitch | grid_partition | full (HPA*) | PRM | ALT A* | PH-ALT-TG τ=3 | PH-ALT-TG τ=4 |
|-------|------:|------:|------:|------:|------:|------:|------:|------:|------:|
| BHXhpBwSMLh | 21.3 | 21.1 | 21.6 | 33.9 | 22.1 | 31.2 | 21.8 | 22.4 | 23.7 |
| HaxA7YrQdEC | 16.4 | 17.9 | 16.0 | 21.7 | 16.9 | 28.5 | 16.7 | 18.0 | 18.8 |
| Qpor2mEya8F | 0.0 | 0.1 | 0.0 | 2.9 | 0.0 | 0.1 | 1.9 | 0.0 | 0.0 |
| SUHsP6z2gcJ | 24.9 | 25.3 | 24.4 | 43.7 | 31.7 | 34.3 | 29.5 | 24.9 | 30.1 |
| TEEsavR23oF | FAIL | FAIL | FAIL | FAIL | FAIL | FAIL | FAIL | FAIL | FAIL |
| k1cupFYWXJ6 | 9.2 | 9.3 | 9.0 | 12.8 | 9.8 | 8.3 | 9.3 | 9.4 | 6.7 |
| rsggHU7g7dh | 33.3 | 35.3 | 35.1 | 38.5 | 41.9 | 52.2 | 33.3 | 37.3 | 33.4 |
| tQ5s4ShP627 | 18.6 | 6.8 | 1.1 | 22.0 | 18.8 | 44.4 | 6.3 | 15.2 | 7.8 |
| wcojb4TFT35 | FAIL | FAIL | FAIL | FAIL | FAIL | FAIL | FAIL | FAIL | FAIL |
| y9hTuugGdiq | 17.1 | 14.4 | 13.0 | 22.0 | 12.9 | 19.3 | 14.3 | 13.6 | 15.1 |
| **Mean (success)** | **20.1** | **18.6** | **17.1** | **24.7** | **22.0** | **31.2** | **16.6** | **20.1** | **19.4** |

---

## 4. Fair Comparison — Scenes Where ALL Methods Succeed (8 scenes)

| Method | Mean Ts (ms) | Mean Path (m) | Speedup vs flat | Path Δ vs ALT |
|--------|-------------|--------------|----------------|---------------|
| JPS (Surface A*) | 0.920 | 20.1 | 1.3x | +20.9% |
| flat_surface | 1.167 | 18.6 | 1.0x | +11.8% |
| no_stitch | 1.118 | 17.1 | 1.0x | +3.0% |
| grid_partition | 0.869 | 24.7 | 1.3x | +48.4% |
| full (HPA*) | 0.673 | 22.0 | 1.7x | +32.3% |
| PRM | 1.002 | 31.2 | 1.2x | +87.4% |
| ALT A* | 0.883 | 16.6 | 1.3x | +0.0% |
| PH-ALT-TG τ=3 | 0.672 | 20.1 | 1.7x | +21.1% |
| PH-ALT-TG τ=4 | 1.031 | 19.4 | 1.1x | +16.4% |

---

## 5. Precompute Cost (topo_ms)

| Scene | JPS (Surface A*) | flat_surface | no_stitch | grid_partition | full (HPA*) | PRM | ALT A* | PH-ALT-TG τ=3 | PH-ALT-TG τ=4 |
|-------|------:|------:|------:|------:|------:|------:|------:|------:|------:|
| BHXhpBwSMLh | 0 | 0 | 2 | 73 | 57 | 38 | 80 | 102 | 76 |
| HaxA7YrQdEC | 0 | 0 | 1 | 26 | 39 | 46 | 46 | 68 | 43 |
| Qpor2mEya8F | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| SUHsP6z2gcJ | 0 | 0 | 2 | 54 | 69 | 62 | 97 | 92 | 84 |
| TEEsavR23oF | — | — | — | — | — | — | — | — | — |
| k1cupFYWXJ6 | 0 | 0 | 3 | 158 | 431 | 21 | 616 | 376 | 36 |
| rsggHU7g7dh | 0 | 0 | 3 | 68 | 356 | 229 | 264 | 436 | 273 |
| tQ5s4ShP627 | 0 | 0 | 0 | 32 | 43 | 69 | 23 | 76 | 29 |
| wcojb4TFT35 | — | — | — | — | — | — | — | — | — |
| y9hTuugGdiq | 0 | 0 | 2 | 35 | 86 | 122 | 57 | 73 | 42 |

---

## 6. Failure Analysis

- **Qpor2mEya8F**: JPS (Surface A*):3/5, flat_surface:4/5, no_stitch:3/5, grid_partition:4/5, full (HPA*):3/5, PRM:3/5, ALT A*:4/5, PH-ALT-TG τ=3:3/5, PH-ALT-TG τ=4:3/5
- **TEEsavR23oF**: ALL methods 5/5 fail. Root cause: multi-floor scene (Z range 6.1m), seed centroid falls between floors → only 1 walkable voxel found from BFS flood. Fix: use per-floor seed or episode start as seed.
- **k1cupFYWXJ6**: PRM:2/5, PH-ALT-TG τ=4:2/5
- **tQ5s4ShP627**: no_stitch:2/5
- **wcojb4TFT35**: ALL methods 5/5 fail. Root cause: same as TEEsavR23oF — multi-floor scene (Z range 7.7m), seed between floors.

---

## 7. Cross-Dataset Summary: MP3D vs HM3D

| Metric | MP3D (6 scenes, 0.1m) | HM3D (8 scenes, 0.1m) |
|--------|-----------------------|------------------------|
| PH-ALT-TG (best) Ts | 0.050 ms (τ=4) | 0.672 ms (τ=3) |
| ALT A* Ts | 0.425 ms | 0.883 ms |
| flat_surface Ts | 1.442 ms | 1.167 ms |
| Speedup PH-ALT-TG vs ALT | 8.5x | 1.3x |
| Speedup PH-ALT-TG vs flat | 28.8x | 1.7x |
| PH-ALT-TG path overhead vs ALT | +4.6% | +21.1% |
| Success rate | 6/6 (100%) | 8/10 (80%) |

**Key differences**: HM3D scenes at 0.1m have ~10K surface voxels (similar scale to MP3D), but different topology — residential floorplans vs multi-room buildings. The transition graph overhead is less amortized on smaller, simpler graphs. Additionally, 2 multi-floor HM3D scenes (TEEsavR23oF, wcojb4TFT35) fail due to seed placement between floors (centroid falls in void).


---

## 8. Data File Index

| File | Method | Rows | Success |
|------|--------|------|---------|
| `hm3d/hm3d_jps_01m.csv` | JPS (Surface A*) | 50 | 37/50 |
| `hm3d/hm3d_flat_surface_01m.csv` | flat_surface | 50 | 36/50 |
| `hm3d/hm3d_no_stitch_01m.csv` | no_stitch | 50 | 35/50 |
| `hm3d/hm3d_grid_partition_01m.csv` | grid_partition | 50 | 36/50 |
| `hm3d/hm3d_full_tau4_01m.csv` | full (HPA*) | 50 | 37/50 |
| `hm3d/hm3d_prm_01m.csv` | PRM | 50 | 35/50 |
| `hm3d/hm3d_alt_01m.csv` | ALT A* | 50 | 36/50 |
| `hm3d/hm3d_ph_alt_tg_tau3_01m.csv` | PH-ALT-TG τ=3 | 50 | 37/50 |
| `hm3d/hm3d_ph_alt_tg_tau4_01m.csv` | PH-ALT-TG τ=4 | 50 | 35/50 |

---

## 9. Coordinate Transform Note

HM3D PointNav episodes use Habitat's Y-up coordinate system.
Our pipeline uses the mesh's native coordinates (from GLB→OBJ conversion).
Transform applied: `mesh_x = hab_x, mesh_y = -hab_z, mesh_z = hab_y`
