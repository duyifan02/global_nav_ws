#!/bin/bash
# Run all ablation + baseline experiments
# Usage: bash run_all_experiments.sh
# Requires: topo_nav_node and jps_planner_node running in separate terminals

set -e

WS=/home/steam/project/planner_benchmark/global_nav_ws
TOOLS=$WS/tools/benchmark/tools
INPUT=$WS/experiments/ablation_benchmark_input.csv
RESULTS=$WS/experiments/results
OCCMAP=/home/steam/project/planner_benchmark/global_nav_ws/occmaps

source /opt/ros/jazzy/setup.bash
source $WS/install/setup.bash

TAU=8.0
TRIALS=5

echo "============================================"
echo "Running ablation + baseline experiments"
echo "Resolution: 0.1m, Tau: $TAU, Trials: $TRIALS"
echo "============================================"

# Config B: flat_surface (no hierarchy, surface A* on topo_nav node)
echo ""
echo ">>> Config B: flat_surface"
python3.12 $TOOLS/auto_benchmark.py \
  --csv $INPUT \
  --output $RESULTS/ablation_flat_surface_results.csv \
  --planner TopoNav --trials $TRIALS --force \
  --tau $TAU --planner-mode flat_surface \
  --occmap-dir $OCCMAP \
  --service-timeout 30 --map-load-timeout 300

# Config D: no_stitch (PH rooms + online voxel A* per room)
echo ""
echo ">>> Config D: no_stitch"
python3.12 $TOOLS/auto_benchmark.py \
  --csv $INPUT \
  --output $RESULTS/ablation_no_stitch_results.csv \
  --planner TopoNav --trials $TRIALS --force \
  --tau $TAU --planner-mode no_stitch \
  --occmap-dir $OCCMAP \
  --service-timeout 30 --map-load-timeout 300

# Config C: grid_partition (uniform grid rooms + HPA* pipeline)
echo ""
echo ">>> Config C: grid_partition"
python3.12 $TOOLS/auto_benchmark.py \
  --csv $INPUT \
  --output $RESULTS/ablation_grid_partition_results.csv \
  --planner TopoNav --trials $TRIALS --force \
  --tau $TAU --planner-mode grid_partition \
  --occmap-dir $OCCMAP \
  --service-timeout 30 --map-load-timeout 300

# Config E: full (existing, re-run for consistency)
echo ""
echo ">>> Config E: full"
python3.12 $TOOLS/auto_benchmark.py \
  --csv $INPUT \
  --output $RESULTS/ablation_full_results.csv \
  --planner TopoNav --trials $TRIALS --force \
  --tau $TAU --planner-mode full \
  --occmap-dir $OCCMAP \
  --service-timeout 30 --map-load-timeout 300

# Baseline: PRM
echo ""
echo ">>> Baseline: PRM"
python3.12 $TOOLS/auto_benchmark.py \
  --csv $INPUT \
  --output $RESULTS/baseline_prm_results.csv \
  --planner TopoNav --trials $TRIALS --force \
  --tau $TAU --planner-mode prm --prm-samples 1000 \
  --occmap-dir $OCCMAP \
  --service-timeout 60 --map-load-timeout 300

# Baseline: ALT A*
echo ""
echo ">>> Baseline: ALT A*"
python3.12 $TOOLS/auto_benchmark.py \
  --csv $INPUT \
  --output $RESULTS/baseline_alt_results.csv \
  --planner TopoNav --trials $TRIALS --force \
  --tau $TAU --planner-mode alt_astar --num-landmarks 16 \
  --occmap-dir $OCCMAP \
  --service-timeout 60 --map-load-timeout 600

# Tau sensitivity sweep
echo ""
echo ">>> Tau sensitivity sweep"
python3.12 $TOOLS/sweep_tau_full.py \
  --csv $INPUT \
  --taus 1,2,3,4,5,6,8,10 \
  --resolution 0.1 --trials 3 \
  --output $RESULTS/tau_sweep_01m_results.csv \
  --occmap-dir $OCCMAP \
  --planner-mode full

echo ""
echo "============================================"
echo "All experiments complete!"
echo "Results in: $RESULTS/"
echo "============================================"
