#!/bin/bash
set -e
cd /home/steam/project/planner_benchmark/global_nav_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

CSV=benchmark_hm3d_startseed_01m.csv
OUTDIR=experiments/results/hm3d_100
TRIALS=3

echo "========================================"
echo "  P6.1: PRM sample sweep HM3D 100 scenes"
echo "========================================"

for N in 2000 5000 10000; do
  OUT="$OUTDIR/100_prm_${N}_01m_500q.csv"
  if [ -f "$OUT" ]; then
    echo "--- SKIP PRM N=$N (exists: $OUT) ---"
    continue
  fi
  echo "--- PRM N=$N ---"
  python3.12 tools/benchmark/tools/auto_benchmark.py \
    --csv "$CSV" --planner-mode prm --prm-samples $N \
    --output "$OUT" --trials $TRIALS --force
done

echo "========================================"
echo "  P6.1 COMPLETE"
echo "========================================"
date
