#!/bin/bash
# Monitor remote experiments and auto-process results when complete
# Usage: bash monitor_remote.sh [check_interval_seconds]

INTERVAL=${1:-120}
REMOTE_DIR="/home/steam/project/planner_benchmark/global_nav_ws"
LOCAL_DIR="$(cd "$(dirname "$0")/../../.." && pwd)"
RESULTS="$LOCAL_DIR/experiments/results/hm3d_100"

log() { echo "[$(date '+%H:%M:%S')] $*"; }

rcount() {
  # Count pattern matches in remote file; returns integer
  local n
  n=$(ssh -p 9022 steam@localhost "grep -c '$1' '$2' 2>/dev/null; true" 2>/dev/null | awk 'NR==1{print $1+0}')
  echo "${n:-0}"
}

scpget() {
  scp -P 9022 "steam@localhost:$1" "$2" 2>/dev/null
}

# ── Phase 1: Wait for P4.1 tau sweep ──────────────────────────────────────
wait_tau_sweep() {
  log "=== Monitoring P4.1 tau sweep ==="
  while true; do
    local scenes done_marker
    scenes=$(rcount '\[SCENE\]' /tmp/p4_tau_sweep.log)
    done_marker=$(rcount 'P4.1 COMPLETE' /tmp/p4_tau_sweep.log)
    log "P4.1: $scenes scenes, done=$done_marker"
    if [ "$done_marker" -ge 1 ]; then
      log "P4.1 COMPLETE! Copying results..."
      for tau in 3 4 5 6; do
        local f="100_tg_tau${tau}_k5_01m_500q.csv"
        if scpget "$REMOTE_DIR/experiments/results/hm3d_100/$f" "$RESULTS/$f"; then
          log "  Copied $f ($(wc -l < "$RESULTS/$f") lines)"
        else
          log "  SKIP $f (not found)"
        fi
      done
      return 0
    fi
    sleep "$INTERVAL"
  done
}

# ── Phase 2: Wait for P1.3 cross-resolution ───────────────────────────────
wait_cross_res() {
  log "=== Monitoring P1.3 cross-resolution ==="
  while true; do
    local done_marker
    done_marker=$(rcount 'P1.3 COMPLETE' /tmp/p13_cross_res.log)
    log "P1.3: done=$done_marker"
    if [ "$done_marker" -ge 1 ]; then
      log "P1.3 COMPLETE! Copying results..."
      for f in 100_alt_0075m_500q.csv 100_tg_auto_k5_0075m_500q.csv \
               100_alt_005m_500q.csv 100_tg_auto_k5_005m_500q.csv; do
        if scpget "$REMOTE_DIR/experiments/results/hm3d_100/$f" "$RESULTS/$f"; then
          log "  Copied $f ($(wc -l < "$RESULTS/$f") lines)"
        else
          log "  SKIP $f (not found)"
        fi
      done
      return 0
    fi
    sleep "$INTERVAL"
  done
}

# ── Phase 3: Wait for P6.1 PRM sweep ─────────────────────────────────────
wait_prm_sweep() {
  log "=== Monitoring P6.1 PRM sweep ==="
  while true; do
    local done_marker
    done_marker=$(rcount 'P6.1 COMPLETE' /tmp/p6_prm_sweep.log)
    log "P6.1: done=$done_marker"
    if [ "$done_marker" -ge 1 ]; then
      log "P6.1 COMPLETE! Copying results..."
      for n in 2000 5000 10000; do
        local f="100_prm_${n}_01m_500q.csv"
        if scpget "$REMOTE_DIR/experiments/results/hm3d_100/$f" "$RESULTS/$f"; then
          log "  Copied $f ($(wc -l < "$RESULTS/$f") lines)"
        else
          log "  SKIP $f (not found)"
        fi
      done
      return 0
    fi
    sleep "$INTERVAL"
  done
}

# ── Analysis runners ──────────────────────────────────────────────────────
run_tau_analysis() {
  log "=== Running tau oracle analysis ==="
  cd "$LOCAL_DIR"
  python3 tools/benchmark/tools/analyze_tau_oracle.py \
    --alt experiments/results/hm3d_100/100_alt_01m_500q.csv \
    --sweep experiments/results/hm3d_100/100_tg_tau3_k5_01m_500q.csv \
            experiments/results/hm3d_100/100_tg_tau4_k5_01m_500q.csv \
            experiments/results/hm3d_100/100_tg_tau5_k5_01m_500q.csv \
            experiments/results/hm3d_100/100_tg_tau6_k5_01m_500q.csv \
    --auto experiments/results/hm3d_100/100_tg_auto_k5_01m_500q.csv \
    --out experiments/results/analysis/tau_oracle.csv
  log "Tau oracle analysis done"
}

run_figures() {
  log "=== Regenerating paper figures ==="
  cd "$LOCAL_DIR"
  python3 tools/benchmark/tools/make_paper_figures.py \
    --results-dir experiments/results --out-dir experiments/figures
  python3 tools/benchmark/tools/plot_paper_figures.py \
    --data-dir experiments/figures/data \
    --analysis-dir experiments/results/analysis \
    --out-dir experiments/figures
  log "Figures regenerated"
}

run_aggregate() {
  log "=== Running aggregate results ==="
  cd "$LOCAL_DIR"
  python3 tools/benchmark/tools/aggregate_results.py \
    --results-dir experiments/results
  log "Aggregate done"
}

# ── Main ──────────────────────────────────────────────────────────────────
log "Monitor started (interval=${INTERVAL}s)"
log "Local: $LOCAL_DIR"
log "Remote: $REMOTE_DIR"

# Phase 1: tau sweep
wait_tau_sweep
run_tau_analysis
run_figures
log ">>> P4.1 processing complete <<<"

# Phase 2: cross-res
wait_cross_res
run_aggregate
run_figures
log ">>> P1.3 processing complete <<<"

# Phase 3: PRM sweep
wait_prm_sweep
run_figures
log ">>> P6.1 processing complete <<<"

log "=========================================="
log "  ALL EXPERIMENTS COMPLETE AND PROCESSED"
log "=========================================="
