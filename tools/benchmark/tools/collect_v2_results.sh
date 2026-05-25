#!/bin/bash
# Collect v2 benchmark results from remote and regenerate figures
set -e

LOCAL="$(cd "$(dirname "$0")/../../.." && pwd)"
REMOTE="steam@localhost"
RDIR="/home/steam/project/planner_benchmark/global_nav_ws"
RESULTS="$LOCAL/experiments/results/hm3d_100"

log() { echo "[$(date '+%H:%M:%S')] $*"; }

# Check if final benchmark is done
done=$(ssh -p 9022 $REMOTE "grep -c 'FINAL_BENCHMARK_COMPLETE' /tmp/final_benchmark.log 2>/dev/null; true" 2>/dev/null | head -1)
if [ "${done:-0}" -lt 1 ]; then
    log "Final benchmark not yet complete. Checking progress..."
    ssh -p 9022 $REMOTE "tail -5 /tmp/final_benchmark.log 2>/dev/null"
    exit 1
fi

log "Final benchmark complete! Collecting results..."

# Copy v2 result files
for f in 100_tg_auto_k5_01m_500q_v2.csv 100_alt_01m_500q_v2.csv 100_tg_auto_k1_01m_500q_v2.csv; do
    if scp -P 9022 "$REMOTE:$RDIR/experiments/results/hm3d_100/$f" "$RESULTS/$f" 2>/dev/null; then
        log "  Copied $f ($(wc -l < "$RESULTS/$f") lines)"
    else
        log "  SKIP $f (not found)"
    fi
done

# Also copy bridge test results
if scp -P 9022 "$REMOTE:/tmp/bridge_test.csv" "$RESULTS/100_tg_auto_k5_01m_500q_bridge.csv" 2>/dev/null; then
    log "  Copied bridge_test.csv"
fi

# Copy MP3D v2 results if available
MP3D_RESULTS="$LOCAL/experiments/results/mp3d_6"
for f in tg_auto_k5_01m_v2.csv alt_01m_v2.csv tg_auto_k1_01m_v2.csv; do
    if scp -P 9022 "$REMOTE:$RDIR/experiments/results/mp3d_6/$f" "$MP3D_RESULTS/$f" 2>/dev/null; then
        log "  Copied MP3D $f ($(wc -l < "$MP3D_RESULTS/$f") lines)"
    else
        log "  SKIP MP3D $f (not found)"
    fi
done

# Validity + speedup check
log "=== VALIDITY CHECK ==="
python3 -c "
import csv, statistics

def load(path):
    try:
        with open(path) as f:
            return list(csv.DictReader(f))
    except FileNotFoundError:
        return None

rdir = '$RESULTS'

for name in ['100_tg_auto_k5_01m_500q_v2', '100_tg_auto_k1_01m_500q_v2', '100_alt_01m_500q_v2']:
    rows = load(f'{rdir}/{name}.csv')
    if not rows:
        print(f'{name}: NOT FOUND'); continue
    succ = [r for r in rows if r.get('success')=='True']
    valid = [r for r in succ if r.get('path_valid')=='True']
    ts = [float(r['planning_time_ms']) for r in succ]
    snap = [float(r.get('diag_snap_ms','0')) for r in succ if r.get('diag_snap_ms')]
    snap_outlier = sum(1 for s in snap if s > 1.0)
    print(f'{name}:')
    print(f'  Total: {len(rows)} queries, Success: {len(succ)}/{len(rows)} ({len(succ)/len(rows)*100:.1f}%)')
    print(f'  Valid: {len(valid)}/{len(succ)} ({len(valid)/len(succ)*100:.1f}%)')
    print(f'  Ts: mean={statistics.mean(ts):.4f}ms  med={statistics.median(ts):.4f}ms  p95={sorted(ts)[int(0.95*len(ts))]:.4f}ms')
    if snap:
        print(f'  Snap: mean={statistics.mean(snap):.4f}ms  p95={sorted(snap)[int(0.95*len(snap))]:.4f}ms  outliers(>1ms): {snap_outlier}/{len(snap)}')
    print()

# Paired speedup
alt_rows = load(f'{rdir}/100_alt_01m_500q_v2.csv')
tg_rows = load(f'{rdir}/100_tg_auto_k5_01m_500q_v2.csv')
if alt_rows and tg_rows:
    def qkey(r):
        return f\"{r.get('scene','')},{r.get('start_x')},{r.get('start_y')},{r.get('start_z')},{r.get('goal_x')},{r.get('goal_y')},{r.get('goal_z')}\"
    alt_succ = {qkey(r): r for r in alt_rows if r.get('success')=='True'}
    tg_succ = {qkey(r): r for r in tg_rows if r.get('success')=='True' and r.get('path_valid')=='True'}
    common = set(alt_succ) & set(tg_succ)
    ratios = [float(alt_succ[k]['planning_time_ms'])/float(tg_succ[k]['planning_time_ms']) for k in common if float(tg_succ[k]['planning_time_ms']) > 0]
    overheads = [(float(tg_succ[k]['path_length_m'])/float(alt_succ[k]['path_length_m'])-1)*100 for k in common if float(alt_succ[k]['path_length_m']) > 0]
    if ratios:
        print(f'=== PAIRED COMPARISON (TG K=5 vs ALT, {len(common)} common queries) ===')
        print(f'  Speedup: mean={statistics.mean(ratios):.2f}x  med={statistics.median(ratios):.2f}x')
        print(f'  TG faster: {sum(1 for r in ratios if r>1)}/{len(ratios)} ({sum(1 for r in ratios if r>1)/len(ratios)*100:.1f}%)')
        print(f'  Overhead: mean={statistics.mean(overheads):+.1f}%  med={statistics.median(overheads):+.1f}%')
"

# Full regeneration (analysis + figures + tables)
log "=== REGENERATING ALL ==="
cd "$LOCAL"
bash tools/benchmark/tools/regenerate_all.sh
