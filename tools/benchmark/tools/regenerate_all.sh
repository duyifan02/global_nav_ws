#!/bin/bash
# Regenerate all analysis CSVs and paper figures.
# Run from the workspace root after collecting v2 results.
set -e

LOCAL="$(cd "$(dirname "$0")/../../.." && pwd)"
cd "$LOCAL"

TOOLS="tools/benchmark/tools"
RESULTS="experiments/results"
HM3D="$RESULTS/hm3d_100"
MP3D="$RESULTS/mp3d_6"
ANALYSIS="$RESULTS/analysis"
FIGURES="experiments/figures"

log() { echo "[$(date '+%H:%M:%S')] $*"; }

mkdir -p "$ANALYSIS" "$FIGURES" "$FIGURES/data"

# Helper: pick v2 file if it exists, otherwise fall back to original
pick() {
    local base="$1"
    local v2="${base%.csv}_v2.csv"
    if [ -f "$v2" ]; then echo "$v2"; else echo "$base"; fi
}

ALT_HM3D=$(pick "$HM3D/100_alt_01m_500q.csv")
TG_K1_HM3D=$(pick "$HM3D/100_tg_auto_k1_01m_500q.csv")
TG_K5_HM3D=$(pick "$HM3D/100_tg_auto_k5_01m_500q.csv")

log "Using files:"
log "  ALT: $(basename $ALT_HM3D)"
log "  TG K=1: $(basename $TG_K1_HM3D)"
log "  TG K=5: $(basename $TG_K5_HM3D)"

log "=== 1/6: Overhead analysis ==="
python3 "$TOOLS/analyze_overhead.py" \
    --alt "$ALT_HM3D" \
    --tg "$TG_K1_HM3D" "$TG_K5_HM3D" \
    --label "tg_k1" "tg_k5" \
    --out-dir "$ANALYSIS" 2>&1 | tail -5

log "=== 2/6: Scene complexity ==="
python3 "$TOOLS/analyze_scene_complexity.py" \
    --alt "$ALT_HM3D" \
    --tg "$TG_K5_HM3D" \
    --out "$ANALYSIS/scene_complexity_k5.csv" 2>&1 | tail -5

log "=== 3/6: Amortization (HM3D) ==="
python3 "$TOOLS/analyze_amortization.py" \
    --alt "$ALT_HM3D" \
    --tg "$TG_K1_HM3D" "$TG_K5_HM3D" \
    --label "TG_K1" "TG_K5" \
    --out "$ANALYSIS/amortization_hm3d_01m.csv" \
    --dataset hm3d 2>&1 | tail -5

log "=== 3b/6: Amortization (MP3D) ==="
ALT_MP3D=$(pick "$MP3D/alt_01m.csv")
TG_K1_MP3D=$(pick "$MP3D/tg_auto_k1_01m.csv")
TG_K5_MP3D=$(pick "$MP3D/tg_auto_k5_01m.csv")
if [ -f "$ALT_MP3D" ] && [ -f "$TG_K5_MP3D" ]; then
    python3 "$TOOLS/analyze_amortization.py" \
        --alt "$ALT_MP3D" \
        --tg "$TG_K1_MP3D" "$TG_K5_MP3D" \
        --label "TG_K1" "TG_K5" \
        --out "$ANALYSIS/amortization_mp3d_01m.csv" \
        --dataset mp3d 2>&1 | tail -5
else
    log "  [SKIP] MP3D amortization: missing data"
fi

log "=== 4/6: Tau oracle ==="
TAU_FILES=""
for tau in 3 4 5 6; do
    f="$HM3D/100_tg_tau${tau}_k5_01m_500q.csv"
    if [ -f "$f" ]; then TAU_FILES="$TAU_FILES $f"; fi
done
if [ -n "$TAU_FILES" ]; then
    python3 "$TOOLS/analyze_tau_oracle.py" \
        --alt "$ALT_HM3D" \
        --sweep $TAU_FILES \
        --auto "$TG_K5_HM3D" \
        --out "$ANALYSIS/tau_oracle.csv" 2>&1 | tail -5
else
    log "  [SKIP] Tau oracle: no tau sweep data"
fi

log "=== 5/6: Figure data CSVs + LaTeX tables ==="
python3 "$TOOLS/make_paper_figures.py" \
    --results-dir "$RESULTS" \
    --out-dir "$FIGURES" 2>&1

log "=== 6/6: PDF figures ==="
python3 "$TOOLS/plot_paper_figures.py" \
    --data-dir "$FIGURES/data" \
    --analysis-dir "$ANALYSIS" \
    --results-dir "$RESULTS" \
    --out-dir "$FIGURES" 2>&1

log "=== Summary of outputs ==="
echo "  Analysis CSVs:"
ls -la "$ANALYSIS"/*.csv 2>/dev/null | awk '{print "    " $NF " (" $5 " bytes)"}'
echo "  Figure data:"
ls -la "$FIGURES/data"/*.csv 2>/dev/null | awk '{print "    " $NF " (" $5 " bytes)"}'
echo "  PDF figures:"
ls -la "$FIGURES"/*.pdf 2>/dev/null | awk '{print "    " $NF " (" $5 " bytes)"}'
echo "  LaTeX tables:"
ls -la "$FIGURES"/*.tex 2>/dev/null | awk '{print "    " $NF " (" $5 " bytes)"}'

log "=== DONE ==="
