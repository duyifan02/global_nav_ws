#!/bin/bash
# Sweep persistence_tau for TopoNav at one or more resolutions.
# Does NOT modify benchmark.csv. Results go to stdout + /tmp/sweep_results.tsv
#
# Usage:
#   bash sweep_tau.sh [--res 0.1,0.2] [--taus 1,2,3,5,8,12] [--trials N]
#
# Prereq: single topo_nav_node running

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
PY="${SCRIPT_DIR}/auto_benchmark.py"
MAKE_CSV="${SCRIPT_DIR}/make_sweep_csv.py"

RESOLUTIONS="0.1,0.2"
TAUS="1.0,2.0,3.0,5.0,8.0,12.0"
TRIALS=1
OUT_TSV="/tmp/sweep_results.tsv"

while [[ $# -gt 0 ]]; do
    case $1 in
        --res)    RESOLUTIONS="$2"; shift 2 ;;
        --taus)   TAUS="$2";        shift 2 ;;
        --trials) TRIALS="$2";      shift 2 ;;
        *) shift ;;
    esac
done

source "${WS_ROOT}/install/setup.bash" 2>/dev/null || \
  source /opt/ros/jazzy/setup.bash 2>/dev/null || true

# Build sweep CSV
SWEEP_CSV="${WS_ROOT}/benchmark_sweep.csv"
python3 "${MAKE_CSV}" "${WS_ROOT}/benchmark.csv" "${SWEEP_CSV}" "${RESOLUTIONS}"
echo ""

echo "================================================================"
echo "  TopoNav τ × resolution sweep  (trials=${TRIALS})"
echo "  resolutions: ${RESOLUTIONS}    taus: ${TAUS}"
echo "================================================================"
printf "%-6s %-6s %-8s %-8s %-12s %-10s %-8s\n" \
       "res" "tau" "Ts(ms)" "σTs" "path(m)" "nodes" "ok/tot"
printf "%-6s %-6s %-8s %-8s %-12s %-10s %-8s\n" \
       "---" "---" "------" "---" "-------" "-----" "------"

echo -e "res\ttau\tTs_mean\tTs_std\tpath_mean\tnodes_mean\tok\ttot" > "${OUT_TSV}"

IFS=',' read -ra RES_LIST <<< "${RESOLUTIONS}"
IFS=',' read -ra TAU_LIST <<< "${TAUS}"

for RES in "${RES_LIST[@]}"; do
    # Build a temp CSV with only this resolution's rows
    RES_CSV=$(mktemp /tmp/sweep_res_XXXXXX.csv)
    python3 - <<PYEOF "${SWEEP_CSV}" "${RES_CSV}" "${RES}"
import csv, sys
src, dst, res = sys.argv[1], sys.argv[2], sys.argv[3]
with open(src) as f:
    rows = list(csv.DictReader(f))
fieldnames = list(rows[0].keys())
filtered = [r for r in rows if abs(float(r['resolution']) - float(res)) < 1e-6]
with open(dst, 'w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=fieldnames)
    w.writeheader()
    w.writerows(filtered)
PYEOF

    for TAU in "${TAU_LIST[@]}"; do
        TMP=$(mktemp /tmp/sweep_run_XXXXXX.log)
        RES_OUT=$(mktemp /tmp/sweep_out_XXXXXX.csv)

        python3 "${PY}" \
            --csv "${RES_CSV}" \
            --output "${RES_OUT}" \
            --planner TopoNav \
            --trials "${TRIALS}" \
            --force \
            --tau "${TAU}" \
            > "${TMP}" 2>&1 || true

        # Parse console output
        TS_VALS=$(grep -oP '(?<=Ts=)[0-9.]+' "${TMP}" | paste -sd ' ')
        PATH_VALS=$(grep -oP '(?<=\| )[0-9.]+(?=m)' "${TMP}" | paste -sd ' ')
        SUCCESS=$(grep -c '✓' "${TMP}" 2>/dev/null || echo 0)
        TOTAL=$(grep -cE '✓|✗' "${TMP}" 2>/dev/null || echo 0)
        NODES_VALS=$(grep -oP "'ns_visited_nodes': \K[0-9]+" "${RES_OUT}" 2>/dev/null | paste -sd ' ')

        read TS_M TS_S PL_M NO_M < <(python3 - <<PYEOF
import math
def stats(vals):
    if not vals: return 0.0, 0.0
    m = sum(vals)/len(vals)
    s = math.sqrt(sum((x-m)**2 for x in vals)/len(vals)) if len(vals)>1 else 0.0
    return m, s

ts = [float(x) for x in "${TS_VALS}".split() if x]
pl = [float(x) for x in "${PATH_VALS}".split() if x]
no = [float(x) for x in "${NODES_VALS}".split() if x]
ts_m, ts_s = stats(ts)
pl_m, _ = stats(pl)
no_m, _ = stats(no)
print(f"{ts_m:.4f} {ts_s:.4f} {pl_m:.2f} {no_m:.0f}")
PYEOF
)

        printf "%-6s %-6s %-8s %-8s %-12s %-10s %-8s\n" \
               "${RES}" "${TAU}" "${TS_M}" "${TS_S}" "${PL_M}" "${NO_M}" "${SUCCESS}/${TOTAL}"
        echo -e "${RES}\t${TAU}\t${TS_M}\t${TS_S}\t${PL_M}\t${NO_M}\t${SUCCESS}\t${TOTAL}" >> "${OUT_TSV}"

        rm -f "${TMP}" "${RES_OUT}"
    done
    rm -f "${RES_CSV}"
    echo "---"
done

echo "================================================================"
echo "Results saved to: ${OUT_TSV}"
