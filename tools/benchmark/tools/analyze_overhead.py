#!/usr/bin/env python3
"""P2.2 + P3.1: Per-query overhead analysis, CDF data export, and bad-case identification.

Reads ALT baseline + TG result CSVs, computes paired overhead per query,
and outputs:
  1. Per-query overhead CSV (for CDF/boxplot figures)
  2. Top bad-case queries by overhead
  3. Summary statistics

Usage:
    python3 analyze_overhead.py \
      --alt experiments/results/hm3d_100/100_alt_01m_500q.csv \
      --tg experiments/results/hm3d_100/100_tg_auto_k5_01m_500q.csv \
      --label "TG K=5" \
      --out-dir experiments/results/analysis

    # Multi-method comparison:
    python3 analyze_overhead.py \
      --alt experiments/results/hm3d_100/100_alt_01m_500q.csv \
      --tg experiments/results/hm3d_100/100_tg_auto_k1_01m_500q.csv \
           experiments/results/hm3d_100/100_tg_auto_k5_01m_500q.csv \
      --label "TG K=1" "TG K=5" \
      --out-dir experiments/results/analysis
"""
import argparse
import csv
import statistics
from pathlib import Path


def sf(v, default=0.0):
    try:
        return float(v)
    except (ValueError, TypeError):
        return default


def pct(vals, p):
    if not vals:
        return 0.0
    s = sorted(vals)
    return s[min(int(len(s) * p / 100), len(s) - 1)]


def query_key(row: dict) -> str:
    return (f"{row.get('scene', '')}|"
            f"{row.get('start_x', '')}_{row.get('start_y', '')}_{row.get('start_z', '')}|"
            f"{row.get('goal_x', '')}_{row.get('goal_y', '')}_{row.get('goal_z', '')}")


def load_method(path: str) -> dict:
    """Load CSV and index by query key."""
    rows = []
    with open(path, newline='') as f:
        rows = list(csv.DictReader(f))
    by_key = {}
    for r in rows:
        by_key[query_key(r)] = r
    return by_key


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--alt', required=True, help='ALT baseline CSV')
    ap.add_argument('--tg', nargs='+', required=True, help='TG result CSV(s)')
    ap.add_argument('--label', nargs='+', required=True, help='Labels for each TG CSV')
    ap.add_argument('--out-dir', default='experiments/results/analysis')
    ap.add_argument('--top-n', type=int, default=20, help='Number of top bad cases')
    args = ap.parse_args()

    assert len(args.tg) == len(args.label), "--tg and --label must have same count"
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    # Load ALT baseline
    alt = load_method(args.alt)
    print(f"ALT: {len(alt)} queries from {args.alt}")

    # Process each TG method
    all_overhead_rows = []

    for tg_path, label in zip(args.tg, args.label):
        tg = load_method(tg_path)
        print(f"\n{'='*70}")
        print(f"  {label}: {len(tg)} queries from {tg_path}")
        print(f"{'='*70}")

        # Find paired queries (both succeed, both have valid paths)
        paired = []
        for key in alt:
            if key not in tg:
                continue
            a, t = alt[key], tg[key]
            a_ok = a.get('success', '') == 'True' and sf(a.get('path_length_m', 0)) > 0
            t_ok = t.get('success', '') == 'True' and sf(t.get('path_length_m', 0)) > 0
            t_valid = t.get('path_valid', 'True') != 'False'
            if a_ok and t_ok and t_valid:
                alt_path = sf(a['path_length_m'])
                tg_path_m = sf(t['path_length_m'])
                overhead = (tg_path_m / alt_path - 1) * 100 if alt_path > 0 else 0
                paired.append({
                    'scene': a.get('scene', ''),
                    'query_key': key,
                    'start_x': a.get('start_x', ''),
                    'start_y': a.get('start_y', ''),
                    'start_z': a.get('start_z', ''),
                    'goal_x': a.get('goal_x', ''),
                    'goal_y': a.get('goal_y', ''),
                    'goal_z': a.get('goal_z', ''),
                    'method': label,
                    'alt_path_m': alt_path,
                    'tg_path_m': tg_path_m,
                    'overhead_pct': overhead,
                    'alt_ts_ms': sf(a.get('planning_time_ms', 0)),
                    'tg_ts_ms': sf(t.get('planning_time_ms', 0)),
                    'tg_tg_astar_ms': sf(t.get('diag_tg_astar_ms', 0)),
                    'tg_stitch_ms': sf(t.get('diag_stitch_ms', 0)),
                    'tg_fallback_nodes': sf(t.get('diag_fallback_nodes', 0)),
                    'tg_room_hops': sf(t.get('diag_room_hops', 0)),
                    'tg_same_room': t.get('diag_same_room', ''),
                    'selected_tau': t.get('selected_tau', ''),
                    'tg_nodes': t.get('tg_nodes', ''),
                    'num_portals': t.get('num_portals', ''),
                    'k_effective': t.get('portal_k_effective', ''),
                    'path_valid': t.get('path_valid', ''),
                })

        if not paired:
            print("  No paired queries found!")
            continue

        overheads = [p['overhead_pct'] for p in paired]
        print(f"  Paired queries:  {len(paired)}")
        print(f"  Mean overhead:   {statistics.mean(overheads):+.2f}%")
        print(f"  Median overhead: {statistics.median(overheads):+.2f}%")
        print(f"  P90 overhead:    {pct(overheads, 90):+.2f}%")
        print(f"  P95 overhead:    {pct(overheads, 95):+.2f}%")
        print(f"  Max overhead:    {max(overheads):+.2f}%")
        print(f"  Min overhead:    {min(overheads):+.2f}%")
        print(f"  Queries with OH < 0: {sum(1 for o in overheads if o < 0)} "
              f"({sum(1 for o in overheads if o < 0)/len(overheads)*100:.1f}%)")

        all_overhead_rows.extend(paired)

        # Top bad cases
        sorted_by_oh = sorted(paired, key=lambda x: x['overhead_pct'], reverse=True)
        top_bad = sorted_by_oh[:args.top_n]

        safe_label = label.lower().replace(' ', '_').replace('=', '')
        bad_path = out_dir / f'top_bad_overhead_{safe_label}.csv'
        with open(bad_path, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(top_bad[0].keys()))
            w.writeheader()
            w.writerows(top_bad)
        print(f"\n  Top {args.top_n} bad cases → {bad_path}")
        print(f"  {'Scene':<30} {'OH%':>8} {'ALT(m)':>8} {'TG(m)':>8} {'Hops':>5} {'FB_N':>6}")
        print(f"  {'-'*75}")
        for b in top_bad[:10]:
            print(f"  {b['scene']:<30} {b['overhead_pct']:+7.1f}% "
                  f"{b['alt_path_m']:8.2f} {b['tg_path_m']:8.2f} "
                  f"{b['tg_room_hops']:5.0f} {b['tg_fallback_nodes']:6.0f}")

    # Write combined overhead CSV (for CDF/boxplot)
    if all_overhead_rows:
        oh_path = out_dir / 'per_query_overhead.csv'
        with open(oh_path, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(all_overhead_rows[0].keys()))
            w.writeheader()
            w.writerows(all_overhead_rows)
        print(f"\n[OUTPUT] Per-query overhead data → {oh_path} ({len(all_overhead_rows)} rows)")

    # Failure analysis
    print(f"\n{'='*70}")
    print(f"  Failure Analysis")
    print(f"{'='*70}")
    for tg_path, label in zip(args.tg, args.label):
        tg = load_method(tg_path)
        failed = [r for r in tg.values() if r.get('success', '') != 'True']
        if not failed:
            print(f"  {label}: 0 failures")
            continue
        reasons = {}
        for r in failed:
            reason = r.get('failure_reason', 'unknown').strip() or 'unknown'
            reasons[reason] = reasons.get(reason, 0) + 1
        print(f"  {label}: {len(failed)} failures")
        for reason, count in sorted(reasons.items(), key=lambda x: -x[1]):
            print(f"    {reason:<30} {count:>5}")


if __name__ == '__main__':
    main()
