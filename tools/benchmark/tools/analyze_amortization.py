#!/usr/bin/env python3
"""P5.2: Amortization curve — T_total(N) = T_precompute + N * T_query.

Computes break-even query counts comparing ALT vs TG methods.

Usage:
    python3 analyze_amortization.py \
      --alt experiments/results/hm3d_100/100_alt_01m_500q.csv \
      --tg experiments/results/hm3d_100/100_tg_auto_k1_01m_500q.csv \
           experiments/results/hm3d_100/100_tg_auto_k5_01m_500q.csv \
      --label "TG K=1" "TG K=5" \
      --out experiments/results/analysis/amortization.csv
"""
import argparse
import csv
import statistics
from collections import defaultdict
from pathlib import Path


def sf(v, default=0.0):
    try:
        return float(v)
    except (ValueError, TypeError):
        return default


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--alt', required=True)
    ap.add_argument('--tg', nargs='+', required=True)
    ap.add_argument('--label', nargs='+', required=True)
    ap.add_argument('--out', default='experiments/results/analysis/amortization.csv')
    ap.add_argument('--dataset', default='HM3D')
    args = ap.parse_args()

    assert len(args.tg) == len(args.label)

    # Load ALT
    with open(args.alt) as f:
        alt_rows = list(csv.DictReader(f))
    alt_succ = [r for r in alt_rows if r.get('success') == 'True']

    # ALT precompute = per-scene topo_ms (or precompute_time_ms)
    alt_scenes = {}
    for r in alt_rows:
        s = r.get('scene', '')
        if s not in alt_scenes:
            alt_scenes[s] = r
    alt_precompute_ms = statistics.mean([sf(v.get('topo_ms', 0)) for v in alt_scenes.values()]) if alt_scenes else 0
    alt_query_ms = statistics.mean([sf(r['planning_time_ms']) for r in alt_succ]) if alt_succ else 0

    print(f"ALT: precompute={alt_precompute_ms:.1f}ms, query={alt_query_ms:.3f}ms")

    N_values = [1, 5, 10, 50, 100, 500, 1000, 5000, 10000]
    results = []

    for n in N_values:
        row = {'N': n, 'ALT_total_ms': alt_precompute_ms + n * alt_query_ms}
        results.append(row)

    methods = [('ALT', alt_precompute_ms, alt_query_ms)]

    for tg_path, label in zip(args.tg, args.label):
        with open(tg_path) as f:
            tg_rows = list(csv.DictReader(f))
        tg_succ = [r for r in tg_rows if r.get('success') == 'True']

        tg_scenes = {}
        for r in tg_rows:
            s = r.get('scene', '')
            if s not in tg_scenes:
                tg_scenes[s] = r
        tg_precompute_ms = statistics.mean([sf(v.get('topo_ms', 0)) for v in tg_scenes.values()]) if tg_scenes else 0
        tg_query_ms = statistics.mean([sf(r['planning_time_ms']) for r in tg_succ]) if tg_succ else 0

        safe = label.replace(' ', '_').replace('=', '')
        methods.append((label, tg_precompute_ms, tg_query_ms))
        print(f"{label}: precompute={tg_precompute_ms:.1f}ms, query={tg_query_ms:.3f}ms")

        for i, n in enumerate(N_values):
            results[i][f'{safe}_total_ms'] = tg_precompute_ms + n * tg_query_ms

    # Find break-even points
    print(f"\n{'='*60}")
    print(f"  Amortization Summary ({args.dataset})")
    print(f"{'='*60}")
    for label, pre, qry in methods[1:]:
        # Break-even: pre_tg + N * qry_tg = pre_alt + N * qry_alt
        # N = (pre_tg - pre_alt) / (qry_alt - qry_tg)
        if alt_query_ms > qry:
            breakeven = (pre - alt_precompute_ms) / (alt_query_ms - qry)
            print(f"  {label} vs ALT: break-even at N={breakeven:.0f} queries")
            if breakeven <= 0:
                print(f"    → TG always faster (cheaper precompute + faster query)")
        else:
            print(f"  {label} vs ALT: TG query is SLOWER → no break-even")

    # Write CSV
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    with open(args.out, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(results[0].keys()))
        w.writeheader()
        w.writerows(results)
    print(f"\nWritten to {args.out}")

    # Table
    print(f"\n{'N':>8}", end='')
    for label, _, _ in methods:
        print(f"  {label:>12}", end='')
    print()
    for row in results:
        print(f"{row['N']:>8}", end='')
        for label, _, _ in methods:
            safe = label.replace(' ', '_').replace('=', '')
            key = 'ALT_total_ms' if label == 'ALT' else f'{safe}_total_ms'
            print(f"  {row[key]:>12.1f}", end='')
        print()


if __name__ == '__main__':
    main()
