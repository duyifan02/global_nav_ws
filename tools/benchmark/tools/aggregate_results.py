#!/usr/bin/env python3
"""Aggregate PH-ALT-TG experiment results for the paper.

Supports:
  - Main comparison tables (ALT vs TG K=1 vs TG K=5)
  - Paired overhead analysis (only queries succeeding in both methods)
  - Failure reason breakdown
  - Timing breakdown (snap, TG A*, stitch, fallback)
  - Path validity statistics
  - Cross-resolution speedup
  - LaTeX table output

Usage:
    python3 aggregate_results.py --results-dir experiments/results [--latex]
"""
import argparse
import csv
import statistics
from collections import Counter
from pathlib import Path
from typing import Dict, List, Optional

# ── MP3D canonical scene mapping ──────────────────────────────────────────────
SCENE_MAP = {
    '1LXtFkjw3qL': 'S1', '1pXnuDYAj8r': 'S2', '2azQ1b91cZZ': 'S3',
    '2n8kARJN3HM': 'S4', '2t7WUuJeko7': 'S5', '5LpN3gDmAk7': 'S6',
}


def load_csv(path: str) -> list:
    with open(path, newline='') as f:
        return list(csv.DictReader(f))


def sf(v, default=0.0):
    try:
        return float(v)
    except (ValueError, TypeError):
        return default


def si(v, default=0):
    try:
        return int(float(v))
    except (ValueError, TypeError):
        return default


def pct(vals, p):
    if not vals:
        return 0.0
    s = sorted(vals)
    idx = min(int(len(s) * p / 100), len(s) - 1)
    return s[idx]


def st(vals):
    if not vals:
        return {'mean': 0, 'med': 0, 'std': 0, 'p90': 0, 'p95': 0, 'max': 0}
    return {
        'mean': statistics.mean(vals),
        'med': statistics.median(vals),
        'std': statistics.pstdev(vals),
        'p90': pct(vals, 90),
        'p95': pct(vals, 95),
        'max': max(vals),
    }


def query_key(row: dict) -> str:
    """Unique key for a query (scene + start + goal)."""
    return (f"{row.get('scene', '')}|"
            f"{row.get('start_x', '')}_{row.get('start_y', '')}_{row.get('start_z', '')}|"
            f"{row.get('goal_x', '')}_{row.get('goal_y', '')}_{row.get('goal_z', '')}")


# ── Full-set summary ─────────────────────────────────────────────────────────

def summarise(rows: list, label: str = '') -> dict:
    total = len(rows)
    succ = [r for r in rows if r.get('success', '') == 'True']
    n_succ = len(succ)

    # Path validity
    valid = [r for r in succ if r.get('path_valid', '') == 'True']
    n_valid = len(valid)
    invalid = [r for r in succ if r.get('path_valid', '') == 'False']

    ts = [sf(r['planning_time_ms']) for r in succ]
    pl = [sf(r['path_length_m']) for r in succ if sf(r['path_length_m']) > 0]

    # Failure reasons
    fail_reasons = Counter()
    for r in rows:
        if r.get('success', '') != 'True':
            reason = r.get('failure_reason', 'unknown').strip() or 'unknown'
            fail_reasons[reason] += 1

    # Per-scene build stats (first row per scene)
    seen = {}
    for r in rows:
        s = r.get('scene', '')
        if s not in seen:
            seen[s] = r

    topo = [sf(v.get('topo_ms', 0)) for v in seen.values() if sf(v.get('topo_ms', 0)) > 0]
    pre = [sf(v.get('precompute_time_ms', 0)) for v in seen.values()]
    mem = [sf(v.get('memory_usage_bytes', 0)) / 1e6 for v in seen.values() if sf(v.get('memory_usage_bytes', 0)) > 0]
    rooms = [sf(v.get('num_rooms', 0)) for v in seen.values() if sf(v.get('num_rooms', 0)) > 0]
    tg_n = [sf(v.get('tg_nodes', 0)) for v in seen.values() if sf(v.get('tg_nodes', 0)) > 0]
    tg_e = [sf(v.get('tg_edges', 0)) for v in seen.values() if sf(v.get('tg_edges', 0)) > 0]
    sel_tau = [sf(v.get('selected_tau', 0)) for v in seen.values() if sf(v.get('selected_tau', 0)) > 0]
    k_eff = [sf(v.get('portal_k_effective', 0)) for v in seen.values() if sf(v.get('portal_k_effective', 0)) > 0]

    # Timing breakdown (from successful queries)
    snap_ms = [sf(r.get('diag_snap_ms', 0)) for r in succ]
    tg_ms = [sf(r.get('diag_tg_astar_ms', 0)) for r in succ]
    stitch_ms = [sf(r.get('diag_stitch_ms', 0)) for r in succ]
    fb_ms = [sf(r.get('diag_fallback_astar_ms', 0)) for r in succ]
    fb_nodes = [si(r.get('diag_fallback_nodes', 0)) for r in succ]

    return {
        'label': label, 'total': total, 'succ': n_succ, 'valid': n_valid,
        'rate': n_succ / total if total else 0,
        'valid_rate': n_valid / n_succ if n_succ else 0,
        'scenes': len(seen),
        'ts': st(ts), 'path': st(pl),
        'topo': st(topo), 'pre': st(pre), 'mem': st(mem), 'rooms': st(rooms),
        'tg_n': st(tg_n), 'tg_e': st(tg_e), 'sel_tau': st(sel_tau), 'k_eff': st(k_eff),
        'snap_ms': st(snap_ms), 'tg_ms': st(tg_ms), 'stitch_ms': st(stitch_ms),
        'fb_ms': st(fb_ms), 'fb_nodes': st(fb_nodes),
        'fail_reasons': fail_reasons,
    }


# ── Paired overhead analysis ─────────────────────────────────────────────────

def paired_overhead(alt_rows: list, tg_rows: list) -> dict:
    """Compute path overhead only on queries that succeed + valid in both methods."""
    alt_map = {}
    for r in alt_rows:
        if r.get('success', '') == 'True' and sf(r.get('path_length_m', 0)) > 0:
            alt_map[query_key(r)] = sf(r['path_length_m'])

    tg_map = {}
    for r in tg_rows:
        if r.get('success', '') == 'True' and sf(r.get('path_length_m', 0)) > 0:
            valid = r.get('path_valid', 'True') != 'False'  # default True for old data
            if valid:
                tg_map[query_key(r)] = sf(r['path_length_m'])

    common = set(alt_map.keys()) & set(tg_map.keys())
    if not common:
        return {'n_paired': 0, 'overhead': st([]), 'alt_path': st([]), 'tg_path': st([])}

    overheads = []
    alt_paths = []
    tg_paths = []
    for k in common:
        a, t = alt_map[k], tg_map[k]
        overheads.append((t / a - 1) * 100 if a > 0 else 0)
        alt_paths.append(a)
        tg_paths.append(t)

    return {
        'n_paired': len(common),
        'overhead': st(overheads),
        'alt_path': st(alt_paths),
        'tg_path': st(tg_paths),
    }


# ── Formatting ───────────────────────────────────────────────────────────────

def fmt_main_table(summaries: list, title: str, alt_key: str = 'ALT'):
    print(f"\n{'=' * 100}")
    print(f"  {title}")
    print(f"{'=' * 100}")
    hdr = (f"{'Method':<28} {'Succ%':>6} {'Valid%':>6} {'Ts(ms)':>8} {'Ts_p95':>8} "
           f"{'Path(m)':>8} {'OH%':>6} {'Topo(ms)':>9} {'K_eff':>6} {'τ':>4}")
    print(hdr)
    print('-' * len(hdr))

    alt_pl = None
    for s in summaries:
        if alt_key in s['label'].upper() and 'TG' not in s['label'].upper():
            alt_pl = s['path']['mean']
            break

    for s in summaries:
        oh = ''
        if alt_pl and alt_pl > 0 and s['path']['mean'] > 0:
            is_alt = alt_key in s['label'].upper() and 'TG' not in s['label'].upper()
            oh = '--' if is_alt else f"{(s['path']['mean'] / alt_pl - 1) * 100:+.1f}"
        tau_s = f"{s['sel_tau']['mean']:.1f}" if s['sel_tau']['mean'] > 0 else '--'
        k_s = f"{s['k_eff']['mean']:.1f}" if s['k_eff']['mean'] > 0 else '--'
        print(f"{s['label']:<28} {s['rate']*100:5.1f}% {s['valid_rate']*100:5.1f}% "
              f"{s['ts']['mean']:8.3f} {s['ts']['p95']:8.3f} "
              f"{s['path']['mean']:8.2f} {oh:>6} {s['topo']['mean']:9.1f} {k_s:>6} {tau_s:>4}")
    print()


def fmt_paired_table(paired_results: dict, title: str):
    print(f"\n{'=' * 90}")
    print(f"  {title} (Paired Comparison)")
    print(f"{'=' * 90}")
    hdr = f"{'Method':<28} {'Paired':>7} {'Mean OH%':>9} {'Med OH%':>8} {'P95 OH%':>8} {'Mean Path':>10}"
    print(hdr)
    print('-' * len(hdr))
    for label, p in paired_results.items():
        if p['n_paired'] == 0:
            print(f"{label:<28} {'(no paired queries)':>30}")
            continue
        print(f"{label:<28} {p['n_paired']:>7} {p['overhead']['mean']:+8.1f}% "
              f"{p['overhead']['med']:+7.1f}% {p['overhead']['p95']:+7.1f}% "
              f"{p['tg_path']['mean']:10.2f}")
    print()


def fmt_failure_table(summaries: list, title: str):
    print(f"\n{'=' * 80}")
    print(f"  {title} — Failure Reasons")
    print(f"{'=' * 80}")
    for s in summaries:
        if not s['fail_reasons']:
            continue
        print(f"  {s['label']}:")
        for reason, count in s['fail_reasons'].most_common():
            print(f"    {reason:<30} {count:>5} ({count/s['total']*100:.1f}%)")
    print()


def fmt_timing_table(summaries: list, title: str):
    print(f"\n{'=' * 90}")
    print(f"  {title} — Query Timing Breakdown (mean / p95, ms)")
    print(f"{'=' * 90}")
    hdr = f"{'Method':<28} {'Total':>8} {'Snap':>8} {'TG A*':>8} {'Stitch':>8} {'Fallback':>8} {'FB nodes':>8}"
    print(hdr)
    print('-' * len(hdr))
    for s in summaries:
        print(f"{s['label']:<28} "
              f"{s['ts']['mean']:7.3f}/{s['ts']['p95']:6.3f}  "
              f"{s['snap_ms']['mean']:5.3f}  "
              f"{s['tg_ms']['mean']:7.3f}/{s['tg_ms']['p95']:6.3f}  "
              f"{s['stitch_ms']['mean']:5.3f}/{s['stitch_ms']['p95']:6.3f}  "
              f"{s['fb_ms']['mean']:5.3f}  "
              f"{s['fb_nodes']['mean']:7.0f}")
    print()


def fmt_latex_main(summaries: list, title: str, paired: dict = None):
    alt_pl = None
    for s in summaries:
        if 'ALT' in s['label'].upper() and 'TG' not in s['label'].upper():
            alt_pl = s['path']['mean']
            break

    print(f"\n% {title}")
    print(r"\begin{tabular}{l r r r r r r r r}")
    print(r"\toprule")
    print(r"Method & Succ\% & Valid\% & $\bar{T}_s$ & $T_s^{95}$ & $\bar{L}$ & OH\% & $T_{topo}$ & $K_{eff}$ \\")
    print(r"\midrule")
    for s in summaries:
        oh = '--'
        if alt_pl and alt_pl > 0 and s['path']['mean'] > 0:
            is_alt = 'ALT' in s['label'].upper() and 'TG' not in s['label'].upper()
            if not is_alt:
                # Use paired overhead if available
                if paired and s['label'] in paired and paired[s['label']]['n_paired'] > 0:
                    oh = f"{paired[s['label']]['overhead']['mean']:+.1f}\\%"
                else:
                    oh = f"{(s['path']['mean'] / alt_pl - 1) * 100:+.1f}\\%"
        k_s = f"{s['k_eff']['mean']:.1f}" if s['k_eff']['mean'] > 0 else '--'
        print(f"  {s['label']} & {s['rate']*100:.1f}\\% & {s['valid_rate']*100:.0f}\\% & "
              f"{s['ts']['mean']:.3f} & {s['ts']['p95']:.3f} & "
              f"{s['path']['mean']:.2f} & {oh} & {s['topo']['mean']:.1f} & {k_s} \\\\")
    print(r"\bottomrule")
    print(r"\end{tabular}")
    print()


# ── CSV output ───────────────────────────────────────────────────────────────

def write_summary_csv(summaries: list, paired: dict, path: str):
    fields = ['label', 'total', 'succ', 'valid', 'rate', 'valid_rate',
              'ts_mean', 'ts_med', 'ts_p95', 'path_mean', 'path_med', 'path_p95',
              'oh_mean', 'oh_med', 'oh_p95', 'n_paired',
              'topo_mean', 'mem_mean', 'k_eff_mean', 'sel_tau_mean']
    rows = []
    for s in summaries:
        p = paired.get(s['label'], {})
        rows.append({
            'label': s['label'], 'total': s['total'], 'succ': s['succ'],
            'valid': s['valid'], 'rate': f"{s['rate']:.4f}", 'valid_rate': f"{s['valid_rate']:.4f}",
            'ts_mean': f"{s['ts']['mean']:.4f}", 'ts_med': f"{s['ts']['med']:.4f}",
            'ts_p95': f"{s['ts']['p95']:.4f}",
            'path_mean': f"{s['path']['mean']:.4f}", 'path_med': f"{s['path']['med']:.4f}",
            'path_p95': f"{s['path']['p95']:.4f}",
            'oh_mean': f"{p.get('overhead', {}).get('mean', '')}" if p.get('n_paired') else '',
            'oh_med': f"{p.get('overhead', {}).get('med', '')}" if p.get('n_paired') else '',
            'oh_p95': f"{p.get('overhead', {}).get('p95', '')}" if p.get('n_paired') else '',
            'n_paired': p.get('n_paired', ''),
            'topo_mean': f"{s['topo']['mean']:.2f}", 'mem_mean': f"{s['mem']['mean']:.2f}",
            'k_eff_mean': f"{s['k_eff']['mean']:.2f}" if s['k_eff']['mean'] > 0 else '',
            'sel_tau_mean': f"{s['sel_tau']['mean']:.2f}" if s['sel_tau']['mean'] > 0 else '',
        })
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)
    print(f"  [CSV] Written: {path}")


# ── Main ─────────────────────────────────────────────────────────────────────

def process_group(files: dict, title: str, alt_label: str, latex: bool,
                  out_dir: Optional[Path] = None):
    """Process a group of result files: full-set + paired + failures + timing."""
    avail = {k: v for k, v in files.items() if v.exists()}
    if len(avail) < 2:
        miss = [k for k, f in files.items() if not f.exists()]
        print(f"[SKIP] {title}: missing {', '.join(miss)}")
        return

    # Load data and summarise
    data = {k: load_csv(str(v)) for k, v in avail.items()}
    sums = [summarise(data[k], k) for k in avail]
    fmt_main_table(sums, title)

    # Paired overhead
    alt_data = data.get(alt_label)
    paired = {}
    if alt_data:
        for label, rows in data.items():
            if label != alt_label:
                paired[label] = paired_overhead(alt_data, rows)
        fmt_paired_table(paired, title)

    # Failure reasons
    fmt_failure_table(sums, title)

    # Timing breakdown
    fmt_timing_table(sums, title)

    # LaTeX
    if latex:
        fmt_latex_main(sums, title, paired)

    # CSV output
    if out_dir:
        safe_title = title.lower().replace(' ', '_').replace('(', '').replace(')', '').replace(',', '')
        write_summary_csv(sums, paired, str(out_dir / f'summary_{safe_title}.csv'))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--results-dir', default='experiments/results')
    ap.add_argument('--latex', action='store_true')
    ap.add_argument('--csv-out', default=None, help='Directory for summary CSVs')
    args = ap.parse_args()

    rd = Path(args.results_dir)
    hd = rd / 'hm3d_100'
    md = rd / 'mp3d_6'
    out = Path(args.csv_out) if args.csv_out else rd / 'summaries'
    out.mkdir(parents=True, exist_ok=True)

    # ── 1. HM3D K-sweep (100 scenes, 0.1m) ──────────────────────────────────
    process_group({
        'ALT (L=16)': hd / '100_alt_01m_500q.csv',
        'TG auto K=1': hd / '100_tg_auto_k1_01m_500q.csv',
        'TG auto K=5': hd / '100_tg_auto_k5_01m_500q.csv',
    }, 'HM3D 100 scenes, 0.1m', 'ALT (L=16)', args.latex, out)

    # ── 2. MP3D (6 scenes, 0.1m) ────────────────────────────────────────────
    process_group({
        'ALT (L=16)': md / 'alt_01m.csv',
        'TG auto K=1': md / 'tg_auto_k1_01m.csv',
        'TG auto K=5': md / 'tg_auto_k5_01m.csv',
    }, 'MP3D 6 scenes, 0.1m', 'ALT (L=16)', args.latex, out)

    # ── 3. HM3D cross-resolution K=5 ────────────────────────────────────────
    for res, rl in [('01m', '0.1m'), ('0075m', '0.075m'), ('005m', '0.05m')]:
        suffix = '_500q' if res == '01m' else ''
        process_group({
            'ALT (L=16)': hd / f'100_alt_{res}{suffix}.csv',
            'TG auto K=5': hd / f'100_tg_auto_k5_{res}{suffix}.csv',
        }, f'HM3D cross-res {rl}', 'ALT (L=16)', args.latex, out)

    # ── 4. Cross-resolution speedup summary ──────────────────────────────────
    print(f"\n{'=' * 80}")
    print("  Cross-Resolution Speedup: TG auto K=5 vs ALT")
    print(f"{'=' * 80}")
    hdr = f"{'Res':<8} {'ALT Ts':>10} {'TG Ts':>10} {'Speedup':>9} {'OH% (paired)':>14} {'N_paired':>9} {'ALT succ':>9} {'TG succ':>8}"
    print(hdr)
    print('-' * len(hdr))

    for res, rl in [('01m', '0.1m'), ('0075m', '0.075m'), ('005m', '0.05m')]:
        suffix = '_500q' if res == '01m' else ''
        af = hd / f'100_alt_{res}{suffix}.csv'
        tf = hd / f'100_tg_auto_k5_{res}{suffix}.csv'
        if not af.exists() or not tf.exists():
            print(f"{rl:<8} {'(pending)':>10}")
            continue
        a_rows = load_csv(str(af))
        t_rows = load_csv(str(tf))
        a_succ = [r for r in a_rows if r.get('success') == 'True']
        t_succ = [r for r in t_rows if r.get('success') == 'True']
        at = statistics.mean([sf(r['planning_time_ms']) for r in a_succ]) if a_succ else 0
        tt = statistics.mean([sf(r['planning_time_ms']) for r in t_succ]) if t_succ else 0
        sp = at / tt if tt > 0 else 0
        p = paired_overhead(a_rows, t_rows)
        oh_str = f"{p['overhead']['mean']:+.1f}%" if p['n_paired'] > 0 else '--'
        print(f"{rl:<8} {at:10.3f} {tt:10.3f} {sp:8.1f}x {oh_str:>14} {p['n_paired']:>9} "
              f"{len(a_succ):>9} {len(t_succ):>8}")
    print()

    # ── 5. HM3D ablation (0.1m, all baselines) ──────────────────────────────
    abl = {
        'JPS_3D': hd / '100_jps_01m.csv',
        'Flat A*': hd / '100_flat_01m.csv',
        'ALT (L=16)': hd / '100_alt_01m_500q.csv',
        'PRM (1k)': hd / '100_prm_1k_01m.csv',
        'Grid-HPA*': hd / '100_grid_tau5_01m.csv',
        'No-Stitch': hd / '100_no_stitch_tau5_01m.csv',
        'TG auto K=1': hd / '100_tg_auto_k1_01m_500q.csv',
        'TG auto K=5': hd / '100_tg_auto_k5_01m_500q.csv',
    }
    avail_abl = {k: v for k, v in abl.items() if v.exists()}
    if len(avail_abl) >= 3:
        data_abl = {k: load_csv(str(v)) for k, v in avail_abl.items()}
        sums_abl = [summarise(data_abl[k], k) for k in avail_abl]
        fmt_main_table(sums_abl, 'HM3D Ablation (100 scenes, 0.1m)')
        if args.latex:
            alt_data = data_abl.get('ALT (L=16)')
            paired_abl = {}
            if alt_data:
                for label, rows in data_abl.items():
                    if label != 'ALT (L=16)':
                        paired_abl[label] = paired_overhead(alt_data, rows)
            fmt_latex_main(sums_abl, 'HM3D Ablation (100 scenes, 0.1m)', paired_abl)


if __name__ == '__main__':
    main()
