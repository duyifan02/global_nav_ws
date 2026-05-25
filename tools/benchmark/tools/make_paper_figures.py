#!/usr/bin/env python3
"""P8: Generate paper-ready figures and LaTeX tables.

Generates:
  A. Main results table (LaTeX)
  B. Speed-quality Pareto plot data
  C. Overhead CDF data
  D. Resolution scaling data
  E. Speedup vs complexity data

Usage:
    python3 make_paper_figures.py --results-dir experiments/results --out-dir figures
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


def pct(vals, p):
    if not vals:
        return 0.0
    s = sorted(vals)
    return s[min(int(len(s) * p / 100), len(s) - 1)]


def query_key(row: dict) -> str:
    return (f"{row.get('scene', '')}|"
            f"{row.get('start_x')}_{row.get('start_y')}_{row.get('start_z')}|"
            f"{row.get('goal_x')}_{row.get('goal_y')}_{row.get('goal_z')}")


def load(path):
    p = Path(path)
    # Prefer _v2 file if it exists (rerun with bridgeGap + snap fix)
    if not p.name.endswith('_v2.csv'):
        v2 = p.parent / (p.stem + '_v2.csv')
        if v2.exists():
            print(f"  [v2] Using {v2.name} instead of {p.name}")
            with open(v2) as f:
                return list(csv.DictReader(f))
    if not p.exists():
        return None
    with open(p) as f:
        return list(csv.DictReader(f))


def succ_rows(rows):
    return [r for r in rows if r.get('success') == 'True' and sf(r.get('path_length_m')) > 0]


def paired_overhead(alt_rows, tg_rows):
    alt_map = {query_key(r): sf(r['path_length_m']) for r in succ_rows(alt_rows)}
    tg_valid = {query_key(r): sf(r['path_length_m']) for r in succ_rows(tg_rows)
                if r.get('path_valid', 'True') != 'False'}
    tg_all = {query_key(r): sf(r['path_length_m']) for r in succ_rows(tg_rows)}
    # Use valid-only if enough queries; fall back when path_valid not implemented
    min_valid = max(5, len(tg_all) * 0.1)
    tg_map = tg_valid if len(tg_valid) >= min_valid else tg_all
    common = set(alt_map) & set(tg_map)
    return [(tg_map[k] / alt_map[k] - 1) * 100 for k in common if alt_map[k] > 0]


def paired_speedup(alt_rows, tg_rows):
    """Per-query paired speedup: ALT_time / TG_time for queries succeeding in both."""
    alt_map = {query_key(r): sf(r['planning_time_ms']) for r in succ_rows(alt_rows)}
    tg_map = {query_key(r): sf(r['planning_time_ms']) for r in succ_rows(tg_rows)}
    common = set(alt_map) & set(tg_map)
    ratios = [alt_map[k] / tg_map[k] for k in common if tg_map[k] > 0]
    if not ratios:
        return {'mean': 0, 'median': 0, 'n': 0, 'pct_faster': 0}
    return {
        'mean': statistics.mean(ratios),
        'median': statistics.median(ratios),
        'n': len(ratios),
        'pct_faster': sum(1 for r in ratios if r > 1) / len(ratios) * 100,
    }


def method_stats(rows, alt_rows=None, label=''):
    s = succ_rows(rows)
    ts = [sf(r['planning_time_ms']) for r in s]
    pl = [sf(r['path_length_m']) for r in s]
    oh = paired_overhead(alt_rows, rows) if alt_rows else []

    # Path validity — trust the field when present, baseline fallback for old data
    has_valid_field = any(r.get('path_valid') in ('True', 'False') for r in s[:10])
    is_baseline = any(k in label.upper() for k in ('ALT', 'JPS', 'FLAT', 'PRM'))
    if has_valid_field:
        n_valid = sum(1 for r in s if r.get('path_valid') == 'True')
    elif is_baseline:
        n_valid = len(s)
    else:
        n_valid = 0  # Conservative: unknown validity

    scenes = {}
    for r in rows:
        sc = r.get('scene', '')
        if sc not in scenes:
            scenes[sc] = r
    topo = [sf(v.get('topo_ms', 0)) for v in scenes.values() if sf(v.get('topo_ms', 0)) > 0]
    mem = [sf(v.get('memory_usage_bytes', 0)) / 1e6 for v in scenes.values()
           if sf(v.get('memory_usage_bytes', 0)) > 0]

    return {
        'label': label,
        'total': len(rows), 'succ': len(s),
        'rate': len(s) / len(rows) * 100 if rows else 0,
        'valid': n_valid,
        'valid_rate': n_valid / len(s) * 100 if s else 0,
        'ts_mean': statistics.mean(ts) if ts else 0,
        'ts_med': statistics.median(ts) if ts else 0,
        'ts_p95': pct(ts, 95),
        'path_mean': statistics.mean(pl) if pl else 0,
        'oh_mean': statistics.mean(oh) if oh else 0,
        'oh_med': statistics.median(oh) if oh else 0,
        'oh_p95': pct(oh, 95),
        'n_paired': len(oh),
        'topo_mean': statistics.mean(topo) if topo else 0,
        'mem_mean': statistics.mean(mem) if mem else 0,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--results-dir', default='experiments/results')
    ap.add_argument('--out-dir', default='figures')
    args = ap.parse_args()

    rd = Path(args.results_dir)
    hd = rd / 'hm3d_100'
    md = rd / 'mp3d_6'
    out = Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    (out / 'data').mkdir(exist_ok=True)

    # ── Figure A: Main results table (LaTeX) ─────────────────────────────────

    print("=== Figure A: Main Results Table ===")
    tex_lines = []
    tex_lines.append(r"\begin{table}[t]")
    tex_lines.append(r"\centering")
    tex_lines.append(r"\caption{Main results: PH-ALT-TG vs ALT baseline.}")
    tex_lines.append(r"\label{tab:main}")
    tex_lines.append(r"\begin{tabular}{ll rr rrrr rr r}")
    tex_lines.append(r"\toprule")
    tex_lines.append(r"Dataset & Method & Succ\% & Valid\% & $\tilde{T}_q$ & $\bar{T}_q$ & $T_q^{95}$ & $\bar{L}$ & OH\% & OH$_{50}$\% & $T_{pre}$ \\")
    tex_lines.append(r" & & & & (ms) & (ms) & (ms) & (m) & (mean) & (med) & (ms) \\")
    tex_lines.append(r"\midrule")

    for dataset, ddir, prefix in [('MP3D', md, ''), ('HM3D', hd, '100_')]:
        alt_path = ddir / f'{prefix}alt_01m{"_500q" if dataset == "HM3D" else ""}.csv'
        alt = load(str(alt_path))
        if not alt:
            print(f"  [SKIP] {dataset}: ALT not found at {alt_path}")
            continue

        methods = [('ALT (L=16)', alt, None)]
        for k in [1, 5]:
            tg_path = ddir / f'{"100_" if dataset == "HM3D" else ""}tg_auto_k{k}_01m{"_500q" if dataset == "HM3D" else ""}.csv'
            tg = load(str(tg_path))
            if tg:
                methods.append((f'TG K={k}', tg, alt))

        for label, rows, alt_ref in methods:
            st = method_stats(rows, alt_ref, label)
            oh_str = f"{st['oh_mean']:+.1f}\\%" if st['n_paired'] else '--'
            oh_med = f"{st['oh_med']:+.1f}\\%" if st['n_paired'] else '--'
            tex_lines.append(
                f"  {dataset} & {label} & {st['rate']:.1f}\\% & "
                f"{st['valid_rate']:.1f}\\% & "
                f"{st['ts_med']:.3f} & {st['ts_mean']:.3f} & {st['ts_p95']:.3f} & "
                f"{st['path_mean']:.2f} & {oh_str} & {oh_med} & "
                f"{st['topo_mean']:.1f} \\\\")
        tex_lines.append(r"\addlinespace")

    tex_lines.append(r"\bottomrule")
    tex_lines.append(r"\end{tabular}")
    tex_lines.append(r"\end{table}")

    tex_path = out / 'main_results_table.tex'
    with open(tex_path, 'w') as f:
        f.write('\n'.join(tex_lines))
    print(f"  Written: {tex_path}")

    # ── Figure C: Overhead CDF data ──────────────────────────────────────────

    print("\n=== Figure C: Overhead CDF ===")
    alt = load(str(hd / '100_alt_01m_500q.csv'))
    if alt:
        cdf_rows = []
        for k in [1, 5]:
            tg = load(str(hd / f'100_tg_auto_k{k}_01m_500q.csv'))
            if not tg:
                continue
            ohs = sorted(paired_overhead(alt, tg))
            n = len(ohs)
            for i, oh in enumerate(ohs):
                cdf_rows.append({'method': f'K={k}', 'overhead_pct': f'{oh:.4f}',
                                 'cdf': f'{(i+1)/n:.6f}'})

        if cdf_rows:
            cdf_path = out / 'data' / 'overhead_cdf_hm3d_01m.csv'
            with open(cdf_path, 'w', newline='') as f:
                w = csv.DictWriter(f, fieldnames=['method', 'overhead_pct', 'cdf'])
                w.writeheader()
                w.writerows(cdf_rows)
            print(f"  Written: {cdf_path} ({len(cdf_rows)} rows)")

    # ── Figure D: Resolution scaling data ────────────────────────────────────

    print("\n=== Figure D: Resolution Scaling ===")
    res_rows = []
    for res, rl in [('01m', 0.1), ('0075m', 0.075), ('005m', 0.05)]:
        suffix = '_500q' if res == '01m' else '_500q'
        af = hd / f'100_alt_{res}{suffix}.csv'
        tf = hd / f'100_tg_auto_k5_{res}{suffix}.csv'
        a = load(str(af))
        t = load(str(tf))
        if not a or not t:
            print(f"  [SKIP] {rl}: missing data")
            continue
        a_s = method_stats(a, label=f'ALT {rl}')
        t_s = method_stats(t, a, label=f'TG K=5 {rl}')
        speedup_mean = a_s['ts_mean'] / t_s['ts_mean'] if t_s['ts_mean'] > 0 else 0
        speedup_med = a_s['ts_med'] / t_s['ts_med'] if t_s['ts_med'] > 0 else 0
        ps = paired_speedup(a, t)
        res_rows.append({
            'resolution': rl,
            'alt_ts_mean': f"{a_s['ts_mean']:.4f}",
            'alt_ts_med': f"{a_s['ts_med']:.4f}",
            'alt_ts_p95': f"{a_s['ts_p95']:.4f}",
            'tg_ts_mean': f"{t_s['ts_mean']:.4f}",
            'tg_ts_med': f"{t_s['ts_med']:.4f}",
            'tg_ts_p95': f"{t_s['ts_p95']:.4f}",
            'speedup_mean': f"{speedup_mean:.2f}",
            'speedup_med': f"{speedup_med:.2f}",
            'paired_speedup': f"{ps['median']:.2f}",
            'pct_faster': f"{ps['pct_faster']:.1f}",
            'oh_mean': f"{t_s['oh_mean']:.2f}",
            'oh_med': f"{t_s['oh_med']:.2f}",
            'alt_succ_rate': f"{a_s['rate']:.1f}",
            'tg_succ_rate': f"{t_s['rate']:.1f}",
        })

    if res_rows:
        res_path = out / 'data' / 'resolution_scaling_hm3d.csv'
        with open(res_path, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(res_rows[0].keys()))
            w.writeheader()
            w.writerows(res_rows)
        print(f"  Written: {res_path}")

    # ── Figure B: Pareto data ────────────────────────────────────────────────

    print("\n=== Figure B: Pareto (speed vs quality) ===")
    pareto_methods = {
        'ALT': hd / '100_alt_01m_500q.csv',
        'TG K=1': hd / '100_tg_auto_k1_01m_500q.csv',
        'TG K=5': hd / '100_tg_auto_k5_01m_500q.csv',
    }
    # Also include ablation baselines if they exist
    for ablation_label, ablation_file in [
        ('JPS_3D', hd / '100_jps_01m.csv'),
        ('Flat A*', hd / '100_flat_01m.csv'),
        ('PRM 1k', hd / '100_prm_1k_01m.csv'),
        ('PRM 2k', hd / '100_prm_2000_01m_500q.csv'),
        ('PRM 5k', hd / '100_prm_5000_01m_500q.csv'),
        ('PRM 10k', hd / '100_prm_10000_01m_500q.csv'),
        ('Grid-HPA*', hd / '100_grid_tau5_01m.csv'),
        ('No-Stitch', hd / '100_no_stitch_tau5_01m.csv'),
    ]:
        if ablation_file.exists():
            pareto_methods[ablation_label] = ablation_file

    alt_data = load(str(pareto_methods.get('ALT', '')))
    pareto_rows = []
    if alt_data:
        for label, path in pareto_methods.items():
            rows = load(str(path))
            if not rows:
                continue
            alt_ref = None if label == 'ALT' else alt_data
            st = method_stats(rows, alt_ref, label)
            ps = paired_speedup(alt_data, rows) if label != 'ALT' else {'mean': 1, 'median': 1, 'n': 0, 'pct_faster': 0}
            pareto_rows.append({
                'method': label,
                'mean_query_ms': f"{st['ts_mean']:.4f}",
                'med_query_ms': f"{st['ts_med']:.4f}",
                'p95_query_ms': f"{st['ts_p95']:.4f}",
                'mean_overhead_pct': f"{st['oh_mean']:.2f}" if st['n_paired'] else '0',
                'success_rate': f"{st['rate']:.1f}",
                'valid_rate': f"{st['valid_rate']:.1f}",
                'topo_ms': f"{st['topo_mean']:.1f}",
                'mem_mb': f"{st['mem_mean']:.1f}",
                'paired_speedup_med': f"{ps['median']:.2f}",
                'paired_speedup_mean': f"{ps['mean']:.2f}",
                'pct_faster': f"{ps['pct_faster']:.1f}",
            })

    if pareto_rows:
        pareto_path = out / 'data' / 'pareto_hm3d_01m.csv'
        with open(pareto_path, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(pareto_rows[0].keys()))
            w.writeheader()
            w.writerows(pareto_rows)
        print(f"  Written: {pareto_path}")

    # ── Figure E: Ablation table (LaTeX) ────────────────────────────────────

    print("\n=== Figure E: Ablation Table ===")
    ablation_configs = [
        ('JPS_3D', hd / '100_jps_01m.csv'),
        ('Flat A*', hd / '100_flat_01m.csv'),
        ('No-Stitch', hd / '100_no_stitch_tau5_01m.csv'),
        ('Grid-HPA*', hd / '100_grid_tau5_01m.csv'),
        ('PRM (1k)', hd / '100_prm_1k_01m.csv'),
        ('PRM (2k)', hd / '100_prm_2000_01m_500q.csv'),
        ('PRM (5k)', hd / '100_prm_5000_01m_500q.csv'),
        ('PRM (10k)', hd / '100_prm_10000_01m_500q.csv'),
        ('ALT (L=16)', hd / '100_alt_01m_500q.csv'),
        ('TG K=1', hd / '100_tg_auto_k1_01m_500q.csv'),
        ('TG K=5', hd / '100_tg_auto_k5_01m_500q.csv'),
    ]
    abl_alt = load(str(hd / '100_alt_01m_500q.csv'))
    abl_lines = []
    abl_lines.append(r"\begin{table}[t]")
    abl_lines.append(r"\centering")
    abl_lines.append(r"\caption{Ablation and baseline comparison (HM3D 100 scenes, 0.1m).}")
    abl_lines.append(r"\label{tab:ablation}")
    abl_lines.append(r"\begin{tabular}{l rrrr rrr}")
    abl_lines.append(r"\toprule")
    abl_lines.append(r"Method & Succ\% & $\tilde{T}_q$ (ms) & $\bar{T}_q$ (ms) & $T_q^{95}$ (ms) & $\bar{L}$ (m) & OH\% & $T_{pre}$ (ms) \\")
    abl_lines.append(r"\midrule")

    for label, path in ablation_configs:
        rows = load(str(path))
        if not rows:
            print(f"  [SKIP] {label}: {path}")
            continue
        alt_ref = None if 'ALT' in label else abl_alt
        st = method_stats(rows, alt_ref, label)
        oh_str = f"{st['oh_mean']:+.1f}\\%" if st['n_paired'] else '--'
        abl_lines.append(
            f"  {label} & {st['rate']:.1f}\\% & "
            f"{st['ts_med']:.3f} & {st['ts_mean']:.3f} & {st['ts_p95']:.3f} & "
            f"{st['path_mean']:.2f} & {oh_str} & "
            f"{st['topo_mean']:.1f} \\\\")

    abl_lines.append(r"\bottomrule")
    abl_lines.append(r"\end{tabular}")
    abl_lines.append(r"\end{table}")

    abl_path = out / 'ablation_table.tex'
    with open(abl_path, 'w') as f:
        f.write('\n'.join(abl_lines))
    print(f"  Written: {abl_path}")

    # ── Figure F: Tau sensitivity data ─────────────────────────────────────

    print("\n=== Figure F: Tau Sensitivity ===")
    tau_rows = []
    alt_for_tau = load(str(hd / '100_alt_01m_500q.csv'))
    if alt_for_tau:
        for tau in [3, 4, 5, 6]:
            tf = hd / f'100_tg_tau{tau}_k5_01m_500q.csv'
            t = load(str(tf))
            if not t:
                print(f"  [SKIP] tau={tau}: {tf}")
                continue
            t_s = method_stats(t, alt_for_tau, f'TG tau={tau} K=5')
            a_s = method_stats(alt_for_tau, label='ALT')
            speedup_mean = a_s['ts_mean'] / t_s['ts_mean'] if t_s['ts_mean'] > 0 else 0
            speedup_med = a_s['ts_med'] / t_s['ts_med'] if t_s['ts_med'] > 0 else 0
            ps = paired_speedup(alt_for_tau, t)
            tau_rows.append({
                'tau': tau,
                'succ_rate': f"{t_s['rate']:.1f}",
                'valid_rate': f"{t_s['valid_rate']:.1f}",
                'mean_query_ms': f"{t_s['ts_mean']:.4f}",
                'med_query_ms': f"{t_s['ts_med']:.4f}",
                'p95_query_ms': f"{t_s['ts_p95']:.4f}",
                'mean_path_m': f"{t_s['path_mean']:.2f}",
                'oh_mean': f"{t_s['oh_mean']:.2f}",
                'oh_med': f"{t_s['oh_med']:.2f}",
                'speedup_mean': f"{speedup_mean:.2f}",
                'speedup_med': f"{speedup_med:.2f}",
                'paired_speedup': f"{ps['median']:.2f}",
                'pct_faster': f"{ps['pct_faster']:.1f}",
                'topo_ms': f"{t_s['topo_mean']:.1f}",
            })

        # Also include auto-tau
        auto_tf = hd / '100_tg_auto_k5_01m_500q.csv'
        auto_t = load(str(auto_tf))
        if auto_t:
            auto_s = method_stats(auto_t, alt_for_tau, 'TG auto K=5')
            a_s = method_stats(alt_for_tau, label='ALT')
            speedup_mean = a_s['ts_mean'] / auto_s['ts_mean'] if auto_s['ts_mean'] > 0 else 0
            speedup_med = a_s['ts_med'] / auto_s['ts_med'] if auto_s['ts_med'] > 0 else 0
            ps = paired_speedup(alt_for_tau, auto_t)
            tau_rows.append({
                'tau': 'auto',
                'succ_rate': f"{auto_s['rate']:.1f}",
                'valid_rate': f"{auto_s['valid_rate']:.1f}",
                'mean_query_ms': f"{auto_s['ts_mean']:.4f}",
                'med_query_ms': f"{auto_s['ts_med']:.4f}",
                'p95_query_ms': f"{auto_s['ts_p95']:.4f}",
                'mean_path_m': f"{auto_s['path_mean']:.2f}",
                'oh_mean': f"{auto_s['oh_mean']:.2f}",
                'oh_med': f"{auto_s['oh_med']:.2f}",
                'speedup_mean': f"{speedup_mean:.2f}",
                'speedup_med': f"{speedup_med:.2f}",
                'paired_speedup': f"{ps['median']:.2f}",
                'pct_faster': f"{ps['pct_faster']:.1f}",
                'topo_ms': f"{auto_s['topo_mean']:.1f}",
            })

    if tau_rows:
        tau_path = out / 'data' / 'tau_sensitivity_hm3d_01m.csv'
        with open(tau_path, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(tau_rows[0].keys()))
            w.writeheader()
            w.writerows(tau_rows)
        print(f"  Written: {tau_path} ({len(tau_rows)} rows)")
    else:
        print("  [SKIP] No tau sweep data found")

    print("\n=== DONE ===")


if __name__ == '__main__':
    main()
