#!/usr/bin/env python3
"""Generate paper-ready PDF figures from CSV data.

Plots:
  1. Pareto (speed vs quality) scatter
  2. Overhead CDF (K=1 vs K=5)
  3. Speedup vs scene complexity scatter
  4. Amortization curve
  5. Tau sensitivity bar chart (when data available)
  6. Resolution scaling (when data available)

Usage:
    python3 plot_paper_figures.py \
      --data-dir experiments/figures/data \
      --analysis-dir experiments/results/analysis \
      --out-dir experiments/figures
"""
import argparse
import csv
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.ticker as mticker

# Paper style
FONT_SIZE = 10
TICK_SIZE = 9
ANNOT_SIZE = 8
PAPER_FIG_DIR = Path('/data2/dyf/papers/Beyond_Geometry/paper_Extended/figures')

plt.rcParams.update({
    'font.size': FONT_SIZE,
    'font.family': 'serif',
    'axes.labelsize': FONT_SIZE,
    'axes.titlesize': FONT_SIZE,
    'xtick.labelsize': TICK_SIZE,
    'ytick.labelsize': TICK_SIZE,
    'legend.fontsize': ANNOT_SIZE,
    'figure.figsize': (4.5, 3.5),
    'figure.dpi': 150,
    'savefig.dpi': 300,
    'savefig.bbox': 'tight',
    'savefig.pad_inches': 0.05,
    'axes.grid': True,
    'grid.alpha': 0.3,
})


def save_figure(fig, path, paper_name=None):
    fig.savefig(path, bbox_inches='tight', pad_inches=0.05)
    print(f"  Written: {path}")
    if paper_name:
        paper_path = PAPER_FIG_DIR / paper_name
        fig.savefig(paper_path, bbox_inches='tight', pad_inches=0.05)
        print(f"  Written: {paper_path}")

COLORS = {
    'ALT': '#2196F3',
    'TG K=1': '#FF9800',
    'TG K=5': '#4CAF50',
    'JPS_3D': '#9E9E9E',
    'Flat A*': '#795548',
    'PRM 1k': '#D32F2F',   # red
    'PRM 2k': '#FBC02D',   # yellow
    'PRM 5k': '#1976D2',   # blue
    'PRM 10k': '#388E3C',  # green
    'Grid-HPA*': '#9C27B0',
    'No-Stitch': '#607D8B',
}

MARKERS = {
    'ALT': 's',
    'TG K=1': '^',
    'TG K=5': 'D',
    'JPS_3D': 'v',
    'Flat A*': 'o',
    'PRM 1k': 'p',
    'PRM 2k': 'p',
    'PRM 5k': 'p',
    'PRM 10k': 'p',
    'Grid-HPA*': 'h',
    'No-Stitch': 'X',
}


def sf(v, default=0.0):
    try:
        return float(v)
    except (ValueError, TypeError):
        return default


def load(path):
    p = Path(path)
    stem = p.stem
    # Strip existing version suffix to find base name
    if stem.endswith('_v2') or stem.endswith('_v3'):
        stem = stem[:-3]
    # Prefer v3 > v2 > original
    for suffix in ('_v3', '_v2', ''):
        candidate = p.parent / (stem + suffix + '.csv')
        if candidate.exists():
            if suffix:
                print(f"  [{suffix[1:]}] Using {candidate.name}")
            with open(candidate) as f:
                return list(csv.DictReader(f))
    return None


def plot_pareto(data_dir, out_dir):
    """Figure B: Speed/quality Pareto as stacked Cleveland dot plots."""
    rows = load(data_dir / 'pareto_hm3d_01m.csv')
    if not rows:
        print("  [SKIP] pareto: no data")
        return

    # Exclude PRM variants from the speed-quality Pareto: PRM returns paths
    # with 0% surface-graph validity, so its overhead is not comparable.
    excluded = {'PRM 1k', 'PRM 2k', 'PRM 5k', 'PRM 10k'}
    kept = []
    for r in rows:
        if r['method'] in excluded:
            continue
        kept.append({
            'method': r['method'],
            'query_ms': sf(r.get('med_query_ms', r.get('mean_query_ms'))),
            'overhead_pct': sf(r.get('med_overhead_pct') or r.get('mean_overhead_pct')),
        })

    kept.sort(key=lambda r: r['query_ms'])
    y = list(range(len(kept)))

    fig, (ax_a, ax_b) = plt.subplots(
        2, 1, figsize=(3.5, 5.0), sharey=True,
        gridspec_kw={'hspace': 0.42},
    )

    for i, r in enumerate(kept):
        m = r['method']
        x = r['query_ms']
        color = '#000000'
        marker = 'o'
        ax_a.hlines(y=i, xmin=0, xmax=x, color=color, linewidth=1.5, alpha=0.7)
        ax_a.scatter(x, i, color=color, marker=marker, s=70, zorder=5,
                     edgecolors='white', linewidths=0.5)
        ax_a.annotate(f"{x:.3f}", xy=(x, i), xytext=(4, 0),
                      textcoords='offset points', ha='left', va='center',
                      fontsize=ANNOT_SIZE, clip_on=True)

        oh = r['overhead_pct']
        xmin, xmax = (oh, 0) if oh < 0 else (0, oh)
        ax_b.hlines(y=i, xmin=xmin, xmax=xmax, color=color, linewidth=1.5, alpha=0.7)
        ax_b.scatter(oh, i, color=color, marker=marker, s=70, zorder=5,
                     edgecolors='white', linewidths=0.5)
        if m == 'ALT':
            label = 'baseline'
        elif abs(oh) < 0.05:
            label = '0%'
        else:
            label = f"{oh:+.1f}%"
        ax_b.annotate(label, xy=(oh, i), xytext=(4, 0), textcoords='offset points',
                      ha='left', va='center', fontsize=ANNOT_SIZE)

    labels = [r['method'] for r in kept]
    for ax in (ax_a, ax_b):
        ax.set_yticks(y)
        ax.set_yticklabels(labels)
        ax.grid(axis='x', alpha=0.3)
        ax.grid(axis='y', visible=False)
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)

    ax_a.invert_yaxis()
    ax_a.set_xlim(0, max(r['query_ms'] for r in kept) * 1.22)
    ax_a.set_xlabel('Median Query Time (ms)')
    ax_a.set_title('(a) Median Query Time (ms)', loc='left')

    oh_vals = [r['overhead_pct'] for r in kept]
    oh_min = min(oh_vals + [0])
    oh_max = max(oh_vals + [0])
    left_pad = max((oh_max - oh_min) * 0.08, 2.0)
    right_pad = max((oh_max - oh_min) * 0.28, 8.0)
    ax_b.set_xlim(oh_min - left_pad, oh_max + right_pad)
    ax_b.axvline(x=0, color='gray', linestyle=':', alpha=0.5, linewidth=0.8)
    ax_b.tick_params(labelleft=True)
    ax_b.set_xlabel('Median Path Overhead vs ALT (%)')
    ax_b.set_title('(b) Median Path Overhead vs ALT (%)', loc='left')

    fig.align_ylabels()

    path = out_dir / 'pareto_hm3d_01m.pdf'
    save_figure(fig, path, 'pareto.pdf')
    plt.close(fig)

def plot_overhead_cdf(data_dir, out_dir):
    """Figure C: Overhead CDF."""
    rows = load(data_dir / 'overhead_cdf_hm3d_01m.csv')
    if not rows:
        print("  [SKIP] overhead CDF: no data")
        return

    # Display name → (color, linestyle, linewidth, zorder)
    METHOD_STYLE = {
        'K=1':      ('TG K=1',     COLORS['TG K=1'],    '-',  2.2, 6),
        'K=5':      ('TG K=5',     COLORS['TG K=5'],    '-',  2.2, 6),
        'JPS_3D':   ('JPS 3D',     COLORS['JPS_3D'],    '--', 1.4, 4),
        'Flat A*':  ('Flat A*',    COLORS['Flat A*'],   '--', 1.4, 4),
        'No-Stitch':('No-Stitch',  COLORS['No-Stitch'], ':',  1.4, 3),
        'Grid-HPA*':('Grid-HPA*',  COLORS['Grid-HPA*'], '-.',  1.4, 3),
    }

    # Clip x-axis to -30..60 so heavy tails don't compress the interesting region
    X_MIN, X_MAX = -30, 60

    fig, ax = plt.subplots(figsize=(5.5, 3.8))

    method_order = ['K=1', 'K=5', 'JPS_3D', 'Flat A*', 'No-Stitch', 'Grid-HPA*']
    for method in method_order:
        if method not in METHOD_STYLE:
            continue
        mrows = [r for r in rows if r['method'] == method]
        if not mrows:
            continue
        label, color, ls, lw, zo = METHOD_STYLE[method]
        x = [sf(r['overhead_pct']) for r in mrows]
        y = [sf(r['cdf']) * 100 for r in mrows]
        ax.plot(x, y, color=color, linestyle=ls, linewidth=lw,
                label=label, zorder=zo)

    # Reference lines
    ax.axvline(x=0,  color='gray', linestyle=':', alpha=0.45, linewidth=0.8)
    ax.axhline(y=50, color='gray', linestyle='--', alpha=0.25, linewidth=0.8)
    ax.axhline(y=95, color='gray', linestyle='--', alpha=0.25, linewidth=0.8)
    ax.text(X_MAX - 0.5, 51, '50%', color='gray', fontsize=ANNOT_SIZE, va='bottom', ha='right', alpha=0.7)
    ax.text(X_MAX - 0.5, 96, '95%', color='gray', fontsize=ANNOT_SIZE, va='bottom', ha='right', alpha=0.7)

    # Median annotations for TG K=1 and K=5 — place above each other to avoid overlap
    MEDIAN_ANNOT = [
        ('K=5',  2.89,  COLORS['TG K=5'],  72),   # (method_key, med_x, color, text_y)
        ('K=1',  5.03,  COLORS['TG K=1'],  56),
    ]
    for _key, med_x, col, ty in MEDIAN_ANNOT:
        ax.axvline(x=med_x, color=col, linestyle='--', alpha=0.6, linewidth=1.0, zorder=2)
        label_name = 'TG K=5' if _key == 'K=5' else 'TG K=1'
        ax.annotate(
            f'{label_name}\n{med_x:+.1f}%',
            xy=(med_x, 50), xycoords='data',
            xytext=(med_x + 1.2, ty), textcoords='data',
            fontsize=ANNOT_SIZE, color=col, va='center', ha='left',
            arrowprops=dict(arrowstyle='->', color=col, lw=0.8),
            bbox=dict(boxstyle='round,pad=0.2', fc='white', alpha=0.85, ec='none'),
            zorder=8,
        )

    ax.set_xlabel('Path Overhead vs ALT (%)')
    ax.set_ylabel('CDF (%)')
    ax.set_title('Path Overhead CDF (HM3D 100 scenes, 0.1 m)')
    ax.set_xlim(X_MIN, X_MAX)
    ax.set_ylim(0, 100)

    # Legend outside axes (right side) to avoid blocking curves
    ax.legend(loc='center left', bbox_to_anchor=(1.02, 0.5),
              framealpha=0.9, borderaxespad=0.0)

    fig.tight_layout()
    path = out_dir / 'overhead_cdf_hm3d_01m.pdf'
    save_figure(fig, path, 'overhead_cdf.pdf')
    plt.close(fig)


def plot_speedup_scatter(analysis_dir, out_dir):
    """Figure E: Speedup vs scene complexity."""
    rows = load(analysis_dir / 'scene_complexity_k5.csv')
    if not rows:
        print("  [SKIP] speedup scatter: no data")
        return

    valid = [r for r in rows if sf(r.get('speedup', 0)) > 0 and sf(r.get('n_paired', 0)) > 0]
    if not valid:
        print("  [SKIP] speedup scatter: no valid scenes")
        return

    # 1. Speedup vs mean geodesic distance
    fig, ax = plt.subplots()
    geo = [sf(r['mean_geodesic_m']) for r in valid]
    sp = [sf(r['speedup']) for r in valid]
    oh = [sf(r['mean_path_overhead_pct']) for r in valid]

    scatter = ax.scatter(geo, sp, c=oh, cmap='RdYlGn_r', s=30, alpha=0.7,
                         edgecolors='gray', linewidths=0.3, vmin=-20, vmax=50)
    fig.colorbar(scatter, ax=ax, label='Path Overhead (%)', shrink=0.8)
    ax.axhline(y=1, color='red', linestyle='--', alpha=0.5, linewidth=0.8, label='Speedup = 1x')
    ax.set_xlabel('Mean Query Geodesic Distance (m)')
    ax.set_ylabel('Speedup vs ALT')
    ax.set_title('Speedup vs Query Distance (HM3D, K=5)')
    ax.legend(loc='upper right')

    path = out_dir / 'speedup_vs_geodesic.pdf'
    save_figure(fig, path, 'speedup_vs_geodesic.pdf')
    plt.close(fig)

    # 2. Speedup vs TG nodes
    fig, ax = plt.subplots()
    tg = [sf(r['tg_nodes']) for r in valid]
    ax.scatter(tg, sp, c=oh, cmap='RdYlGn_r', s=30, alpha=0.7,
               edgecolors='gray', linewidths=0.3, vmin=-20, vmax=50)
    ax.axhline(y=1, color='red', linestyle='--', alpha=0.5, linewidth=0.8)
    ax.set_xlabel('TG Nodes (portals)')
    ax.set_ylabel('Speedup vs ALT')
    ax.set_title('Speedup vs Graph Complexity (HM3D, K=5)')

    path = out_dir / 'speedup_vs_tg_nodes.pdf'
    save_figure(fig, path, 'speedup_vs_tg_nodes.pdf')
    plt.close(fig)


def plot_amortization(analysis_dir, out_dir):
    """Figure: Amortization curve."""
    for dataset in ['hm3d', 'mp3d']:
        rows = load(analysis_dir / f'amortization_{dataset}_01m.csv')
        if not rows:
            continue

        fig, ax = plt.subplots()
        ns = [sf(r['N']) for r in rows]
        for col, label, color in [
            ('ALT_total_ms', 'ALT (L=16)', COLORS['ALT']),
            ('TG_K1_total_ms', 'TG K=1', COLORS['TG K=1']),
            ('TG_K5_total_ms', 'TG K=5', COLORS['TG K=5']),
        ]:
            vals = [sf(r[col]) for r in rows]
            ax.plot(ns, vals, color=color, linewidth=1.8, marker='o', markersize=4, label=label)

        ax.set_xscale('log')
        ax.set_yscale('log')
        ax.set_xlabel('Number of Queries (N)')
        ax.set_ylabel('Total Time (ms)')
        ax.set_title(f'Amortized Cost ({dataset.upper()}, 0.1m)')
        ax.legend(loc='upper left')
        ax.xaxis.set_major_formatter(mticker.ScalarFormatter())

        path = out_dir / f'amortization_{dataset}_01m.pdf'
        paper_name = 'amortization.pdf' if dataset == 'hm3d' else f'amortization_{dataset}.pdf'
        save_figure(fig, path, paper_name)
        plt.close(fig)


def plot_tau_sensitivity(data_dir, out_dir):
    """Figure F: Tau sensitivity."""
    rows = load(data_dir / 'tau_sensitivity_hm3d_01m.csv')
    if not rows or len(rows) < 2:
        print("  [SKIP] tau sensitivity: insufficient data (need tau sweep results)")
        return

    fixed = [r for r in rows if r['tau'] != 'auto']
    auto = [r for r in rows if r['tau'] == 'auto']

    if not fixed:
        print("  [SKIP] tau sensitivity: no fixed-tau data")
        return

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(9, 3.5))

    taus = [str(r['tau']) for r in fixed]
    query_ms = [sf(r.get('med_query_ms', r['mean_query_ms'])) for r in fixed]
    oh_mean = [sf(r['oh_mean']) for r in fixed]

    x = range(len(taus))
    bars1 = ax1.bar(x, query_ms, color=COLORS['TG K=5'], alpha=0.8, label='Fixed tau')
    if auto:
        ax1.axhline(y=sf(auto[0].get('med_query_ms', auto[0]['mean_query_ms'])), color='red', linestyle='--',
                     linewidth=1.2, label='Auto-tau')
    ax1.set_xlabel('Persistence Threshold (tau)')
    ax1.set_ylabel('Query Time (ms)')
    ax1.set_xticks(x)
    ax1.set_xticklabels(taus)
    ax1.legend()
    ax1.set_title('Query Time vs tau')

    bars2 = ax2.bar(x, oh_mean, color=COLORS['TG K=5'], alpha=0.8, label='Fixed tau')
    if auto:
        ax2.axhline(y=sf(auto[0]['oh_mean']), color='red', linestyle='--',
                     linewidth=1.2, label='Auto-tau')
    ax2.set_xlabel('Persistence Threshold (tau)')
    ax2.set_ylabel('Mean Path Overhead (%)')
    ax2.set_xticks(x)
    ax2.set_xticklabels(taus)
    ax2.axhline(y=0, color='gray', linestyle=':', alpha=0.5)
    ax2.legend()
    ax2.set_title('Overhead vs tau')

    fig.suptitle('Tau Sensitivity (HM3D 100, K=5, 0.1m)', fontsize=FONT_SIZE)
    fig.tight_layout()

    path = out_dir / 'tau_sensitivity_hm3d_01m.pdf'
    save_figure(fig, path, 'tau_sensitivity.pdf')
    plt.close(fig)


def plot_resolution_scaling(data_dir, out_dir):
    """Figure D: Resolution scaling."""
    rows = load(data_dir / 'resolution_scaling_hm3d.csv')
    if not rows or len(rows) < 2:
        print("  [SKIP] resolution scaling: insufficient data (need cross-res results)")
        return

    fig, (ax1, ax2) = plt.subplots(
        2, 1, figsize=(3.5, 5.0),
        gridspec_kw={'hspace': 0.42},
    )

    res = [sf(r['resolution']) for r in rows]
    res_labels = [f"{r['resolution']}m" for r in rows]

    alt_ts = [sf(r.get('alt_ts_med', r['alt_ts_mean'])) for r in rows]
    tg_ts = [sf(r.get('tg_ts_med', r['tg_ts_mean'])) for r in rows]
    speedup = [sf(r.get('speedup_med', r.get('speedup', 0))) for r in rows]
    oh = [sf(r.get('oh_med', r['oh_mean'])) for r in rows]

    x = range(len(res))
    w = 0.35
    ax1.bar([i - w/2 for i in x], alt_ts, w, color=COLORS['ALT'], label='ALT', alpha=0.8)
    ax1.bar([i + w/2 for i in x], tg_ts, w, color=COLORS['TG K=5'], label='TG K=5', alpha=0.8)
    ax1.set_xlabel('Resolution')
    ax1.set_ylabel('Median Query Time (ms)')
    ax1.set_xticks(x)
    ax1.set_xticklabels(res_labels)
    ax1.legend(loc='upper left')
    ax1.set_title('(a) Median Query Time')

    ax2.bar([i - w/2 for i in x], speedup, w, color=COLORS['TG K=5'], alpha=0.8, label='Speedup (median)')
    ax2b = ax2.twinx()
    ax2b.plot(x, oh, color='#D32F2F', marker='o', linewidth=1.5, markersize=4, label='Median overhead (%)')
    ax2.set_xlabel('Resolution')
    ax2.set_ylabel('Speedup vs ALT')
    ax2b.set_ylabel('')
    ax2b.tick_params(axis='y', colors='#D32F2F')
    ax2.set_xticks(x)
    ax2.set_xticklabels(res_labels)
    ax2.set_title('(b) Speedup and Overhead')
    lines1, labels1 = ax2.get_legend_handles_labels()
    lines2, labels2 = ax2b.get_legend_handles_labels()
    ax2.legend(lines1 + lines2, labels1 + labels2, loc='upper left')

    for ax in (ax1, ax2):
        ax.grid(axis='y', alpha=0.3)
        ax.grid(axis='x', visible=False)
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
    ax2b.spines['top'].set_visible(False)
    ax2b.spines['right'].set_color('#D32F2F')
    fig.align_ylabels([ax1, ax2])

    path = out_dir / 'resolution_scaling_hm3d.pdf'
    save_figure(fig, path, 'resolution_scaling.pdf')
    plt.close(fig)


def plot_timing_breakdown(results_dir, out_dir):
    """Figure G: Query time component breakdown (stacked bar)."""
    tg_path = Path(results_dir) / 'hm3d_100' / '100_tg_auto_k5_01m_500q.csv'
    alt_path = Path(results_dir) / 'hm3d_100' / '100_alt_01m_500q.csv'
    tg = load(tg_path)
    alt = load(alt_path)
    if not tg or not alt:
        print("  [SKIP] timing breakdown: missing data")
        return

    import statistics as st

    tg_succ = [r for r in tg if r.get('success') == 'True']
    alt_succ = [r for r in alt if r.get('success') == 'True']

    normal = [r for r in tg_succ if sf(r.get('diag_snap_ms', 0)) < 1.0]
    all_q = tg_succ

    def med(rows, key): return st.median([sf(r.get(key, 0)) for r in rows]) if rows else 0

    # Component medians for normal queries
    cats = ['Snap', 'TG A*', 'Stitch', 'Other']
    n_snap = med(normal, 'diag_snap_ms')
    n_astar = med(normal, 'diag_tg_astar_ms')
    n_stitch = med(normal, 'diag_stitch_ms')
    n_total = med(normal, 'planning_time_ms')
    n_other = max(0, n_total - n_snap - n_astar - n_stitch)
    normal_vals = [n_snap, n_astar, n_stitch, n_other]

    # All queries
    a_snap = med(all_q, 'diag_snap_ms')
    a_astar = med(all_q, 'diag_tg_astar_ms')
    a_stitch = med(all_q, 'diag_stitch_ms')
    a_total = med(all_q, 'planning_time_ms')
    a_other = max(0, a_total - a_snap - a_astar - a_stitch)
    all_vals = [a_snap, a_astar, a_stitch, a_other]

    # ALT
    alt_total = med(alt_succ, 'planning_time_ms')

    fig, ax = plt.subplots(figsize=(5, 3.5))
    x = [0, 1, 2.2]
    labels = [f'TG K=5\n(normal, n={len(normal)})',
              f'TG K=5\n(all, n={len(all_q)})',
              f'ALT\n(n={len(alt_succ)})']

    colors_stack = ['#FF9800', '#4CAF50', '#2196F3', '#9E9E9E']

    # TG bars: stacked components
    for ci, (cat, color) in enumerate(zip(cats, colors_stack)):
        vals = [normal_vals[ci], all_vals[ci]]
        bot = [sum(normal_vals[:ci]), sum(all_vals[:ci])]
        ax.bar(x[:2], vals, 0.7, bottom=bot, color=color, label=cat, alpha=0.85)

    # ALT bar: single solid bar (no breakdown available)
    ax.bar(x[2], alt_total, 0.7, color=COLORS['ALT'], label='ALT (total)', alpha=0.85)

    ax.set_xticks(x)
    ax.set_xticklabels(labels, fontsize=ANNOT_SIZE)
    ax.set_ylabel('Median Query Time (ms)')
    ax.set_title('Query Time Breakdown (HM3D 100, 0.1m)')
    ax.legend(loc='upper right')

    # Annotate totals
    for xi, total in zip(x, [n_total, a_total, alt_total]):
        ax.text(xi, total + 0.003, f'{total:.3f}', ha='center', va='bottom', fontsize=ANNOT_SIZE)

    path = out_dir / 'timing_breakdown_hm3d_01m.pdf'
    save_figure(fig, path, 'timing_breakdown.pdf')
    plt.close(fig)


def plot_speedup_histogram(results_dir, out_dir):
    """Figure H: Per-scene speedup distribution histogram."""
    tg_path = Path(results_dir) / 'hm3d_100' / '100_tg_auto_k5_01m_500q.csv'
    alt_path = Path(results_dir) / 'hm3d_100' / '100_alt_01m_500q.csv'
    tg = load(tg_path)
    alt = load(alt_path)
    if not tg or not alt:
        print("  [SKIP] speedup histogram: missing data")
        return

    import statistics as st
    from collections import defaultdict

    def succ(rows): return [r for r in rows if r.get('success') == 'True']

    # Per-scene median query time
    alt_by_scene = defaultdict(list)
    tg_by_scene = defaultdict(list)
    for r in succ(alt):
        alt_by_scene[r['scene']].append(sf(r['planning_time_ms']))
    for r in succ(tg):
        tg_by_scene[r['scene']].append(sf(r['planning_time_ms']))

    speedups = []
    for scene in alt_by_scene:
        if scene in tg_by_scene and tg_by_scene[scene]:
            alt_med = st.median(alt_by_scene[scene])
            tg_med = st.median(tg_by_scene[scene])
            if tg_med > 0:
                speedups.append(alt_med / tg_med)

    if not speedups:
        print("  [SKIP] speedup histogram: no common scenes")
        return

    fig, ax = plt.subplots(figsize=(5, 3.5))
    ax.hist(speedups, bins=25, color=COLORS['TG K=5'], alpha=0.8, edgecolor='white')
    ax.axvline(x=1, color='red', linestyle='--', linewidth=1.2, label='Speedup = 1x')
    ax.axvline(x=st.median(speedups), color='orange', linestyle='-', linewidth=1.5,
               label=f'Median = {st.median(speedups):.2f}x')
    ax.set_xlabel('Per-Scene Median Speedup (TG K=5 vs ALT)')
    ax.set_ylabel('Number of Scenes')
    ax.set_title(f'Speedup Distribution (HM3D, n={len(speedups)} scenes)')
    ax.legend()

    faster = sum(1 for s in speedups if s > 1)
    ax.text(0.98, 0.95, f'TG faster: {faster}/{len(speedups)} ({100*faster/len(speedups):.0f}%)',
            transform=ax.transAxes, ha='right', va='top', fontsize=ANNOT_SIZE,
            bbox=dict(boxstyle='round,pad=0.3', facecolor='wheat', alpha=0.8))

    path = out_dir / 'speedup_histogram_hm3d_01m.pdf'
    save_figure(fig, path, 'speedup_histogram.pdf')
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--data-dir', default='experiments/figures/data')
    ap.add_argument('--analysis-dir', default='experiments/results/analysis')
    ap.add_argument('--results-dir', default='experiments/results')
    ap.add_argument('--out-dir', default='experiments/figures')
    args = ap.parse_args()

    dd = Path(args.data_dir)
    ad = Path(args.analysis_dir)
    rd = Path(args.results_dir)
    out = Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)

    print("=== Generating Paper Figures ===\n")

    print("--- Pareto (speed vs quality) ---")
    plot_pareto(dd, out)

    print("\n--- Overhead CDF ---")
    plot_overhead_cdf(dd, out)

    print("\n--- Speedup vs Complexity ---")
    plot_speedup_scatter(ad, out)

    print("\n--- Amortization ---")
    plot_amortization(ad, out)

    print("\n--- Tau Sensitivity ---")
    plot_tau_sensitivity(dd, out)

    print("\n--- Resolution Scaling ---")
    plot_resolution_scaling(dd, out)

    print("\n--- Timing Breakdown ---")
    plot_timing_breakdown(rd, out)

    print("\n--- Speedup Histogram ---")
    plot_speedup_histogram(rd, out)

    print("\n=== DONE ===")


if __name__ == '__main__':
    main()
