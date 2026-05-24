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
plt.rcParams.update({
    'font.size': 10,
    'font.family': 'serif',
    'axes.labelsize': 11,
    'axes.titlesize': 12,
    'xtick.labelsize': 9,
    'ytick.labelsize': 9,
    'legend.fontsize': 9,
    'figure.figsize': (4.5, 3.5),
    'figure.dpi': 150,
    'savefig.dpi': 300,
    'savefig.bbox': 'tight',
    'savefig.pad_inches': 0.05,
    'axes.grid': True,
    'grid.alpha': 0.3,
})

COLORS = {
    'ALT': '#2196F3',
    'TG K=1': '#FF9800',
    'TG K=5': '#4CAF50',
    'JPS_3D': '#9E9E9E',
    'Flat A*': '#795548',
    'PRM 1k': '#E91E63',
    'PRM 2k': '#F06292',
    'PRM 5k': '#F48FB1',
    'PRM 10k': '#F8BBD0',
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
    if not p.exists():
        return None
    with open(p) as f:
        return list(csv.DictReader(f))


def plot_pareto(data_dir, out_dir):
    """Figure B: Speed vs quality Pareto plot."""
    rows = load(data_dir / 'pareto_hm3d_01m.csv')
    if not rows:
        print("  [SKIP] pareto: no data")
        return

    fig, ax = plt.subplots()
    # Custom label offsets to avoid overlaps
    label_offsets = {
        'ALT': (-25, -15),
        'TG K=1': (15, -15),
        'TG K=5': (15, 8),
        'JPS_3D': (-30, 10),
        'Flat A*': (15, 10),
        'No-Stitch': (-10, -18),
        'PRM 1k': (-15, 10),
        'Grid-HPA*': (-20, -18),
    }

    for r in rows:
        m = r['method']
        x = sf(r['mean_overhead_pct'])
        y = sf(r['mean_query_ms'])
        color = COLORS.get(m, '#333333')
        marker = MARKERS.get(m, 'o')
        ax.scatter(x, y, c=color, marker=marker, s=80, zorder=5, edgecolors='white', linewidths=0.5)
        ox, oy = label_offsets.get(m, (10, 8))
        ax.annotate(m, (x, y), textcoords="offset points",
                    xytext=(ox, oy), fontsize=7.5, ha='center')

    ax.set_xlabel('Mean Path Overhead vs ALT (%)')
    ax.set_ylabel('Mean Query Time (ms)')
    ax.set_title('Speed vs Quality (HM3D 100, 0.1m)')
    ax.axvline(x=0, color='gray', linestyle=':', alpha=0.5, linewidth=0.8)

    path = out_dir / 'pareto_hm3d_01m.pdf'
    fig.savefig(path)
    plt.close(fig)
    print(f"  Written: {path}")


def plot_overhead_cdf(data_dir, out_dir):
    """Figure C: Overhead CDF."""
    rows = load(data_dir / 'overhead_cdf_hm3d_01m.csv')
    if not rows:
        print("  [SKIP] overhead CDF: no data")
        return

    fig, ax = plt.subplots()
    methods = sorted(set(r['method'] for r in rows))
    for method in methods:
        mrows = [r for r in rows if r['method'] == method]
        x = [sf(r['overhead_pct']) for r in mrows]
        y = [sf(r['cdf']) * 100 for r in mrows]
        label = f'TG {method}'
        color = COLORS.get(label, '#333')
        ax.plot(x, y, color=color, linewidth=1.8, label=label)

    ax.set_xlabel('Path Overhead vs ALT (%)')
    ax.set_ylabel('CDF (%)')
    ax.set_title('Overhead CDF (HM3D 100, 0.1m)')
    ax.axvline(x=0, color='gray', linestyle=':', alpha=0.5, linewidth=0.8)
    ax.axhline(y=50, color='gray', linestyle='--', alpha=0.3, linewidth=0.8)
    ax.axhline(y=95, color='gray', linestyle='--', alpha=0.3, linewidth=0.8)
    ax.legend(loc='lower right')
    ax.set_xlim(-30, 60)
    ax.set_ylim(0, 100)

    path = out_dir / 'overhead_cdf_hm3d_01m.pdf'
    fig.savefig(path)
    plt.close(fig)
    print(f"  Written: {path}")


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
    ax.legend(loc='upper right', fontsize=8)

    path = out_dir / 'speedup_vs_geodesic.pdf'
    fig.savefig(path)
    plt.close(fig)
    print(f"  Written: {path}")

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
    fig.savefig(path)
    plt.close(fig)
    print(f"  Written: {path}")


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
        fig.savefig(path)
        plt.close(fig)
        print(f"  Written: {path}")


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
    query_ms = [sf(r['mean_query_ms']) for r in fixed]
    oh_mean = [sf(r['oh_mean']) for r in fixed]

    x = range(len(taus))
    bars1 = ax1.bar(x, query_ms, color=COLORS['TG K=5'], alpha=0.8, label='Fixed tau')
    if auto:
        ax1.axhline(y=sf(auto[0]['mean_query_ms']), color='red', linestyle='--',
                     linewidth=1.2, label='Auto-tau')
    ax1.set_xlabel('Persistence Threshold (tau)')
    ax1.set_ylabel('Mean Query Time (ms)')
    ax1.set_xticks(x)
    ax1.set_xticklabels(taus)
    ax1.legend(fontsize=8)
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
    ax2.legend(fontsize=8)
    ax2.set_title('Overhead vs tau')

    fig.suptitle('Tau Sensitivity (HM3D 100, K=5, 0.1m)', fontsize=12)
    fig.tight_layout()

    path = out_dir / 'tau_sensitivity_hm3d_01m.pdf'
    fig.savefig(path)
    plt.close(fig)
    print(f"  Written: {path}")


def plot_resolution_scaling(data_dir, out_dir):
    """Figure D: Resolution scaling."""
    rows = load(data_dir / 'resolution_scaling_hm3d.csv')
    if not rows or len(rows) < 2:
        print("  [SKIP] resolution scaling: insufficient data (need cross-res results)")
        return

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(9, 3.5))

    res = [sf(r['resolution']) for r in rows]
    res_labels = [f"{r['resolution']}m" for r in rows]

    alt_ts = [sf(r['alt_ts_mean']) for r in rows]
    tg_ts = [sf(r['tg_ts_mean']) for r in rows]
    speedup = [sf(r['speedup']) for r in rows]
    oh = [sf(r['oh_mean']) for r in rows]

    x = range(len(res))
    w = 0.35
    ax1.bar([i - w/2 for i in x], alt_ts, w, color=COLORS['ALT'], label='ALT', alpha=0.8)
    ax1.bar([i + w/2 for i in x], tg_ts, w, color=COLORS['TG K=5'], label='TG K=5', alpha=0.8)
    ax1.set_xlabel('Resolution')
    ax1.set_ylabel('Mean Query Time (ms)')
    ax1.set_xticks(x)
    ax1.set_xticklabels(res_labels)
    ax1.legend(fontsize=8)
    ax1.set_title('Query Time')

    ax2.bar([i - w/2 for i in x], speedup, w, color=COLORS['TG K=5'], alpha=0.8, label='Speedup')
    ax2b = ax2.twinx()
    ax2b.plot(x, oh, color='red', marker='s', linewidth=1.5, markersize=5, label='Overhead %')
    ax2.set_xlabel('Resolution')
    ax2.set_ylabel('Speedup vs ALT')
    ax2b.set_ylabel('Mean Overhead (%)', color='red')
    ax2.set_xticks(x)
    ax2.set_xticklabels(res_labels)
    ax2.set_title('Speedup & Overhead')
    lines1, labels1 = ax2.get_legend_handles_labels()
    lines2, labels2 = ax2b.get_legend_handles_labels()
    ax2.legend(lines1 + lines2, labels1 + labels2, fontsize=8, loc='upper left')

    fig.suptitle('Resolution Scaling (HM3D 100, K=5)', fontsize=12)
    fig.tight_layout()

    path = out_dir / 'resolution_scaling_hm3d.pdf'
    fig.savefig(path)
    plt.close(fig)
    print(f"  Written: {path}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--data-dir', default='experiments/figures/data')
    ap.add_argument('--analysis-dir', default='experiments/results/analysis')
    ap.add_argument('--out-dir', default='experiments/figures')
    args = ap.parse_args()

    dd = Path(args.data_dir)
    ad = Path(args.analysis_dir)
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

    print("\n=== DONE ===")


if __name__ == '__main__':
    main()
