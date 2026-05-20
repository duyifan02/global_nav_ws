#!/usr/bin/env python3
"""
Generate paper figures from HM3D 100-scene benchmark results.
Output: experiments/results/figures/
"""
import csv, statistics, os
from collections import defaultdict
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

BASE = os.path.join(os.path.dirname(__file__), '../experiments/results/hm3d_100')
OUT  = os.path.join(os.path.dirname(__file__), '../experiments/results/figures')
os.makedirs(OUT, exist_ok=True)

plt.rcParams.update({
    'font.size': 11, 'axes.labelsize': 12, 'axes.titlesize': 12,
    'xtick.labelsize': 10, 'ytick.labelsize': 10,
    'legend.fontsize': 10, 'figure.dpi': 150,
})


def load_ts(fn):
    path = os.path.join(BASE, fn)
    if not os.path.exists(path):
        return {}
    by_scene = defaultdict(list)
    for r in csv.DictReader(open(path)):
        if str(r.get('success', '')).strip().lower() in ('true', '1', 'yes'):
            t = r.get('planning_time_ms', '').strip()
            if t and t not in ('None', 'nan'):
                by_scene[r['scene'].replace('hm3d_', '')].append(float(t))
    return {sc: statistics.median(v) for sc, v in by_scene.items()}


def load_path_len(fn):
    path = os.path.join(BASE, fn)
    if not os.path.exists(path):
        return {}
    by_scene = defaultdict(list)
    for r in csv.DictReader(open(path)):
        if str(r.get('success', '')).strip().lower() in ('true', '1', 'yes'):
            pl = r.get('path_length_m', '').strip()
            if pl and pl not in ('None', 'nan'):
                by_scene[r['scene'].replace('hm3d_', '')].append(float(pl))
    return {sc: statistics.median(v) for sc, v in by_scene.items()}


# ── Load all methods ──────────────────────────────────────────────────────────
jps   = load_ts('100_jps_01m.csv')
alt   = load_ts('100_alt_01m.csv')
flat  = load_ts('100_flat_01m.csv')
nos   = load_ts('100_no_stitch_tau5_01m.csv')
grid  = load_ts('100_grid_tau5_01m.csv')
prm   = load_ts('100_prm_1k_01m.csv')
full5 = load_ts('100_full_tau5_01m.csv')
tg3   = load_ts('100_tg_tau3_01m.csv')
tg4   = load_ts('100_tg_tau4_01m.csv')
tg5   = load_ts('100_tg_tau5_01m.csv')

jps_pl   = load_path_len('100_jps_01m.csv')
alt_pl   = load_path_len('100_alt_01m.csv')
nos_pl   = load_path_len('100_no_stitch_tau5_01m.csv')
grid_pl  = load_path_len('100_grid_tau5_01m.csv')
prm_pl   = load_path_len('100_prm_1k_01m.csv')
full5_pl = load_path_len('100_full_tau5_01m.csv')
tg5_pl   = load_path_len('100_tg_tau5_01m.csv')
tg3_pl   = load_path_len('100_tg_tau3_01m.csv')


# ── Figure 1: Speedup distribution histogram (TG5 vs ALT) ────────────────────
def fig_speedup_distribution():
    common = set(jps) & set(alt) & set(tg5)
    speedups = sorted([alt[sc] / tg5[sc] for sc in common])

    bins = [0, 0.5, 1, 2, 5, 10, 100]
    labels = ['<0.5x', '0.5–1x', '1–2x', '2–5x', '5–10x', '≥10x']
    counts = [0]*6
    for s in speedups:
        for i, (lo, hi) in enumerate(zip(bins[:-1], bins[1:])):
            if lo <= s < hi:
                counts[i] += 1
                break

    colors = ['#d73027', '#fc8d59', '#91bfdb', '#4575b4', '#1a9641', '#006d2c']
    fig, ax = plt.subplots(figsize=(6, 3.5))
    bars = ax.bar(labels, counts, color=colors, edgecolor='white', linewidth=0.5)
    ax.set_xlabel('Speedup (TG τ=5 vs ALT A*)')
    ax.set_ylabel('Number of scenes')
    ax.set_title(f'Query speedup distribution on {len(common)} HM3D scenes')
    for bar, c in zip(bars, counts):
        if c > 0:
            ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.3,
                    str(c), ha='center', va='bottom', fontsize=10, fontweight='bold')
    med = statistics.median(speedups)
    ax.axvline(x=2.5, color='gray', linestyle='--', alpha=0.5)
    ax.text(2.5, max(counts)*0.95, f'  median {med:.1f}x', fontsize=9, color='gray')
    ax.set_ylim(0, max(counts) * 1.2)
    plt.tight_layout()
    fig.savefig(os.path.join(OUT, 'fig1_speedup_distribution.pdf'), bbox_inches='tight')
    fig.savefig(os.path.join(OUT, 'fig1_speedup_distribution.png'), bbox_inches='tight')
    plt.close()
    print(f'Fig1 saved: speedup distribution ({len(common)} scenes, median {med:.1f}x)')


# ── Figure 2: Speed-quality Pareto scatter ────────────────────────────────────
def fig_pareto_scatter():
    def path_overhead(d_pl):
        common = set(jps_pl) & set(d_pl)
        if not common: return None
        oh = [(d_pl[s]-jps_pl[s])/jps_pl[s]*100 for s in common if jps_pl[s]>0]
        return statistics.median(oh)

    def median_ts(d):
        if not d: return None
        return statistics.median(d.values())

    methods = [
        ('Surface A*',        jps,   jps_pl,   '#888888', 'o', 8),
        ('PRM (1k)',           prm,   prm_pl,   '#b15928', 's', 8),
        ('no-stitch (PH)',     nos,   nos_pl,   '#ff7f00', 'D', 8),
        ('Grid HPA*',         grid,  grid_pl,  '#a65628', '^', 9),
        ('ALT A* (16 lm)',    alt,   alt_pl,   '#1f78b4', 'o', 10),
        ('Full HPA* τ=5',     full5, full5_pl, '#cab2d6', 's', 9),
        ('TG τ=3',            tg3,   tg3_pl,   '#33a02c', 'v', 10),
        ('TG τ=5 (ours)',     tg5,   tg5_pl,   '#e31a1c', '*', 14),
    ]

    fig, ax = plt.subplots(figsize=(6, 4.5))
    for name, ts_d, pl_d, color, marker, ms in methods:
        ts = median_ts(ts_d)
        oh = path_overhead(pl_d)
        if ts is None or oh is None: continue
        ax.scatter(ts, oh, c=color, marker=marker, s=ms**2, zorder=5,
                   label=name, edgecolors='white', linewidths=0.5)

    ax.set_xlabel('Median query time $T_s$ (ms, lower is better →)')
    ax.set_ylabel('Path length overhead vs JPS (%, lower is better ↓)')
    ax.set_title('Speed–quality Pareto: HM3D 100 scenes (0.1m)')
    ax.legend(loc='upper right', framealpha=0.9, fontsize=9)
    ax.set_xlim(-0.02, 0.38)
    ax.set_ylim(-5, 58)
    ax.grid(True, alpha=0.3)
    # Pareto frontier annotation
    ax.annotate('TG τ=5\n(Pareto front)', xy=(0.045, 12), xytext=(0.13, 5),
                arrowprops=dict(arrowstyle='->', color='#e31a1c'),
                fontsize=9, color='#e31a1c')
    plt.tight_layout()
    fig.savefig(os.path.join(OUT, 'fig2_pareto_scatter.pdf'), bbox_inches='tight')
    fig.savefig(os.path.join(OUT, 'fig2_pareto_scatter.png'), bbox_inches='tight')
    plt.close()
    print('Fig2 saved: speed-quality Pareto scatter')


# ── Figure 3: Ablation bar chart (component contributions) ───────────────────
def fig_ablation_bars():
    jps_med = statistics.median(jps.values())

    def speedup_vs_jps(d):
        common = set(jps) & set(d)
        if not common: return None
        return statistics.median([jps[sc]/d[sc] for sc in common])

    entries = [
        ('Surface A*\n(JPS)',          jps,   '#888888'),
        ('PRM\n(1k)',                   prm,   '#b15928'),
        ('No-stitch\n(PH, τ=5)',       nos,   '#ff7f00'),
        ('ALT A*\n(16 lm)',            alt,   '#1f78b4'),
        ('Grid HPA*\n(τ=5)',           grid,  '#a65628'),
        ('Full HPA*\n(τ=5)',           full5, '#cab2d6'),
        ('PH-ALT-TG\nτ=3',            tg3,   '#33a02c'),
        ('PH-ALT-TG\nτ=5 (ours)',     tg5,   '#e31a1c'),
    ]

    labels = [e[0] for e in entries]
    speedups = [speedup_vs_jps(e[1]) or 1.0 for e in entries]
    colors = [e[2] for e in entries]

    fig, ax = plt.subplots(figsize=(8, 3.8))
    bars = ax.bar(labels, speedups, color=colors, edgecolor='white', linewidth=0.5)
    ax.axhline(y=1, color='gray', linestyle='--', linewidth=0.8)
    ax.set_ylabel('Speedup vs Surface A* (×, higher is better)')
    ax.set_title('Query time speedup: HM3D 100 scenes, 0.1m resolution')
    for bar, s in zip(bars, speedups):
        ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.1,
                f'{s:.1f}x', ha='center', va='bottom', fontsize=9, fontweight='bold')
    ax.set_ylim(0, max(speedups) * 1.18)
    plt.tight_layout()
    fig.savefig(os.path.join(OUT, 'fig3_ablation_bars.pdf'), bbox_inches='tight')
    fig.savefig(os.path.join(OUT, 'fig3_ablation_bars.png'), bbox_inches='tight')
    plt.close()
    print('Fig3 saved: ablation bar chart')


# ── Figure 4: Best-tau per-scene distribution ─────────────────────────────────
def fig_best_tau():
    all_scenes = set(alt) & set(tg3) & set(tg4) & set(tg5)
    tau_wins = defaultdict(int)
    alt_wins = 0
    for sc in all_scenes:
        best_tg = min(tg3.get(sc,1e9), tg4.get(sc,1e9), tg5.get(sc,1e9))
        if best_tg < alt[sc]:
            if tg3.get(sc,1e9) == best_tg:
                tau_wins['τ=3'] += 1
            elif tg4.get(sc,1e9) == best_tg:
                tau_wins['τ=4'] += 1
            else:
                tau_wins['τ=5'] += 1
        else:
            alt_wins += 1

    sizes  = [tau_wins['τ=3'], tau_wins['τ=4'], tau_wins['τ=5'], alt_wins]
    labels = [f'TG τ=3\n({tau_wins["τ=3"]} scenes)',
              f'TG τ=4\n({tau_wins["τ=4"]} scenes)',
              f'TG τ=5\n({tau_wins["τ=5"]} scenes)',
              f'ALT wins\n({alt_wins} scenes)']
    colors = ['#33a02c', '#b2df8a', '#e31a1c', '#1f78b4']

    fig, ax = plt.subplots(figsize=(5, 4))
    wedges, texts, autotexts = ax.pie(sizes, labels=labels, colors=colors,
                                       autopct='%1.0f%%', startangle=90,
                                       wedgeprops={'edgecolor': 'white', 'linewidth': 1.5})
    for at in autotexts:
        at.set_fontsize(10)
    ax.set_title(f'Best τ per scene vs ALT ({len(all_scenes)} scenes)\n'
                 f'TG wins: {sum(tau_wins.values())}/{len(all_scenes)} (96%)')
    plt.tight_layout()
    fig.savefig(os.path.join(OUT, 'fig4_best_tau_pie.pdf'), bbox_inches='tight')
    fig.savefig(os.path.join(OUT, 'fig4_best_tau_pie.png'), bbox_inches='tight')
    plt.close()
    print(f'Fig4 saved: best-tau pie ({len(all_scenes)} scenes)')


# ── Figure 5: CDF of per-scene speedup (TG5 vs ALT) ─────────────────────────
def fig_speedup_cdf():
    common = set(alt) & set(tg5)
    speedups = sorted([alt[sc]/tg5[sc] for sc in common])
    n = len(speedups)
    cdf = [(i+1)/n for i in range(n)]

    # Also TG3
    common3 = set(alt) & set(tg3)
    speedups3 = sorted([alt[sc]/tg3[sc] for sc in common3])
    cdf3 = [(i+1)/len(speedups3) for i in range(len(speedups3))]

    fig, ax = plt.subplots(figsize=(5.5, 3.5))
    ax.plot(speedups3, cdf3, color='#33a02c', linewidth=1.8, label='TG τ=3')
    ax.plot(speedups, cdf, color='#e31a1c', linewidth=1.8, label='TG τ=5')
    ax.axvline(x=1, color='gray', linestyle='--', linewidth=1, alpha=0.8)
    ax.axhline(y=0.5, color='gray', linestyle=':', linewidth=0.8, alpha=0.5)
    med5 = statistics.median(speedups)
    med3 = statistics.median(speedups3)
    ax.axvline(x=med5, color='#e31a1c', linestyle=':', linewidth=1, alpha=0.7)
    ax.axvline(x=med3, color='#33a02c', linestyle=':', linewidth=1, alpha=0.7)
    ax.text(med5+0.1, 0.15, f'τ=5 median\n{med5:.1f}x', fontsize=8.5, color='#e31a1c')
    ax.text(med3+0.1, 0.35, f'τ=3 median\n{med3:.1f}x', fontsize=8.5, color='#33a02c')
    wins5 = sum(1 for s in speedups if s>1)/n
    wins3 = sum(1 for s in speedups3 if s>1)/len(speedups3)
    ax.text(0.02, 0.92, f'TG τ=5 faster: {wins5*100:.0f}%\nTG τ=3 faster: {wins3*100:.0f}%',
            transform=ax.transAxes, fontsize=9, verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    ax.set_xlabel('Speedup vs ALT A* (×)')
    ax.set_ylabel('Fraction of scenes (CDF)')
    ax.set_title(f'Per-scene speedup CDF: TG vs ALT ({n} scenes)')
    ax.legend(loc='lower right')
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 1.02)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    fig.savefig(os.path.join(OUT, 'fig5_speedup_cdf.pdf'), bbox_inches='tight')
    fig.savefig(os.path.join(OUT, 'fig5_speedup_cdf.png'), bbox_inches='tight')
    plt.close()
    print(f'Fig5 saved: speedup CDF ({n} scenes)')


if __name__ == '__main__':
    print("Generating figures...")
    fig_speedup_distribution()
    fig_pareto_scatter()
    fig_ablation_bars()
    fig_best_tau()
    fig_speedup_cdf()
    print(f"\nAll figures saved to: {OUT}")
