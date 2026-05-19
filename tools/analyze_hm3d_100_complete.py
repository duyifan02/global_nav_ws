#!/usr/bin/env python3
"""
Complete analysis of HM3D 100-scene results.
Produces paper-ready tables comparing JPS, ALT, TG τ=3/4/5, Full τ=5.
"""
import csv
import statistics
import os
from collections import defaultdict

BASE = os.path.join(os.path.dirname(__file__), '../experiments/results/hm3d_100')


def load_by_scene(fn):
    path = os.path.join(BASE, fn)
    if not os.path.exists(path):
        return {}
    rows = list(csv.DictReader(open(path)))
    by_scene = {}
    for r in rows:
        sc = r['scene'].replace('hm3d_', '')
        if str(r.get('success', '')).strip().lower() in ('true', '1', 'yes'):
            t = r.get('planning_time_ms', '').strip()
            if t and t not in ('None', 'nan'):
                by_scene.setdefault(sc, []).append(float(t))
    return {sc: statistics.median(v) for sc, v in by_scene.items()}


def speedup_stats(ref, method, label=''):
    common = set(ref) & set(method)
    if not common:
        return f"{label}: no common scenes"
    su = sorted([ref[sc] / method[sc] for sc in common])
    wins = sum(1 for s in su if s > 1)
    return (f"{label} ({len(common)} scenes): faster={wins}/{len(common)} "
            f"({100*wins//len(common)}%), "
            f"median={statistics.median(su):.2f}x, "
            f"mean={statistics.mean(su):.2f}x, "
            f"p75={su[3*len(su)//4]:.2f}x")


def main():
    print("Loading results...")
    jps   = load_by_scene('100_jps_01m.csv')
    alt   = load_by_scene('100_alt_01m.csv')
    flat  = load_by_scene('100_flat_01m.csv')
    tg3   = load_by_scene('100_tg_tau3_01m.csv')
    tg4   = load_by_scene('100_tg_tau4_01m.csv')
    tg5   = load_by_scene('100_tg_tau5_01m.csv')
    full5 = load_by_scene('100_full_tau5_01m.csv')

    print(f"\nScenes loaded: JPS={len(jps)}, ALT={len(alt)}, "
          f"TG3={len(tg3)}, TG4={len(tg4)}, TG5={len(tg5)}, Full5={len(full5)}")

    # ── Per-method aggregate stats ────────────────────────────────────────
    print("\n=== Per-Method Summary ===")
    print(f"{'Method':15s} {'N':5s} {'Median':8s} {'Mean':8s} {'p95':8s}")
    for label, d in [('JPS', jps), ('flat', flat), ('ALT', alt),
                     ('TG τ=3', tg3), ('TG τ=4', tg4), ('TG τ=5', tg5),
                     ('Full τ=5', full5)]:
        if not d:
            print(f"{label:15s}  no data")
            continue
        vals = sorted(d.values())
        p95 = vals[int(0.95 * len(vals))]
        print(f"{label:15s} {len(d):5d} {statistics.median(vals):8.3f} "
              f"{statistics.mean(vals):8.3f} {p95:8.3f}")

    # ── Speedup comparisons ───────────────────────────────────────────────
    print("\n=== Speedup vs JPS ===")
    for label, d in [('ALT', alt), ('TG τ=3', tg3), ('TG τ=4', tg4),
                     ('TG τ=5', tg5), ('Full τ=5', full5)]:
        print(' ', speedup_stats(jps, d, label))

    print("\n=== Speedup vs ALT ===")
    for label, d in [('TG τ=3', tg3), ('TG τ=4', tg4), ('TG τ=5', tg5),
                     ('Full τ=5', full5)]:
        print(' ', speedup_stats(alt, d, label))

    # ── Best-tau per scene ────────────────────────────────────────────────
    tg_methods = {k: v for k, v in [('τ=3', tg3), ('τ=4', tg4), ('τ=5', tg5)] if v}
    if len(tg_methods) >= 2 and alt:
        all_scenes = set(alt)
        for m in tg_methods.values():
            all_scenes &= set(m)
        tg_wins = sum(
            1 for sc in all_scenes
            if min(m.get(sc, float('inf')) for m in tg_methods.values()) < alt[sc]
        )
        print(f"\n=== Best TG (any τ) vs ALT ({len(all_scenes)} scenes) ===")
        print(f"  TG wins: {tg_wins}/{len(all_scenes)} ({100*tg_wins//len(all_scenes)}%)")

        # Which tau wins per scene
        tau_wins = defaultdict(int)
        for sc in all_scenes:
            best_t = min(tg_methods.items(), key=lambda kv: kv[1].get(sc, float('inf')))
            tau_wins[best_t[0]] += 1
        print(f"  Best tau distribution: {dict(sorted(tau_wins.items()))}")

    # ── Top-10 speedup scenes (TG5 vs ALT) ───────────────────────────────
    if tg5 and alt:
        common = set(jps) & set(alt) & set(tg5)
        rows = sorted([(sc, jps.get(sc, 0), alt[sc], tg5[sc], alt[sc]/tg5[sc])
                       for sc in common], key=lambda x: -x[4])
        print("\n=== Top-15 Scenes by TG5 vs ALT Speedup ===")
        print(f"{'Scene':25s} {'JPS':8s} {'ALT':8s} {'TG5':8s} {'Speedup':10s}")
        for sc, j, a, t, sp in rows[:15]:
            print(f"{sc:25s} {j:.3f}ms {a:.3f}ms {t:.3f}ms {sp:.1f}x")

        # Speedup distribution
        dist = {'≥10x': 0, '5-10x': 0, '2-5x': 0, '1-2x': 0, '0.5-1x': 0, '<0.5x': 0}
        for _, _, a, t, sp in rows:
            if sp >= 10: dist['≥10x'] += 1
            elif sp >= 5: dist['5-10x'] += 1
            elif sp >= 2: dist['2-5x'] += 1
            elif sp >= 1: dist['1-2x'] += 1
            elif sp >= 0.5: dist['0.5-1x'] += 1
            else: dist['<0.5x'] += 1
        print(f"\n=== Speedup Distribution ({len(rows)} scenes) ===")
        for k, v in dist.items():
            bar = '█' * v
            print(f"  {k:8s}: {v:3d} ({100*v//len(rows):2d}%) {bar}")

    # ── LaTeX table for paper ─────────────────────────────────────────────
    print("\n=== LaTeX Table (Methods × Metrics) ===")
    methods = [
        ('JPS', jps, 'Surface A*'),
        ('flat', flat, 'Flat surface'),
        ('ALT', alt, 'ALT A* (16 lm)'),
        ('Full τ=5', full5, 'Full HPA* τ=5'),
        ('TG τ=3', tg3, 'PH-ALT-TG τ=3'),
        ('TG τ=4', tg4, 'PH-ALT-TG τ=4'),
        ('TG τ=5', tg5, 'PH-ALT-TG τ=5'),
    ]
    print(r"\begin{tabular}{lrrr}")
    print(r"\toprule")
    print(r"Method & Scenes & Median $T_s$ (ms) & vs. JPS \\")
    print(r"\midrule")
    jps_med = statistics.median(jps.values()) if jps else 1
    for _, d, name in methods:
        if not d:
            print(f"{name} & -- & -- & -- \\\\")
            continue
        med = statistics.median(d.values())
        n = len(d)
        ratio = jps_med / med
        bold = r'\textbf{' if 'TG' in _ else ''
        endbold = r'}' if 'TG' in _ else ''
        print(f"{bold}{name}{endbold} & {n} & {bold}{med:.3f}{endbold} & "
              f"{bold}{ratio:.1f}x{endbold} \\\\")
    print(r"\bottomrule")
    print(r"\end{tabular}")


if __name__ == '__main__':
    main()
