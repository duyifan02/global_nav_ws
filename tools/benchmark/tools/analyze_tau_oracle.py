#!/usr/bin/env python3
"""P4.2: Oracle tau analysis — compare auto-tau vs per-scene optimal tau.

Reads fixed-tau sweep results + auto-tau results, computes oracle tau
per scene, and evaluates auto-tau selector accuracy.

Usage:
    python3 analyze_tau_oracle.py \
      --alt experiments/results/hm3d_100/100_alt_01m_500q.csv \
      --sweep experiments/results/hm3d_100/100_tg_tau3_k5_01m_500q.csv \
              experiments/results/hm3d_100/100_tg_tau4_k5_01m_500q.csv \
              experiments/results/hm3d_100/100_tg_tau5_k5_01m_500q.csv \
              experiments/results/hm3d_100/100_tg_tau6_k5_01m_500q.csv \
      --auto experiments/results/hm3d_100/100_tg_auto_k5_01m_500q.csv \
      --out experiments/results/analysis/tau_oracle.csv
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


def query_key(row: dict) -> str:
    return (f"{row.get('scene', '')}|"
            f"{row.get('start_x')}_{row.get('start_y')}_{row.get('start_z')}|"
            f"{row.get('goal_x')}_{row.get('goal_y')}_{row.get('goal_z')}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--alt', required=True, help='ALT baseline CSV')
    ap.add_argument('--sweep', nargs='+', required=True, help='Fixed-tau CSVs')
    ap.add_argument('--auto', required=True, help='Auto-tau CSV')
    ap.add_argument('--out', default='experiments/results/analysis/tau_oracle.csv')
    args = ap.parse_args()

    # Load ALT
    with open(args.alt) as f:
        alt_rows = list(csv.DictReader(f))
    alt_by_key = {}
    for r in alt_rows:
        if r.get('success') == 'True' and sf(r.get('path_length_m')) > 0:
            alt_by_key[query_key(r)] = sf(r['path_length_m'])

    # Load fixed-tau sweeps
    sweep_data = {}  # tau -> {scene -> {key -> row}}
    for path in args.sweep:
        with open(path) as f:
            rows = list(csv.DictReader(f))
        if not rows:
            continue
        tau = sf(rows[0].get('selected_tau', 0))
        if tau == 0:
            # Try to infer from filename
            import re
            m = re.search(r'tau(\d+)', path)
            tau = float(m.group(1)) if m else 0
        sweep_data[tau] = defaultdict(dict)
        for r in rows:
            sweep_data[tau][r.get('scene', '')][query_key(r)] = r

    # Load auto-tau
    with open(args.auto) as f:
        auto_rows = list(csv.DictReader(f))
    auto_by_scene = defaultdict(list)
    for r in auto_rows:
        auto_by_scene[r.get('scene', '')].append(r)

    taus = sorted(sweep_data.keys())
    print(f"Tau candidates: {taus}")
    print(f"ALT queries: {len(alt_by_key)}")
    print(f"Auto-tau scenes: {len(auto_by_scene)}")

    # Per-scene analysis
    results = []
    for scene in sorted(auto_by_scene.keys()):
        auto_scene_rows = auto_by_scene[scene]
        auto_tau = sf(auto_scene_rows[0].get('selected_tau', 0))

        # Per-tau metrics for this scene
        tau_metrics = {}
        for tau in taus:
            scene_rows = sweep_data.get(tau, {}).get(scene, {})
            if not scene_rows:
                continue
            succ = [r for r in scene_rows.values()
                    if r.get('success') == 'True' and sf(r.get('path_length_m')) > 0]
            if not succ:
                continue

            ts_vals = [sf(r['planning_time_ms']) for r in succ]
            # Paired overhead
            ohs = []
            for r in succ:
                k = query_key(r)
                if k in alt_by_key:
                    ohs.append((sf(r['path_length_m']) / alt_by_key[k] - 1) * 100)

            mean_ts = statistics.mean(ts_vals)
            mean_oh = statistics.mean(ohs) if ohs else 999
            tau_metrics[tau] = {
                'mean_ts': mean_ts,
                'mean_oh': mean_oh,
                'success': len(succ),
                'total': len(scene_rows),
            }

        if not tau_metrics:
            continue

        # Oracle speed: min query time among taus with OH <= 10%
        valid_speed = {t: m for t, m in tau_metrics.items() if m['mean_oh'] <= 10}
        oracle_speed_tau = min(valid_speed, key=lambda t: valid_speed[t]['mean_ts']) if valid_speed else min(tau_metrics, key=lambda t: tau_metrics[t]['mean_ts'])

        # Oracle quality: min OH among taus
        oracle_quality_tau = min(tau_metrics, key=lambda t: tau_metrics[t]['mean_oh'])

        # Auto-tau metrics
        auto_succ = [r for r in auto_scene_rows
                     if r.get('success') == 'True' and sf(r.get('path_length_m')) > 0]
        auto_ts = statistics.mean([sf(r['planning_time_ms']) for r in auto_succ]) if auto_succ else 0
        auto_ohs = []
        for r in auto_succ:
            k = query_key(r)
            if k in alt_by_key:
                auto_ohs.append((sf(r['path_length_m']) / alt_by_key[k] - 1) * 100)
        auto_oh = statistics.mean(auto_ohs) if auto_ohs else 999

        results.append({
            'scene': scene,
            'auto_tau': auto_tau,
            'oracle_speed_tau': oracle_speed_tau,
            'oracle_quality_tau': oracle_quality_tau,
            'auto_matches_speed': auto_tau == oracle_speed_tau,
            'auto_matches_quality': auto_tau == oracle_quality_tau,
            'auto_ts_ms': f"{auto_ts:.4f}",
            'oracle_speed_ts_ms': f"{tau_metrics.get(oracle_speed_tau, {}).get('mean_ts', 0):.4f}",
            'auto_oh_pct': f"{auto_oh:.2f}",
            'oracle_quality_oh_pct': f"{tau_metrics.get(oracle_quality_tau, {}).get('mean_oh', 0):.2f}",
            **{f'tau{int(t)}_ts': f"{m['mean_ts']:.4f}" for t, m in tau_metrics.items()},
            **{f'tau{int(t)}_oh': f"{m['mean_oh']:.2f}" for t, m in tau_metrics.items()},
        })

    # Write output
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    if results:
        with open(args.out, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(results[0].keys()))
            w.writeheader()
            w.writerows(results)
        print(f"\nWritten {len(results)} scenes to {args.out}")

    # Summary
    speed_match = sum(1 for r in results if r['auto_matches_speed'])
    quality_match = sum(1 for r in results if r['auto_matches_quality'])
    n = len(results)
    print(f"\n{'='*60}")
    print(f"  Oracle Tau Summary ({n} scenes)")
    print(f"{'='*60}")
    print(f"  Auto == Oracle(speed):   {speed_match}/{n} ({speed_match/n*100:.1f}%)")
    print(f"  Auto == Oracle(quality): {quality_match}/{n} ({quality_match/n*100:.1f}%)")

    auto_ts_vals = [sf(r['auto_ts_ms']) for r in results if sf(r['auto_ts_ms']) > 0]
    oracle_ts_vals = [sf(r['oracle_speed_ts_ms']) for r in results if sf(r['oracle_speed_ts_ms']) > 0]
    if auto_ts_vals and oracle_ts_vals:
        print(f"  Auto mean Ts:   {statistics.mean(auto_ts_vals):.4f} ms")
        print(f"  Oracle mean Ts: {statistics.mean(oracle_ts_vals):.4f} ms")
        print(f"  Speed loss:     {(statistics.mean(auto_ts_vals)/statistics.mean(oracle_ts_vals)-1)*100:+.1f}%")

    auto_oh_vals = [sf(r['auto_oh_pct']) for r in results if sf(r['auto_oh_pct']) < 900]
    oracle_oh_vals = [sf(r['oracle_quality_oh_pct']) for r in results if sf(r['oracle_quality_oh_pct']) < 900]
    if auto_oh_vals and oracle_oh_vals:
        print(f"  Auto mean OH:   {statistics.mean(auto_oh_vals):+.2f}%")
        print(f"  Oracle mean OH: {statistics.mean(oracle_oh_vals):+.2f}%")

    # Tau distribution
    auto_taus = [sf(r['auto_tau']) for r in results]
    from collections import Counter
    tau_dist = Counter(auto_taus)
    print(f"\n  Auto-tau distribution:")
    for t in sorted(tau_dist):
        print(f"    tau={t:.0f}: {tau_dist[t]} scenes ({tau_dist[t]/n*100:.1f}%)")


if __name__ == '__main__':
    main()
