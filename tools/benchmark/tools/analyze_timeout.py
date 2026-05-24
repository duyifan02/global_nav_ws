#!/usr/bin/env python3
"""P7: Timeout/failure breakdown across resolutions.

Analyzes failure modes for each resolution to show that 0.05m failures
are mostly voxelization/connectivity issues affecting all methods equally.

Usage:
    python3 analyze_timeout.py \
      --results-dir experiments/results/hm3d_100 \
      --out experiments/results/analysis/timeout_breakdown.csv
"""
import argparse
import csv
import statistics
from collections import Counter, defaultdict
from pathlib import Path


def sf(v, default=0.0):
    try:
        return float(v)
    except (ValueError, TypeError):
        return default


def query_key(row: dict) -> str:
    return (f"{row.get('scene', '')}|"
            f"{row.get('start_x', '')}_{row.get('start_y', '')}_{row.get('start_z', '')}|"
            f"{row.get('goal_x', '')}_{row.get('goal_y', '')}_{row.get('goal_z', '')}")


def analyze_method(rows, label):
    """Analyze success/failure breakdown for one method."""
    total = len(rows)
    succ = [r for r in rows if r.get('success') == 'True']
    n_succ = len(succ)
    failed = [r for r in rows if r.get('success') != 'True']

    # Failure reason breakdown
    reasons = Counter()
    for r in failed:
        reason = r.get('failure_reason', '').strip()
        if not reason or reason == 'unknown':
            # Infer from available data
            if not r.get('planning_time_ms') or r.get('planning_time_ms') == 'None':
                reason = 'voxelization_or_map_failed'
            elif sf(r.get('topo_ms', 0)) == 0 and r.get('planner_mode') in ('ph_alt_tg', ''):
                reason = 'topo_build_failed'
            else:
                reason = 'query_failed'
        reasons[reason] += 1

    # Success stats
    ts_vals = [sf(r['planning_time_ms']) for r in succ if r.get('planning_time_ms')]
    pl_vals = [sf(r['path_length_m']) for r in succ if sf(r.get('path_length_m', 0)) > 0]

    # Path validity
    valid = [r for r in succ if r.get('path_valid') == 'True']
    invalid = [r for r in succ if r.get('path_valid') == 'False']

    # Scene-level failure
    scene_status = defaultdict(lambda: {'total': 0, 'succ': 0, 'fail': 0})
    for r in rows:
        s = r.get('scene', '')
        scene_status[s]['total'] += 1
        if r.get('success') == 'True':
            scene_status[s]['succ'] += 1
        else:
            scene_status[s]['fail'] += 1

    all_fail_scenes = sum(1 for v in scene_status.values() if v['succ'] == 0)
    partial_fail = sum(1 for v in scene_status.values() if 0 < v['fail'] < v['total'])

    return {
        'label': label,
        'total': total,
        'success': n_succ,
        'failed': len(failed),
        'success_rate': n_succ / total * 100 if total else 0,
        'valid': len(valid),
        'invalid': len(invalid),
        'valid_rate': len(valid) / n_succ * 100 if n_succ else 0,
        'reasons': reasons,
        'mean_ts_ms': statistics.mean(ts_vals) if ts_vals else 0,
        'p95_ts_ms': sorted(ts_vals)[min(int(len(ts_vals) * 0.95), len(ts_vals) - 1)] if ts_vals else 0,
        'mean_path_m': statistics.mean(pl_vals) if pl_vals else 0,
        'n_scenes': len(scene_status),
        'all_fail_scenes': all_fail_scenes,
        'partial_fail_scenes': partial_fail,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--results-dir', default='experiments/results/hm3d_100')
    ap.add_argument('--out', default='experiments/results/analysis/timeout_breakdown.csv')
    args = ap.parse_args()

    rd = Path(args.results_dir)

    # Discover result files
    configs = []
    for res, rl in [('01m', '0.1m'), ('0075m', '0.075m'), ('005m', '0.05m')]:
        for pattern, method_label in [
            (f'100_alt_{res}*.csv', f'ALT {rl}'),
            (f'100_tg_auto_k1_{res}*.csv', f'TG K=1 {rl}'),
            (f'100_tg_auto_k5_{res}*.csv', f'TG K=5 {rl}'),
            (f'100_tg_auto_{res}*.csv', f'TG auto {rl}'),
        ]:
            matches = sorted(rd.glob(pattern))
            # Prefer _500q files
            preferred = [m for m in matches if '500q' in m.name]
            if preferred:
                matches = preferred
            if matches:
                configs.append((matches[0], method_label, rl))

    if not configs:
        print("No result files found!")
        return

    results = []
    for path, label, res in configs:
        with open(path, newline='') as f:
            rows = list(csv.DictReader(f))
        r = analyze_method(rows, label)
        r['resolution'] = res
        r['file'] = path.name
        results.append(r)

    # Print summary
    print(f"\n{'=' * 100}")
    print(f"  Timeout/Failure Breakdown — HM3D 100 scenes")
    print(f"{'=' * 100}")
    hdr = f"{'Method':<22} {'Res':>5} {'Total':>5} {'Succ':>5} {'Fail':>5} {'Rate':>6} {'Valid%':>6} {'Ts(ms)':>8} {'AllFail':>7}"
    print(hdr)
    print('-' * len(hdr))
    for r in results:
        print(f"{r['label']:<22} {r['resolution']:>5} {r['total']:>5} "
              f"{r['success']:>5} {r['failed']:>5} {r['success_rate']:5.1f}% "
              f"{r['valid_rate']:5.1f}% {r['mean_ts_ms']:8.3f} {r['all_fail_scenes']:>7}")

    # Failure reason breakdown per resolution
    for res in ['0.1m', '0.075m', '0.05m']:
        res_results = [r for r in results if r['resolution'] == res]
        if not res_results:
            continue
        print(f"\n  Failure reasons at {res}:")
        for r in res_results:
            if r['reasons']:
                print(f"    {r['label']}:")
                for reason, count in r['reasons'].most_common():
                    print(f"      {reason:<35} {count:>4} ({count/r['total']*100:.1f}%)")

    # Cross-resolution comparison
    print(f"\n{'=' * 80}")
    print(f"  Cross-Resolution Failure Correlation")
    print(f"{'=' * 80}")

    # For each resolution pair, check if the same queries fail
    for res_a, res_b in [('0.1m', '0.075m'), ('0.1m', '0.05m'), ('0.075m', '0.05m')]:
        methods_a = {r['label']: r for r in results if r['resolution'] == res_a}
        methods_b = {r['label']: r for r in results if r['resolution'] == res_b}
        # Try to match ALT methods
        alt_a = next((c for c, l, r in configs if r == res_a and 'ALT' in l), None)
        alt_b = next((c for c, l, r in configs if r == res_b and 'ALT' in l), None)
        if alt_a and alt_b:
            with open(alt_a, newline='') as f:
                rows_a = list(csv.DictReader(f))
            with open(alt_b, newline='') as f:
                rows_b = list(csv.DictReader(f))

            fail_a = {r.get('scene') for r in rows_a if r.get('success') != 'True'}
            fail_b = {r.get('scene') for r in rows_b if r.get('success') != 'True'}
            both = fail_a & fail_b
            print(f"  ALT: {res_a} fails={len(fail_a)} scenes, {res_b} fails={len(fail_b)} scenes, both={len(both)}")

    # Write CSV
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    csv_rows = []
    for r in results:
        csv_rows.append({
            'method': r['label'],
            'resolution': r['resolution'],
            'total_queries': r['total'],
            'success': r['success'],
            'failed': r['failed'],
            'success_rate': f"{r['success_rate']:.1f}",
            'valid_paths': r['valid'],
            'invalid_paths': r['invalid'],
            'valid_rate': f"{r['valid_rate']:.1f}",
            'mean_ts_ms': f"{r['mean_ts_ms']:.4f}",
            'p95_ts_ms': f"{r['p95_ts_ms']:.4f}",
            'n_scenes': r['n_scenes'],
            'all_fail_scenes': r['all_fail_scenes'],
            'partial_fail_scenes': r['partial_fail_scenes'],
            'failure_reasons': '; '.join(f'{k}={v}' for k, v in r['reasons'].most_common()),
        })

    with open(args.out, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(csv_rows[0].keys()))
        w.writeheader()
        w.writerows(csv_rows)
    print(f"\n  Written: {args.out}")


if __name__ == '__main__':
    main()
