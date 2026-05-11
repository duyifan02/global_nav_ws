#!/usr/bin/env python3
"""
Aggregate benchmark results and generate LaTeX tables.

Reads results CSVs, computes per-scene and cross-scene statistics
(mean, median, p95), generates LaTeX tables with consistent S1-S6 naming.

Usage:
    python3 aggregate_results.py --csvs results1.csv results2.csv \
        --output-tex tables.tex --output-csv summary.csv
"""

import argparse
import csv
import sys
from pathlib import Path
from typing import List, Dict

import numpy as np

# Canonical scene ID mapping
SCENE_MAP = {
    '1LXtFkjw3qL': 'S1',
    '1pXnuDYAj8r': 'S2',
    '2azQ1b91cZZ': 'S3',
    '2n8kARJN3HM': 'S4',
    '2t7WUuJeko7': 'S5',
    '5LpN3gDmAk7': 'S6',
}


def scene_id(scene_name: str) -> str:
    """Map scene hash to canonical S1-S6 ID."""
    for key, sid in SCENE_MAP.items():
        if key in scene_name:
            return sid
    return scene_name


def load_results(csv_paths: List[str]) -> List[dict]:
    """Load and merge rows from multiple CSV files."""
    rows = []
    for p in csv_paths:
        with open(p, newline='') as f:
            for r in csv.DictReader(f):
                r['_source'] = p
                r['scene_id'] = scene_id(r.get('scene', ''))
                rows.append(r)
    return rows


def safe_float(v, default=None):
    try:
        if v is None or str(v).strip().lower() in ('', 'none', 'nan'):
            return default
        return float(v)
    except (ValueError, TypeError):
        return default


def compute_stats(values: List[float]) -> dict:
    """Compute mean, median, std, p95, min, max."""
    if not values:
        return {}
    arr = np.array(values)
    return {
        'mean': float(np.mean(arr)),
        'median': float(np.median(arr)),
        'std': float(np.std(arr)),
        'p95': float(np.percentile(arr, 95)),
        'min': float(np.min(arr)),
        'max': float(np.max(arr)),
        'count': len(values),
    }


def group_by(rows: List[dict], key_fn) -> Dict[str, List[dict]]:
    groups = {}
    for r in rows:
        k = key_fn(r)
        groups.setdefault(k, []).append(r)
    return groups


def generate_comparison_table(rows: List[dict]) -> str:
    """Generate LaTeX comparison table: rows=scenes, cols=planners."""
    by_planner = group_by(rows, lambda r: r.get('planner', '') + '|' + r.get('planner_mode', 'full'))

    # Collect all scene IDs
    all_scenes = sorted(set(r['scene_id'] for r in rows))

    lines = []
    lines.append(r'\begin{table}[h]')
    lines.append(r'\centering')
    lines.append(r'\caption{Planning performance comparison across all scenes}')

    planner_names = sorted(by_planner.keys())
    ncols = len(planner_names)
    col_spec = 'l' + 'r' * ncols * 3  # Ts, Path, Nodes for each planner
    lines.append(r'\begin{tabular}{l' + '|rrr' * ncols + '}')
    lines.append(r'\toprule')

    # Header
    header = 'Scene'
    for pname in planner_names:
        short = pname.replace('|full', '').replace('|', '-')
        header += f' & \\multicolumn{{3}}{{c}}{{{short}}}'
    lines.append(header + r' \\')

    subheader = ''
    for _ in planner_names:
        subheader += ' & $T_s$ (ms) & Path (m) & Nodes'
    lines.append(subheader + r' \\')
    lines.append(r'\midrule')

    # Data rows
    for sid in all_scenes:
        row_str = sid
        for pname in planner_names:
            scene_rows = [r for r in by_planner[pname] if r['scene_id'] == sid]
            ok_rows = [r for r in scene_rows if r.get('success') == 'True']
            if ok_rows:
                ts = safe_float(ok_rows[0].get('planning_time_ms'), 0)
                path = safe_float(ok_rows[0].get('path_length_m'), 0)
                nodes = safe_float(ok_rows[0].get('ns_visited_nodes'), 0)
                row_str += f' & {ts:.2f} & {path:.2f} & {int(nodes)}'
            else:
                row_str += ' & -- & -- & --'
        lines.append(row_str + r' \\')

    # Summary stats
    lines.append(r'\midrule')
    for stat_name in ['Mean', 'Median', 'P95']:
        row_str = stat_name
        for pname in planner_names:
            ok_rows = [r for r in by_planner[pname] if r.get('success') == 'True']
            ts_vals = [safe_float(r.get('planning_time_ms')) for r in ok_rows
                       if safe_float(r.get('planning_time_ms')) is not None]
            path_vals = [safe_float(r.get('path_length_m')) for r in ok_rows
                         if safe_float(r.get('path_length_m')) is not None]
            node_vals = [safe_float(r.get('ns_visited_nodes')) for r in ok_rows
                         if safe_float(r.get('ns_visited_nodes')) is not None]
            fn = {'Mean': np.mean, 'Median': np.median,
                  'P95': lambda x: np.percentile(x, 95)}[stat_name]
            ts_s = f'{fn(ts_vals):.2f}' if ts_vals else '--'
            path_s = f'{fn(path_vals):.2f}' if path_vals else '--'
            node_s = f'{int(fn(node_vals))}' if node_vals else '--'
            row_str += f' & {ts_s} & {path_s} & {node_s}'
        lines.append(row_str + r' \\')

    lines.append(r'\bottomrule')
    lines.append(r'\end{tabular}')
    lines.append(r'\end{table}')

    return '\n'.join(lines)


def generate_summary_csv(rows: List[dict], output_path: str):
    """Write per-scene summary CSV with all metrics."""
    summary = []
    for r in rows:
        summary.append({
            'scene_id': r['scene_id'],
            'scene': r.get('scene', ''),
            'planner': r.get('planner', ''),
            'planner_mode': r.get('planner_mode', 'full'),
            'resolution': r.get('resolution', ''),
            'success': r.get('success', ''),
            'planning_time_ms': r.get('planning_time_ms', ''),
            'path_length_m': r.get('path_length_m', ''),
            'ns_visited_nodes': r.get('ns_visited_nodes', ''),
            'topo_ms': r.get('topo_ms', ''),
            'te_ms': r.get('te_ms', ''),
            'memory_usage_bytes': r.get('memory_usage_bytes', ''),
            'total_time_ms': r.get('total_time_ms', ''),
            'num_rooms': r.get('num_rooms', ''),
        })

    if summary:
        cols = list(summary[0].keys())
        with open(output_path, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=cols)
            w.writeheader()
            w.writerows(summary)


def main():
    parser = argparse.ArgumentParser(description='Aggregate benchmark results')
    parser.add_argument('--csvs', nargs='+', required=True, help='Result CSV files')
    parser.add_argument('--output-tex', default='comparison_table.tex')
    parser.add_argument('--output-csv', default='summary.csv')
    args = parser.parse_args()

    rows = load_results(args.csvs)
    print(f'Loaded {len(rows)} rows from {len(args.csvs)} files')

    # Generate LaTeX table
    tex = generate_comparison_table(rows)
    with open(args.output_tex, 'w') as f:
        f.write(tex)
    print(f'LaTeX table written to {args.output_tex}')

    # Generate summary CSV
    generate_summary_csv(rows, args.output_csv)
    print(f'Summary CSV written to {args.output_csv}')

    # Print quick stats
    by_planner = group_by(rows, lambda r: r.get('planner', '') + '|' + r.get('planner_mode', 'full'))
    for pname, prows in sorted(by_planner.items()):
        ok = [r for r in prows if r.get('success') == 'True']
        ts = [safe_float(r.get('planning_time_ms')) for r in ok
              if safe_float(r.get('planning_time_ms')) is not None]
        paths = [safe_float(r.get('path_length_m')) for r in ok
                 if safe_float(r.get('path_length_m')) is not None]
        print(f'\n{pname}: {len(ok)}/{len(prows)} success')
        if ts:
            print(f'  Ts: mean={np.mean(ts):.2f}, median={np.median(ts):.2f}, '
                  f'p95={np.percentile(ts, 95):.2f} ms')
        if paths:
            print(f'  Path: mean={np.mean(paths):.2f}, median={np.median(paths):.2f} m')


if __name__ == '__main__':
    main()
