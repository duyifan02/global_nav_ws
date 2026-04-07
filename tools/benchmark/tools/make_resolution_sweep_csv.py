#!/usr/bin/env python3
"""
从已有的 0.1m benchmark rows 生成不同分辨率的 input CSV。
用法：
    python3 make_resolution_sweep_csv.py \
        --template benchmark.csv \
        --resolution 0.05 \
        --output benchmark_0.05m_input.csv
"""
import argparse, csv
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--template', required=True, help='已有的 benchmark CSV（含 obj_path / seed / start / goal）')
    parser.add_argument('--resolution', type=float, required=True)
    parser.add_argument('--planners', nargs='+', default=['JPS_3D', 'TopoNav'])
    parser.add_argument('--output', required=True)
    args = parser.parse_args()

    rows = list(csv.DictReader(open(args.template)))
    fieldnames = rows[0].keys()

    seen_scenes = {}  # scan_id → representative row (one per scan)
    for r in rows:
        obj = r.get('obj_path', '').strip()
        if not obj or obj.lower() == 'none':
            continue
        # Extract scan_id from .../scans/{scan_id}/...
        parts = Path(obj).parts
        scan_id = None
        for i, p in enumerate(parts):
            if p == 'scans' and i + 1 < len(parts):
                scan_id = parts[i + 1]
                break
        if scan_id and scan_id not in seen_scenes:
            seen_scenes[scan_id] = r

    out_rows = []
    for scene, template_row in seen_scenes.items():
        for planner in args.planners:
            new_row = dict(template_row)
            new_row['planner'] = planner
            new_row['resolution'] = str(args.resolution)
            # Clear all result fields
            for col in ['planning_time_ms', 'success', 'planning_time_std_ms',
                        'path_length_m', 'ns_visited_nodes', 'tp_ms', 'te_ms',
                        'topo_ms', 'memory_usage_bytes', 'total_time_ms', 'trials_count']:
                new_row[col] = ''
            out_rows.append(new_row)

    with open(args.output, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        w.writerows(out_rows)

    print(f'写入 {len(out_rows)} 行 → {args.output}')
    for r in out_rows:
        print(f"  {r['planner']:10s} {r['scene']:20s} res={r['resolution']}")


if __name__ == '__main__':
    main()
