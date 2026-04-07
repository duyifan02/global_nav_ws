#!/usr/bin/env python3
"""
Generate benchmark_sweep.csv: TopoNav rows at target resolutions.
Copies scene/obj/seed/start/goal from existing JPS_3D rows, changes planner
and resolution, clears result columns.
"""
import csv, sys, os

def main():
    src = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        os.path.dirname(__file__), '../../../../benchmark.csv')
    out = sys.argv[2] if len(sys.argv) > 2 else os.path.join(
        os.path.dirname(__file__), '../../../../benchmark_sweep.csv')
    resolutions = [float(x) for x in sys.argv[3].split(',')] if len(sys.argv) > 3 else [0.1, 0.2]

    RESULT_COLS = ['planning_time_ms','success','planning_time_std_ms',
                   'path_length_m','ns_visited_nodes','tp_ms','te_ms',
                   'topo_ms','memory_usage_bytes','total_time_ms','trials_count']
    KEY_COLS = ['scene','obj_path','seed_x','seed_y','seed_z',
                'start_x','start_y','start_z','goal_x','goal_y','goal_z']

    with open(src) as f:
        rows = list(csv.DictReader(f))

    jps_rows = [r for r in rows if r['planner'] == 'JPS_3D' and r['scene'] == 'spiral']

    fieldnames = rows[0].keys() if rows else []
    out_rows = []
    for res in resolutions:
        for r in jps_rows:
            new = dict(r)
            new['planner'] = 'TopoNav'
            new['resolution'] = str(res)
            new['timestamp'] = '20260331_000000'
            for c in RESULT_COLS:
                new[c] = ''
            out_rows.append(new)

    with open(out, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(out_rows)

    print(f"Written {len(out_rows)} rows to {out}")
    for r in out_rows:
        print(f"  {r['planner']} res={r['resolution']} {os.path.basename(r['obj_path'])[:20]}")

if __name__ == '__main__':
    main()
