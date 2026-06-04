#!/usr/bin/env python3
"""Analyze HM3D tuning results (27 configs, 10 minival scenes, start-as-seed)."""
import csv
import os
import sys
from collections import defaultdict

TUNING_DIR = "experiments/results/hm3d_tuning"

def load_csv(path):
    rows = []
    with open(path) as f:
        reader = csv.DictReader(f)
        for r in reader:
            rows.append(r)
    return rows

def analyze_method(rows, label):
    """Compute per-scene and overall stats for one method."""
    scenes = defaultdict(list)
    for r in rows:
        scene = r["scene"]
        ok = r["success"] == "True"
        ts = float(r["planning_time_ms"]) if ok else None
        pl = float(r["path_length_m"]) if ok and r["path_length_m"] else None
        topo = float(r["topo_ms"]) if r.get("topo_ms") and r["topo_ms"] else 0
        fallback = int(r["ns_visited_nodes"]) if ok and r.get("ns_visited_nodes") else 0
        scenes[scene].append({"ok": ok, "ts": ts, "pl": pl, "topo": topo, "fallback": fallback})

    scene_stats = {}
    for scene, trials in scenes.items():
        ok_trials = [t for t in trials if t["ok"]]
        n_ok = len(ok_trials)
        n_total = len(trials)
        if n_ok > 0:
            mean_ts = sum(t["ts"] for t in ok_trials) / n_ok
            mean_pl = sum(t["pl"] for t in ok_trials if t["pl"]) / max(1, len([t for t in ok_trials if t["pl"]]))
            max_fb = max(t["fallback"] for t in ok_trials)
            mean_topo = sum(t["topo"] for t in ok_trials) / n_ok
        else:
            mean_ts = mean_pl = max_fb = mean_topo = None
        scene_stats[scene] = {
            "n_ok": n_ok, "n_total": n_total,
            "mean_ts": mean_ts, "mean_pl": mean_pl,
            "max_fallback": max_fb, "mean_topo": mean_topo
        }

    # Overall (success scenes only)
    ok_scenes = {s: v for s, v in scene_stats.items() if v["mean_ts"] is not None}
    if ok_scenes:
        overall_ts = sum(v["mean_ts"] for v in ok_scenes.values()) / len(ok_scenes)
        overall_pl = sum(v["mean_pl"] for v in ok_scenes.values() if v["mean_pl"]) / max(1, len([v for v in ok_scenes.values() if v["mean_pl"]]))
        total_ok = sum(v["n_ok"] for v in scene_stats.values())
        total_n = sum(v["n_total"] for v in scene_stats.values())
        max_fb = max(v["max_fallback"] for v in ok_scenes.values() if v["max_fallback"] is not None)
    else:
        overall_ts = overall_pl = max_fb = None
        total_ok = 0
        total_n = sum(v["n_total"] for v in scene_stats.values())

    return {
        "label": label,
        "scene_stats": scene_stats,
        "overall_ts": overall_ts,
        "overall_pl": overall_pl,
        "total_ok": total_ok,
        "total_n": total_n,
        "max_fallback": max_fb,
    }

def main():
    results = {}

    # Map filenames to labels
    files = {}
    for fname in sorted(os.listdir(TUNING_DIR)):
        if not fname.endswith(".csv"):
            continue
        base = fname.replace(".csv", "")
        files[base] = os.path.join(TUNING_DIR, fname)

    for base, path in sorted(files.items()):
        rows = load_csv(path)
        stats = analyze_method(rows, base)
        results[base] = stats

    # Group by resolution
    by_res = defaultdict(dict)
    for base, stats in results.items():
        if "_005m" in base:
            res = "0.05m"
        elif "_0075m" in base:
            res = "0.075m"
        elif "_01m" in base:
            res = "0.1m"
        else:
            res = "unknown"
        by_res[res][base] = stats

    for res in ["0.1m", "0.075m", "0.05m"]:
        if res not in by_res:
            continue
        group = by_res[res]
        print(f"\n{'='*80}")
        print(f"  Resolution: {res}")
        print(f"{'='*80}")

        # Find baselines
        jps_key = [k for k in group if k.startswith("jps_")]
        flat_key = [k for k in group if k.startswith("flat_")]
        alt_key = [k for k in group if k.startswith("alt_")]

        jps_ts = group[jps_key[0]]["overall_ts"] if jps_key and group[jps_key[0]]["overall_ts"] else None
        flat_ts = group[flat_key[0]]["overall_ts"] if flat_key and group[flat_key[0]]["overall_ts"] else None
        alt_ts = group[alt_key[0]]["overall_ts"] if alt_key and group[alt_key[0]]["overall_ts"] else None
        alt_pl = group[alt_key[0]]["overall_pl"] if alt_key and group[alt_key[0]]["overall_pl"] else None

        print(f"\n{'Method':<25} {'Ts(ms)':>8} {'Path(m)':>8} {'OK':>6} {'MaxFB':>8} {'vs JPS':>8} {'vs flat':>8} {'vs ALT':>8} {'Path%':>8}")
        print("-" * 105)

        for base in sorted(group.keys()):
            s = group[base]
            ts = s["overall_ts"]
            pl = s["overall_pl"]
            ok = f"{s['total_ok']}/{s['total_n']}"
            fb = s["max_fallback"]
            if ts is not None:
                vs_jps = f"{jps_ts/ts:.1f}x" if jps_ts else "-"
                vs_flat = f"{flat_ts/ts:.1f}x" if flat_ts else "-"
                vs_alt = f"{alt_ts/ts:.1f}x" if alt_ts else "-"
                path_oh = f"{(pl/alt_pl-1)*100:+.1f}%" if alt_pl and pl else "-"
                print(f"{base:<25} {ts:>8.3f} {pl:>8.1f} {ok:>6} {fb:>8} {vs_jps:>8} {vs_flat:>8} {vs_alt:>8} {path_oh:>8}")
            else:
                print(f"{base:<25} {'FAIL':>8} {'-':>8} {ok:>6} {'-':>8} {'-':>8} {'-':>8} {'-':>8} {'-':>8}")

    # Per-scene breakdown for best config at 0.1m
    print(f"\n{'='*80}")
    print(f"  Per-Scene Breakdown at 0.1m (best TG configs)")
    print(f"{'='*80}")

    res_01 = by_res.get("0.1m", {})
    # Get all scenes from JPS
    jps_data = res_01.get("jps_01m")
    if jps_data:
        all_scenes = sorted(jps_data["scene_stats"].keys())
        # Show: JPS, flat, ALT, then all TG configs
        methods = ["jps_01m", "flat_01m", "alt_01m"] + sorted([k for k in res_01 if k.startswith("tg_")])

        print(f"\n{'Scene':<25}", end="")
        for m in methods:
            short = m.replace("_01m", "")
            print(f" {short:>10}", end="")
        print()
        print("-" * (25 + 11 * len(methods)))

        for scene in all_scenes:
            short_scene = scene.replace("hm3d_", "")[:15]
            print(f"{short_scene:<25}", end="")
            for m in methods:
                if m in res_01:
                    ss = res_01[m]["scene_stats"].get(scene)
                    if ss and ss["mean_ts"] is not None:
                        print(f" {ss['mean_ts']:>10.3f}", end="")
                    else:
                        print(f" {'FAIL':>10}", end="")
                else:
                    print(f" {'-':>10}", end="")
            print()

    # Landmark ablation at 0.1m
    print(f"\n{'='*80}")
    print(f"  Landmark Ablation at 0.1m (tau=4)")
    print(f"{'='*80}")
    lm_keys = sorted([k for k in res_01 if "lm" in k])
    if lm_keys:
        print(f"\n{'Config':<25} {'Ts(ms)':>8} {'Path(m)':>8} {'OK':>6} {'MaxFB':>8} {'vs ALT':>8}")
        print("-" * 65)
        for k in ["alt_01m"] + lm_keys + ["tg_tau4_01m"]:
            if k in res_01:
                s = res_01[k]
                ts = s["overall_ts"]
                pl = s["overall_pl"]
                ok = f"{s['total_ok']}/{s['total_n']}"
                fb = s["max_fallback"]
                vs_alt = f"{alt_ts/ts:.1f}x" if alt_ts and ts else "-"
                print(f"{k:<25} {ts:>8.3f} {pl:>8.1f} {ok:>6} {fb:>8} {vs_alt:>8}")

if __name__ == "__main__":
    os.chdir("/data2/dyf/papers/Beyond_Geometry/global_nav_ws")
    main()
