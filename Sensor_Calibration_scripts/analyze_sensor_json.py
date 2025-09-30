#!/usr/bin/env python3
"""
analyze_sensor_json.py
Read one or more JSON files (produced by your RAW frame analyzer) and compute:
- Black level averages and standard deviations per channel (R, G1, G2, B) and "used"
- White balance averages and standard deviations per channel (R, G1, G2, B)

Usage:
  python3 analyze_sensor_json.py /path/to/*.json
  python3 analyze_sensor_json.py file1.json file2.json --save-csv results.csv

Notes:
- Comments are in English by request.
- Units: DN [digital numbers] for black level; dimensionless scale for white balance.
"""

import argparse
import json
import math
import os
import statistics
import sys
from typing import Dict, List, Tuple

def load_json(path: str) -> dict:
    with open(path, "r") as f:
        return json.load(f)

def safe_get(dct: dict, path: List[str], default=None):
    cur = dct
    for k in path:
        if not isinstance(cur, dict) or k not in cur:
            return default
        cur = cur[k]
    return cur

def collect_values(files: List[str]) -> Tuple[List[Dict], List[Dict]]:
    """Return lists with per-file black level and white balance dicts."""
    bl_rows, wb_rows = [], []
    for f in files:
        try:
            j = load_json(f)
        except Exception as e:
            print(f"ERROR: failed to read '{f}': {e}", file=sys.stderr)
            continue

        # Black level (expected keys: Black -> 'black_level_estimates' with 'R','G1','G2','B','used')
        bl = safe_get(j, ["black_level_estimates"], {})
        bl_row = {
            "file": os.path.basename(f),
            "R": bl.get("R"),
            "G1": bl.get("G1"),
            "G2": bl.get("G2"),
            "B": bl.get("B"),
            "used": bl.get("used"),
        }
        bl_rows.append(bl_row)

        # White balance (expected keys: 'white_balance' with 'r','g1','g2','b')
        wb = safe_get(j, ["white_balance"], {})
        wb_row = {
            "file": os.path.basename(f),
            "R": wb.get("r"),
            "G1": wb.get("g1"),
            "G2": wb.get("g2"),
            "B": wb.get("b"),
        }
        wb_rows.append(wb_row)

    return bl_rows, wb_rows

def mean_std(vals: List[float]) -> Tuple[float, float]:
    vals = [v for v in vals if v is not None and not (isinstance(v, float) and math.isnan(v))]
    if not vals:
        return float("nan"), float("nan")
    if len(vals) == 1:
        return float(vals[0]), 0.0
    return statistics.mean(vals), statistics.stdev(vals)

def summarize(rows: List[Dict], keys: List[str]) -> Dict[str, Tuple[float, float]]:
    out = {}
    for k in keys:
        avg, std = mean_std([r.get(k) for r in rows])
        out[k] = (avg, std)
    return out

def format_summary(title: str, summary: Dict[str, Tuple[float, float]], units: str = "") -> str:
    lines = [title]
    for ch in ["R", "G1", "G2", "B"] + (["used"] if "used" in summary else []):
        if ch in summary:
            avg, std = summary[ch]
            if units:
                lines.append(f"- {ch}: {avg:.3f} {units} ± {std:.3f}")
            else:
                lines.append(f"- {ch}: {avg:.3f} ± {std:.3f}")
    return "\n".join(lines)

def maybe_save_csv(path: str, bl_rows: List[Dict], wb_rows: List[Dict], bl_sum: Dict, wb_sum: Dict):
    try:
        import csv
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            # Per-file black level
            w.writerow(["# Per-file Black Level [DN]"])
            w.writerow(["file", "R", "G1", "G2", "B", "used"])
            for r in bl_rows:
                w.writerow([r["file"], r["R"], r["G1"], r["G2"], r["B"], r["used"]])
            # Per-file white balance
            w.writerow([]); w.writerow(["# Per-file White Balance [scale]"])
            w.writerow(["file", "R", "G1", "G2", "B"])
            for r in wb_rows:
                w.writerow([r["file"], r["R"], r["G1"], r["G2"], r["B"]])
            # Summary
            w.writerow([]); w.writerow(["# Summary Black Level [DN] (avg ± std)"])
            for ch in ["R","G1","G2","B","used"]:
                if ch in bl_sum:
                    a,s = bl_sum[ch]
                    w.writerow([ch, f"{a:.3f}", f"{s:.3f}"])
            w.writerow([]); w.writerow(["# Summary White Balance [scale] (avg ± std)"])
            for ch in ["R","G1","G2","B"]:
                if ch in wb_sum:
                    a,s = wb_sum[ch]
                    w.writerow([ch, f"{a:.3f}", f"{s:.3f}"])
        print(f"Saved CSV to: {path}")
    except Exception as e:
        print(f"ERROR: failed to save CSV '{path}': {e}", file=sys.stderr)

def main():
    ap = argparse.ArgumentParser(description="Compute black level and white balance stats from JSON files.")
    ap.add_argument("files", nargs="+", help="JSON files or glob patterns, e.g., out.*.json")
    ap.add_argument("--save-csv", default=None, help="Optional path to save a CSV with per-file values and summary.")
    args = ap.parse_args()

    # Expand globs ourselves to maintain order
    expanded = []
    for arg in args.files:
        import glob
        hits = sorted(glob.glob(arg))
        if hits:
            expanded.extend(hits)
        else:
            expanded.append(arg)  # keep as-is; will error if not found

    bl_rows, wb_rows = collect_values(expanded)

    # Summaries
    bl_sum = summarize(bl_rows, ["R","G1","G2","B","used"])
    wb_sum = summarize(wb_rows, ["R","G1","G2","B"])

    # Print results
    print("\n=== Per-file Black Level [DN] ===")
    for r in bl_rows:
        print(f"{r['file']:>16}: R={r['R']}, G1={r['G1']}, G2={r['G2']}, B={r['B']}, used={r['used']}")

    print("\n=== Per-file White Balance [scale] ===")
    for r in wb_rows:
        print(f"{r['file']:>16}: R={r['R']}, G1={r['G1']}, G2={r['G2']}, B={r['B']}")

    print("\n=== Summary Black Level [DN] (avg ± std) ===")
    print(format_summary("", bl_sum, units="[DN]"))

    print("\n=== Summary White Balance [scale] (avg ± std) ===")
    print(format_summary("", wb_sum, units="[scale]"))

    # Optional CSV
    if args.save_csv:
        maybe_save_csv(args.save_csv, bl_rows, wb_rows, bl_sum, wb_sum)

if __name__ == "__main__":
    sys.exit(main())
