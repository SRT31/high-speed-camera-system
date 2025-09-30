
#!/usr/bin/env python3
"""
sensor_calibrate_imx219.py  (fixed)

Calibrate IMX219 (or any Bayer RAW) frames to obtain:
- Black level (`dcraw -k`)
- Saturation point (`dcraw -S`)
- White balance multipliers (`dcraw -r r g1 g2 b`)

It accepts:
  1) Raw-mosaic TIFFs created with:  dcraw -4 -E -T your.raw
  2) Bare .raw frames from raspiraw:
     - packed RAW10/RAW12, or 16-bit container (Y16)
     - you must provide width/height/bitdepth/packing.

Features:
- Robust statistics (percentiles & Huber-like clipping)
- ROI-driven or automatic WB estimation (bright non-clipped pixels)
- Per-channel histograms (optional)
- Fixed Pattern Noise diagnostics (row/column medians)
- JSON + plain-text report
- Suggested dcraw flags printed at the end
"""

import argparse
import json
import math
import os
import sys
from typing import Dict, Optional, Tuple

import numpy as np

# Optional imports for TIFF I/O and plotting
try:
    from PIL import Image
    PIL_OK = True
except Exception:
    PIL_OK = False

try:
    import matplotlib.pyplot as plt
    MPL_OK = True
except Exception:
    MPL_OK = False


def human(v: float) -> str:
    if v is None or (isinstance(v, float) and (np.isnan(v) or np.isinf(v))):
        return "n/a"
    if abs(v) >= 1000:
        return f"{v:,.0f}"
    if abs(v) >= 100:
        return f"{v:.1f}"
    if abs(v) >= 10:
        return f"{v:.2f}"
    return f"{v:.3f}"


def clip(a, lo, hi):
    return max(lo, min(hi, a))


def parse_roi(s: str) -> Tuple[int, int, int, int]:
    try:
        x, y, w, h = map(int, s.split(","))
        return x, y, w, h
    except Exception:
        raise argparse.ArgumentTypeError("ROI must be 'x,y,w,h' integers")


def load_tiff(path: str) -> np.ndarray:
    if not PIL_OK:
        raise RuntimeError("Pillow not available. Can't read TIFFs. Install pillow.")
    im = Image.open(path)
    arr = np.array(im)
    if arr.ndim != 2:
        raise ValueError(f"Expected a raw-mosaic TIFF (single channel). Got shape: {arr.shape}")
    if arr.dtype != np.uint16:
        arr = arr.astype(np.uint16, copy=False)
    return arr


def unpack_raw10_row(row_bytes: bytes, width: int) -> np.ndarray:
    """Unpack a single row of RAW10 data (5 bytes -> 4 pixels)."""
    W = width
    out = np.empty(W, dtype=np.uint16)
    x = 0
    i = 0
    full_groups = W // 4
    for g in range(full_groups):
        b0 = row_bytes[i+0]
        b1 = row_bytes[i+1]
        b2 = row_bytes[i+2]
        b3 = row_bytes[i+3]
        b4 = row_bytes[i+4]
        i += 5
        out[x+0] = (b0 << 2) | ((b4 >> 0) & 0x03)
        out[x+1] = (b1 << 2) | ((b4 >> 2) & 0x03)
        out[x+2] = (b2 << 2) | ((b4 >> 4) & 0x03)
        out[x+3] = (b3 << 2) | ((b4 >> 6) & 0x03)
        x += 4
    rem = W - x
    if rem:
        # Read one more 5-byte group and use the first 'rem' pixels
        # Pad if the row_bytes is shorter (defensive).
        pad = row_bytes[i:i+5] + b"\x00" * max(0, 5 - len(row_bytes[i:i+5]))
        b0, b1, b2, b3, b4 = pad[:5]
        vals = [
            (b0 << 2) | ((b4 >> 0) & 0x03),
            (b1 << 2) | ((b4 >> 2) & 0x03),
            (b2 << 2) | ((b4 >> 4) & 0x03),
            (b3 << 2) | ((b4 >> 6) & 0x03),
        ]
        out[x:W] = vals[:rem]
    return out


def unpack_raw10(packed: bytes, width: int, height: int) -> np.ndarray:
    """Unpack MIPI RAW10 (row by row)."""
    import math as _m
    out = np.empty((height, width), dtype=np.uint16)
    row_stride = int(_m.ceil(width / 4.0) * 5)  # bytes per row
    for y in range(height):
        row = packed[y*row_stride:(y+1)*row_stride]
        out[y, :] = unpack_raw10_row(row, width)
    return out


def unpack_raw12_row(row_bytes: bytes, width: int) -> np.ndarray:
    """Unpack a single row of RAW12 data (3 bytes -> 2 pixels)."""
    W = width
    out = np.empty(W, dtype=np.uint16)
    x = 0
    i = 0
    full_groups = W // 2
    for g in range(full_groups):
        b0 = row_bytes[i+0]
        b1 = row_bytes[i+1]
        b2 = row_bytes[i+2]
        i += 3
        out[x+0] = (b0 << 4) | (b2 & 0x0F)
        out[x+1] = (b1 << 4) | (b2 >> 4)
        x += 2
    if x < W:
        pad = row_bytes[i:i+3] + b"\x00" * max(0, 3 - len(row_bytes[i:i+3]))
        b0, b1, b2 = pad[:3]
        out[x] = (b0 << 4) | (b2 & 0x0F)
    return out


def unpack_raw12(packed: bytes, width: int, height: int) -> np.ndarray:
    import math as _m
    out = np.empty((height, width), dtype=np.uint16)
    row_stride = int(_m.ceil(width / 2.0) * 3)
    for y in range(height):
        row = packed[y*row_stride:(y+1)*row_stride]
        out[y, :] = unpack_raw12_row(row, width)
    return out


def load_raw(path: str, args) -> np.ndarray:
    ext = os.path.splitext(path)[1].lower()
    if ext in (".tif", ".tiff"):
        return load_tiff(path)
    if not args.width or not args.height or not args.bitdepth or not args.packing:
        raise ValueError("For non-TIFF input you must provide --width --height --bitdepth --packing")
    with open(path, "rb") as f:
        bs = f.read()
    if args.packing.lower() == "raw10":
        arr = unpack_raw10(bs, args.width, args.height)
    elif args.packing.lower() == "raw12":
        arr = unpack_raw12(bs, args.width, args.height)
    elif args.packing.lower() in ("y16", "u16", "uint16"):
        arr = np.frombuffer(bs, dtype="<u2").reshape(args.height, args.width)
    else:
        raise ValueError(f"Unknown packing: {args.packing}")
    return arr


BAYER_ORDERS = ("RGGB", "GRBG", "GBRG", "BGGR")

def split_bayer_planes(raw: np.ndarray, order: str):
    order = order.upper()
    if order not in BAYER_ORDERS:
        raise ValueError("Bayer order must be one of %s" % (BAYER_ORDERS,))
    r = raw[0::2, 0::2]
    g1 = raw[0::2, 1::2]
    g2 = raw[1::2, 0::2]
    b = raw[1::2, 1::2]
    if order == "RGGB":
        return r, g1, g2, b
    if order == "GRBG":
        return g1, r, b, g2
    if order == "GBRG":
        return g2, b, r, g1
    if order == "BGGR":
        return b, g2, g1, r
    return r, g1, g2, b


def combine_green(g1: np.ndarray, g2: np.ndarray) -> np.ndarray:
    return 0.5 * (g1.astype(np.float64) + g2.astype(np.float64))


class ChannelStats:
    __slots__ = ("name","min","p01","p1","p5","median","mean","p95","p99","p999","max","std")
    def __init__(self, name="", **kw):
        for k in self.__slots__:
            setattr(self, k, kw.get(k))


def robust_channel_stats(x: np.ndarray, bitdepth: int) -> ChannelStats:
    flat = x.reshape(-1).astype(np.float64)
    pcts = np.percentile(flat, [0, 0.1, 1, 5, 50, 95, 99, 99.9, 100])
    mean = float(np.mean(flat))
    std = float(np.std(flat))
    return ChannelStats(
        min=float(pcts[0]),
        p01=float(pcts[1]),
        p1=float(pcts[2]),
        p5=float(pcts[3]),
        median=float(pcts[4]),
        mean=mean,
        p95=float(pcts[5]),
        p99=float(pcts[6]),
        p999=float(pcts[7]),
        max=float(pcts[8]),
        std=std,
    )


def estimate_black_level(x: np.ndarray, bitdepth: int, mode: str) -> float:
    st = robust_channel_stats(x, bitdepth)
    if mode == "dark":
        return float(np.median([st.p01, st.p1, st.median]))
    else:
        return float(st.p01)


def estimate_saturation(x: np.ndarray, bitdepth: int) -> float:
    st = robust_channel_stats(x, bitdepth)
    fullscale = (1 << bitdepth) - 1
    sat = st.p999
    if abs(st.max - fullscale) <= 1.0:
        sat = fullscale
    return float(min(fullscale, sat))


def wb_from_roi(raw: np.ndarray, order: str, roi: Tuple[int, int, int, int]) -> Tuple[float, float, float, float]:
    x, y, w, h = roi
    sub = raw[y:y+h, x:x+w]
    r, g1, g2, b = split_bayer_planes(sub, order)
    g = combine_green(g1, g2)
    r_m = float(np.mean(r))
    g1_m = float(np.mean(g1))
    g2_m = float(np.mean(g2))
    b_m = float(np.mean(b))
    g_ref = float(np.mean(g))
    return (g_ref / r_m, g_ref / g1_m, g_ref / g2_m, g_ref / b_m)


def wb_brightest_nonclipped(raw: np.ndarray, order: str, sat: float, margin: float = 16.0) -> Tuple[float, float, float, float]:
    r, g1, g2, b = split_bayer_planes(raw, order)
    g = combine_green(g1, g2)
    thresh_hi = max(1.0, sat - margin)
    bright_mask = (g > np.percentile(g, 98)) & (g < thresh_hi)

    def safe_mean(arr):
        vals = arr[bright_mask]
        if vals.size < 16:
            vals = arr.reshape(-1)
        return float(np.mean(vals))

    r_m = safe_mean(r)
    g1_m = safe_mean(g1)
    g2_m = safe_mean(g2)
    b_m = safe_mean(b)
    g_ref = (g1_m + g2_m) * 0.5
    return (g_ref / r_m, g_ref / g1_m, g_ref / g2_m, g_ref / b_m)


def fpn_metrics(raw: np.ndarray) -> Dict[str, float]:
    row_meds = np.median(raw, axis=1)
    col_meds = np.median(raw, axis=0)
    return {
        "row_median_std": float(np.std(row_meds)),
        "col_median_std": float(np.std(col_meds)),
    }


def dynamic_range_stops(black: float, sat: float, dark_noise: float) -> Optional[float]:
    if dark_noise is None or dark_noise <= 0:
        return None
    rng = max(1e-6, sat - black)
    return math.log2(rng / max(1e-6, dark_noise))


def save_histograms(path_prefix: str, channels: Dict[str, np.ndarray], bitdepth: int):
    if not MPL_OK:
        print("matplotlib not available; skipping histograms.")
        return
    bins = min(2048, 2 ** bitdepth)
    for name, ch in channels.items():
        plt.figure()
        plt.hist(ch.reshape(-1), bins=bins)
        plt.title(f"Histogram: {name}")
        plt.xlabel("DN")
        plt.ylabel("Count")
        out = f"{path_prefix}_hist_{name}.png"
        plt.savefig(out, dpi=140, bbox_inches="tight")
        plt.close()
        print(f"[saved] {out}")


def main():
    ap = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--input", required=True, help="Path to a raw-mosaic TIFF (dcraw -4 -E) or a bare .raw")
    ap.add_argument("--width", type=int, help="Width (required for non-TIFF)")
    ap.add_argument("--height", type=int, help="Height (required for non-TIFF)")
    ap.add_argument("--bitdepth", type=int, required=True, choices=[8,10,12,14,16], help="Sensor bit depth")
    ap.add_argument("--packing", default=None, help="One of: raw10, raw12, y16 (for bare .raw)")
    ap.add_argument("--bayer", default="BGGR", choices=["RGGB","GRBG","GBRG","BGGR"], help="Bayer order")
    ap.add_argument("--mode", default="auto", choices=["auto","dark"], help="'dark' if this is a dark frame capture")
    ap.add_argument("--wb-mode", dest="wb_mode", default="auto", choices=["auto","roi"], help="How to compute white balance")
    ap.add_argument("--roi", type=parse_roi, default=None, help="ROI 'x,y,w,h' for WB when --wb-mode=roi")
    ap.add_argument("--save-plots", action="store_true", help="Save per-channel histograms (PNG)")
    ap.add_argument("--bad-pixel-threshold", type=float, default=None, help="If in dark mode, DN above (black+threshold) marks hot pixels; a badpixels.txt will be written")
    ap.add_argument("--report", default="sensor_report.json", help="Output JSON report filename")
    args = ap.parse_args()

    print(f"[opts] wb_mode={args.wb_mode} mode={args.mode} bayer={args.bayer} bitdepth={args.bitdepth}")

    raw = load_raw(args.input, args)
    H, W = raw.shape
    print(f"Loaded raw: {args.input} [{W}x{H}] bitdepth={args.bitdepth} bayer={args.bayer} mode={args.mode}")

    R, G1, G2, B = split_bayer_planes(raw, args.bayer)
    G = combine_green(G1, G2)

    cs = {
        "R": robust_channel_stats(R, args.bitdepth),
        "G1": robust_channel_stats(G1, args.bitdepth),
        "G2": robust_channel_stats(G2, args.bitdepth),
        "B": robust_channel_stats(B, args.bitdepth),
    }

    black_R = estimate_black_level(R, args.bitdepth, args.mode)
    black_G1 = estimate_black_level(G1, args.bitdepth, args.mode)
    black_G2 = estimate_black_level(G2, args.bitdepth, args.mode)
    black_B = estimate_black_level(B, args.bitdepth, args.mode)
    black = float(np.median([black_R, black_G1, black_G2, black_B]))

    sat_R = estimate_saturation(R, args.bitdepth)
    sat_G1 = estimate_saturation(G1, args.bitdepth)
    sat_G2 = estimate_saturation(G2, args.bitdepth)
    sat_B = estimate_saturation(B, args.bitdepth)
    sat = float(np.median([sat_R, sat_G1, sat_G2, sat_B]))

    if args.wb_mode == "roi":
        if args.roi is None:
            print("ERROR: --wb-mode=roi requires --roi x,y,w,h", file=sys.stderr)
            sys.exit(2)
        r_mult, g1_mult, g2_mult, b_mult = wb_from_roi(raw, args.bayer, args.roi)
        wb_source = f"roi={args.roi}"
    else:
        r_mult, g1_mult, g2_mult, b_mult = wb_brightest_nonclipped(raw, args.bayer, sat)
        wb_source = "brightest_nonclipped"

    dark_noise = None
    if args.mode == "dark":
        g_dark = np.concatenate([G1.reshape(-1), G2.reshape(-1)], axis=0)
        q25, q75 = np.percentile(g_dark, [25, 75])
        dark_noise = float((q75 - q25) / 1.349)

    fpn = fpn_metrics(raw)

    if args.save_plots:
        save_histograms(os.path.splitext(args.input)[0], {"R":R, "G1":G1, "G2":G2, "B":B}, args.bitdepth)

    k_int = int(round(clip(black, 0, (1<<args.bitdepth)-1)))
    S_int = int(round(clip(sat, 1, (1<<args.bitdepth)-1)))

    r_mult_f = float(r_mult); g1_mult_f = float(g1_mult); g2_mult_f = float(g2_mult); b_mult_f = float(b_mult)
    dr_stops = dynamic_range_stops(black, sat, dark_noise)

    print("\n=== SENSOR CALIBRATION REPORT ===")
    print(f"Black level estimate (DN): R={human(black_R)}, G1={human(black_G1)}, G2={human(black_G2)}, B={human(black_B)} -> used {k_int}")
    print(f"Saturation estimate (DN):  R={human(sat_R)}, G1={human(sat_G1)}, G2={human(sat_G2)}, B={human(sat_B)} -> used {S_int}")
    print(f"White balance source: {wb_source}")
    print(f"WB multipliers (-r):  r={r_mult_f:.4f}  g1={g1_mult_f:.4f}  g2={g2_mult_f:.4f}  b={b_mult_f:.4f}")
    if dr_stops is not None:
        print(f"Approx dynamic range (stops): {dr_stops:.2f}")
    print(f"FPN (row_median_std, col_median_std) DN: {human(fpn['row_median_std'])}, {human(fpn['col_median_std'])}")

    print("\nSuggested dcraw flags:")
    print(f"  -k {k_int} -S {S_int} -r {r_mult_f:.4f} {g1_mult_f:.4f} {g2_mult_f:.4f} {b_mult_f:.4f}")
    print("Common extras: -q 3 -6 -T -W -o 1")

    report = {
        "input": args.input,
        "width": W,
        "height": H,
        "bitdepth": args.bitdepth,
        "bayer": args.bayer,
        "mode": args.mode,
        "black_level_estimates": {
            "R": black_R, "G1": black_G1, "G2": black_G2, "B": black_B, "used": k_int
        },
        "saturation_estimates": {
            "R": sat_R, "G1": sat_G1, "G2": sat_G2, "B": sat_B, "used": S_int
        },
        "white_balance": {
            "source": wb_source,
            "r": r_mult_f, "g1": g1_mult_f, "g2": g2_mult_f, "b": b_mult_f
        },
        "dark_noise_std_dn": dark_noise,
        "dynamic_range_stops": dr_stops,
        "fpn_metrics": fpn,
    }
    with open(args.report, "w") as f:
        json.dump(report, f, indent=2)
    print(f"[saved] {args.report}")

    if args.mode == "dark" and args.bad_pixel_threshold is not None:
        thr = black + args.bad_pixel_threshold
        bad = np.where(raw > thr)
        out = os.path.splitext(args.report)[0] + "_badpixels.txt"
        with open(out, "w") as f:
            for y, x in zip(bad[0], bad[1]):
                f.write(f"{int(x)} {int(y)}\n")
        print(f"[saved] {out}  ({bad[0].size} candidates)")


if __name__ == "__main__":
    main()
