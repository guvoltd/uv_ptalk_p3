#!/usr/bin/env python3
"""
parse_gif_pixels.py

Open an animated GIF, iterate frames, parse pixels per frame,
and optionally export per-frame CSVs listing non-transparent pixels.

Usage:
    python parse_gif_pixels.py input.gif --out-dir output_folder --export-csv

Dependencies:
    pip install pillow numpy
"""

import os
import sys
import csv
import argparse

# Try to use a user-specified 'animatedgif' library if available.
# If not present, fall back to Pillow (PIL).
try:
    import animatedgif as ag  # If you have a real animatedgif package, it will be used
    _USE_ANIMATEDGIF = True
except Exception:
    _USE_ANIMATEDGIF = False

from PIL import Image, ImageSequence
import numpy as np

def ensure_out_dir(path):
    os.makedirs(path, exist_ok=True)
    return path

def parse_frame_pixels_pillow(frame: Image.Image, threshold_alpha: int = 1):
    """
    Convert a PIL frame to RGBA numpy array and return:
      - non_transparent_count
      - list of (x, y, r, g, b, a) for non-transparent pixels (can be large)
    threshold_alpha: pixels with alpha > threshold_alpha are considered visible
    """
    rgba = frame.convert("RGBA")
    arr = np.array(rgba)  # shape (h, w, 4)
    alpha = arr[:, :, 3]
    mask = alpha > threshold_alpha
    ys, xs = np.nonzero(mask)
    pixels = []
    for y, x in zip(ys, xs):
        r, g, b, a = arr[y, x].tolist()
        pixels.append((int(x), int(y), int(r), int(g), int(b), int(a)))
    return len(pixels), pixels

def export_pixels_csv(pixels, out_csv_path):
    """
    Write pixels list to CSV with header: x,y,r,g,b,a
    """
    with open(out_csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["x", "y", "r", "g", "b", "a"])
        writer.writerows(pixels)

def process_with_pillow(gif_path, out_dir, export_csv=False, max_pixels_export=295940):
    im = Image.open(gif_path)
    frame_index = 0
    summary = []
    for frame in ImageSequence.Iterator(im):
        non_trans_count, pixels = parse_frame_pixels_pillow(frame)
        summary.append((frame_index, non_trans_count))
        print(f"Frame {frame_index}: non-transparent pixels = {non_trans_count}")
        if export_csv:
            # Avoid writing extremely large CSVs by default
            if non_trans_count <= max_pixels_export:
                csv_path = os.path.join(out_dir, f"frame_{frame_index:03d}.csv")
                export_pixels_csv(pixels, csv_path)
                print(f"  -> exported {len(pixels)} pixels to {csv_path}")
            else:
                print(f"  -> skipped CSV export for frame {frame_index} (too many pixels: {non_trans_count})")
        frame_index += 1
    return summary

def process_with_animatedgif(gif_path, out_dir, export_csv=False):
    """
    Example wrapper if a real 'animatedgif' library is available.
    This function assumes the library exposes a simple API; adapt as needed.
    """
    # NOTE: This is a best-effort wrapper. Replace with the actual API calls
    # of your installed animatedgif library.
    ag_gif = ag.load(gif_path)  # hypothetical API
    summary = []
    for i, frame in enumerate(ag_gif.frames):
        # Convert frame to PIL Image if possible
        if hasattr(frame, "to_pil"):
            pil_frame = frame.to_pil()
        else:
            # try to create from raw bytes if available
            pil_frame = Image.fromarray(np.array(frame))
        non_trans_count, pixels = parse_frame_pixels_pillow(pil_frame)
        summary.append((i, non_trans_count))
        print(f"[animatedgif] Frame {i}: non-transparent pixels = {non_trans_count}")
        if export_csv:
            csv_path = os.path.join(out_dir, f"frame_{i:03d}.csv")
            export_pixels_csv(pixels, csv_path)
            print(f"  -> exported {len(pixels)} pixels to {csv_path}")
    return summary

def main():
    parser = argparse.ArgumentParser(description="Parse pixels from an animated GIF.")
    parser.add_argument("gif", help="Path to input animated GIF")
    parser.add_argument("--out-dir", default="gif_pixels_out", help="Directory to write outputs")
    parser.add_argument("--export-csv", action="store_true", help="Export per-frame pixel CSVs")
    parser.add_argument("--alpha-threshold", type=int, default=1, help="Alpha threshold to consider pixel visible (0-255)")
    args = parser.parse_args()

    gif_path = args.gif
    if not os.path.isfile(gif_path):
        print(f"Error: file not found: {gif_path}", file=sys.stderr)
        sys.exit(2)

    out_dir = ensure_out_dir(args.out_dir)

    if _USE_ANIMATEDGIF:
        print("Using installed animatedgif library for frame extraction.")
        summary = process_with_animatedgif(gif_path, out_dir, export_csv=args.export_csv)
    else:
        print("animatedgif not found; using Pillow for frame extraction.")
        summary = process_with_pillow(gif_path, out_dir, export_csv=args.export_csv)

    print("\nSummary (frame_index, non_transparent_pixel_count):")
    for s in summary:
        print(f"  {s[0]}: {s[1]}")

if __name__ == "__main__":
    main()
