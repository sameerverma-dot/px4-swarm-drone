#!/usr/bin/env python3
"""
How many pixels does YOLO need to see a person?

The survey has hit a real trade-off: coverage rate wants altitude, detection
wants low altitude. Before spending flights on it, measure where detection
actually dies as a function of TARGET PIXEL SIZE, and test whether raising the
camera resolution recovers it.

Method: take a real person crop, composite it onto a flat grey background (the
sim's ground) at a controlled pixel size, run YOLO, record the best 'person'
confidence. Repeat over sizes, capture resolutions and inference sizes.

HONEST CAVEAT: the crop is a side-on person from COCO, not a nadir view. So the
absolute confidences here are OPTIMISTIC compared with looking straight down at
someone's head and shoulders. What transfers is the SHAPE of the curve - where
confidence collapses - and the relative comparison between configurations.
"""
import math
import pathlib
import sys

import cv2
import numpy as np
from ultralytics import YOLO

HFOV = 1.74
GROUND = 118          # the default world's grey ground plane
PERSON_M = 0.5        # a standing person is ~0.5 m across seen from above


def person_crop():
    """Cut the clearest person out of bus.jpg using YOLO itself."""
    import ultralytics
    img = cv2.imread(str(pathlib.Path(ultralytics.__file__).parent / 'assets' / 'bus.jpg'))
    r = YOLO('yolov8n.pt')(img, classes=[0], conf=0.5, verbose=False)[0]
    best, area = None, 0
    for b in r.boxes:
        x1, y1, x2, y2 = [int(v) for v in b.xyxy[0].tolist()]
        if (x2 - x1) * (y2 - y1) > area:
            area, best = (x2 - x1) * (y2 - y1), (x1, y1, x2, y2)
    x1, y1, x2, y2 = best
    return img[y1:y2, x1:x2]


def scene(crop, target_px, w, h):
    """Grey frame of w x h with the person scaled so its WIDTH is target_px."""
    ch, cw = crop.shape[:2]
    tw = max(2, int(round(target_px)))
    th = max(2, int(round(tw * ch / cw)))
    if th >= h:
        th = h - 2
        tw = max(2, int(round(th * cw / ch)))
    small = cv2.resize(crop, (tw, th), interpolation=cv2.INTER_AREA)
    frame = np.full((h, w, 3), GROUND, np.uint8)
    # mild noise so it isn't a perfectly synthetic flat field
    frame = cv2.add(frame, np.random.default_rng(0).integers(
        -6, 7, (h, w, 3), dtype=np.int16).astype(np.int16).clip(-255, 255).astype(np.uint8))
    y0, x0 = (h - th) // 2, (w - tw) // 2
    frame[y0:y0 + th, x0:x0 + tw] = small
    return frame


def best_person_conf(model, frame, imgsz):
    r = model(frame, classes=[0], conf=0.01, imgsz=imgsz, verbose=False)[0]
    return max((float(b.conf[0]) for b in r.boxes), default=0.0)


def px_at(alt, cap_w):
    """Target width in pixels at a given altitude and capture width."""
    return PERSON_M / (2 * alt * math.tan(HFOV / 2) / cap_w)


def main():
    model = YOLO('yolov8n.pt')
    crop = person_crop()
    print(f"person crop: {crop.shape[1]}x{crop.shape[0]} px from bus.jpg\n")

    # ---------- 1. confidence vs target pixel size ----------
    print("1. Confidence vs target size  (capture 640x480, imgsz 640)")
    print(f"   {'px':>5} {'conf':>6}  {'':<24} {'~altitude':>10}")
    sizes = [6, 8, 10, 13, 16, 20, 24, 27, 32, 40, 50, 64, 80, 100]
    curve = {}
    for px in sizes:
        c = best_person_conf(model, scene(crop, px, 640, 480), 640)
        curve[px] = c
        bar = '#' * int(c * 24)
        alt = PERSON_M * 640 / (2 * math.tan(HFOV / 2) * px)
        print(f"   {px:>5} {c:>6.2f}  {bar:<24} {alt:>9.1f}m")

    # ---------- 2. does capture resolution help on its own? ----------
    print("\n2. Same PHYSICAL scene, different capture resolution & imgsz")
    print("   (the question: does a 1280 camera recover what altitude costs?)")
    print(f"   {'altitude':>9} {'capture':>10} {'imgsz':>6} {'target px':>10} {'conf':>6}")
    for alt in (5.0, 10.0):
        for cap_w, cap_h in ((640, 480), (1280, 960)):
            for imgsz in (640, 1280):
                px = px_at(alt, cap_w)
                c = best_person_conf(model, scene(crop, px, cap_w, cap_h), imgsz)
                # what the target becomes AFTER ultralytics resizes to imgsz
                eff = px * imgsz / cap_w
                print(f"   {alt:>8.0f}m {cap_w:>6}x{cap_h:<3} {imgsz:>6} "
                      f"{px:>7.0f}px {c:>6.2f}   (effective {eff:.0f}px at inference)")

    # ---------- 3. the practical answer ----------
    print("\n3. Detection floor")
    for thr in (0.40, 0.25):
        ok = [p for p, c in curve.items() if c >= thr]
        if ok:
            need = min(ok)
            alt = PERSON_M * 640 / (2 * math.tan(HFOV / 2) * need)
            alt2 = PERSON_M * 1280 / (2 * math.tan(HFOV / 2) * need)
            print(f"   conf >= {thr:.2f} needs >= {need} px -> max altitude "
                  f"{alt:.1f} m at 640 capture, {alt2:.1f} m at 1280 (with imgsz to match)")
    return 0


if __name__ == '__main__':
    sys.exit(main())
