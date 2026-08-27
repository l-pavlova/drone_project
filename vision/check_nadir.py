"""Flag frames whose camera is not pointing straight down.

    python vision/check_nadir.py <stills_or_dataset_dir> [--sample N] [--list]

**The telemetry cannot tell you this.** A DJI SRT carries no gimbal angles, and
`rel_alt` reads a flat ~30 m whether the camera is nadir or aimed at the horizon.
Flight 0034 (2026-08-21, 4.7 min, 175 stills extracted) turned out to be oblique
from end to end, and that was only discovered after the stills were cut, a split
was designed and 53 frames were pre-labelled. Flight 0035 is nadir except for its
last ~7 frames, where the gimbal tilts up during the return.

The test is the cheapest thing that works: **the top strip of a nadir frame is
ground, and the top strip of an oblique frame is sky.** Sky is bright and
blue-dominant; tarmac, roofs and canopy are darker and red-dominant. Measured on
this footage:

    nadir  (0035): brightness 110-140, (B - R) -17 to -29
    oblique(0034): brightness 205-216, (B - R) +26 to +47

The two populations are nowhere near each other, so a crude threshold is enough
and a fancier horizon detector would be false precision. It is a SCREEN, not a
proof: an overcast white sky or a very pale flat roof filling the top of a nadir
frame could trip it, which is why `--list` prints the per-frame numbers so a
borderline call can be looked at rather than trusted.
"""
import argparse
import glob
import os
import sys

sys.stdout.reconfigure(encoding="utf-8")

TOP_FRAC = 0.12          # strip of the frame to test
T_BRIGHT = 180           # above this AND blue-dominant => sky
T_BLUE = 10              # (B - R) mean


def frames(root):
    direct = sorted(glob.glob(os.path.join(root, "*.jpg")))
    if direct:
        return direct
    return sorted(glob.glob(os.path.join(root, "images", "*", "*.jpg")))


def is_oblique(img):
    """(mean brightness, mean B-R, is_oblique) for one BGR image.

    Split out of main() so the projection consumers can apply the same screen:
    an oblique frame projected AS IF nadir does not fail, it silently places its
    cars tens of metres from where they are, and the bay votes that follow are
    confidently wrong. Flight 0074 is 22% oblique, which is where that stopped
    being a theoretical concern.
    """
    top = img[:int(img.shape[0] * TOP_FRAC)].reshape(-1, 3).astype(float)
    bright = top.mean()
    blue = (top[:, 0] - top[:, 2]).mean()          # OpenCV is BGR
    return bright, blue, bool(bright > T_BRIGHT and blue > T_BLUE)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("root", help="stills dir, or a dataset dir with images/<split>")
    ap.add_argument("--sample", type=int, default=0,
                    help="test only N evenly spaced frames (default: all)")
    ap.add_argument("--list", action="store_true", help="print every frame")
    a = ap.parse_args()

    import cv2

    fs = frames(a.root)
    if not fs:
        sys.exit(f"no .jpg found under {a.root}")
    if a.sample and a.sample < len(fs):
        step = len(fs) / a.sample
        fs = [fs[int(i * step)] for i in range(a.sample)]

    oblique = []
    for p in fs:
        img = cv2.imread(p)
        if img is None:
            continue
        bright, blue, bad = is_oblique(img)
        if bad:
            oblique.append(os.path.basename(p))
        if a.list:
            print(f"  {os.path.basename(p):28s} brightness {bright:6.1f}  "
                  f"B-R {blue:+6.1f}  {'OBLIQUE' if bad else 'nadir'}")

    n = len(fs)
    print(f"\n{n} frame(s) tested, {len(oblique)} look OBLIQUE "
          f"({100 * len(oblique) / n:.0f}%)")
    if len(oblique) == n:
        print("  EVERY frame is oblique -- this flight is not a nadir survey and "
              "cannot be used for the detector or for projection.")
    elif oblique:
        print("  " + ", ".join(oblique[:12]) + (" ..." if len(oblique) > 12 else ""))
        print("  Exclude these before labelling; a nadir detector trained on them "
              "learns a viewpoint no survey produces.")
    else:
        print("  All nadir.")
    sys.exit(1 if oblique else 0)


if __name__ == "__main__":
    main()
