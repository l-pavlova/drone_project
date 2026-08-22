"""Small shared helpers for YOLO label files.

Three functions that were being copy-pasted between `check_labels.py`,
`detect_real.py`, `make_split.py`, `merge_datasets.py` and `prelabel.py`.

`vision/detect_baseline.py` keeps its own `iou()` deliberately: it is committed
code behind numbers already on record for the sim, and editing it for tidiness
would put those numbers at risk for no gain.
"""
import os


def frame_no(fn):
    """Trailing integer of a frame filename, or -1. Tolerates a prefix, so
    `f34_frame_0050.jpg` and `frame_0050.png` both give 50."""
    try:
        return int(os.path.splitext(os.path.basename(fn))[0].split("_")[-1])
    except ValueError:
        return -1


def iou(a, b):
    """Intersection over union of two [x0, y0, x1, y1] boxes."""
    ix0, iy0 = max(a[0], b[0]), max(a[1], b[1])
    ix1, iy1 = min(a[2], b[2]), min(a[3], b[3])
    if ix1 <= ix0 or iy1 <= iy0:
        return 0.0
    inter = (ix1 - ix0) * (iy1 - iy0)
    union = ((a[2] - a[0]) * (a[3] - a[1])
             + (b[2] - b[0]) * (b[3] - b[1]) - inter)
    return inter / union if union > 0 else 0.0


def read_yolo(path, w=1.0, h=1.0):
    """YOLO .txt -> [(x0, y0, x1, y1), ...], scaled by w/h (default: normalised).

    A missing file returns [] -- callers that need to distinguish "no cars here"
    from "nobody has labelled this yet" must test for the file themselves, and
    `check_labels.py` is the thing that does.
    """
    out = []
    if not os.path.exists(path):
        return out
    for line in open(path, encoding="utf-8"):
        f = line.split()
        if len(f) != 5:
            continue
        try:
            cx, cy, bw, bh = (float(v) for v in f[1:])
        except ValueError:
            continue
        out.append(((cx - bw / 2) * w, (cy - bh / 2) * h,
                    (cx + bw / 2) * w, (cy + bh / 2) * h))
    return out
