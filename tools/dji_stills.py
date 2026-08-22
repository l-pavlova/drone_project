"""Pull stills out of a DJI video at a fixed rate, each paired with its SRT pose.

The .SRT carries one block per video frame, so a still's pose is just the block
with the same index -- no interpolation, no clock alignment.  Writes
<out>/frame_####.jpg plus a poses.json in the same shape score_occupancy.py
reads (x/y/alt local ENU metres about ORIGIN), so a real flight can be fed to
the same tooling as a simulated one.

    python tools/dji_stills.py pics/dji/DJI_..._0035_D.MP4 [--hz 1] [--out DIR]

Note: DJI SRTs from this airframe carry no gimbal angles and no yaw, so the
poses here have no attitude and will project as nadir.  See dji_srt.py.
"""
import argparse, json, os, sys, math

sys.stdout.reconfigure(encoding="utf-8")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from dji_srt import parse, to_pose  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("video")
    ap.add_argument("--hz", type=float, default=1.0, help="stills per second (default 1)")
    ap.add_argument("--out", help="output dir (default pics/dji/stills/<video stem>)")
    ap.add_argument("--scale", type=float, default=1.0, help="resize factor, 1.0 = native")
    ap.add_argument("--quality", type=int, default=92)
    ap.add_argument("--start", type=float, default=0.0, help="skip the first N seconds")
    ap.add_argument("--end", type=float, help="stop at N seconds")
    a = ap.parse_args()

    import cv2

    vid = os.path.abspath(a.video)
    stem = os.path.splitext(os.path.basename(vid))[0]
    srt = os.path.splitext(vid)[0] + ".SRT"
    out = a.out or os.path.join(os.path.dirname(vid), "stills", stem)
    os.makedirs(out, exist_ok=True)

    cap = cv2.VideoCapture(vid)
    if not cap.isOpened():
        sys.exit(f"cannot open {vid}")
    fps = cap.get(cv2.CAP_PROP_FPS)
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"{stem}: {w}x{h} @ {fps:.2f} fps, {total} frames ({total / fps:.1f} s)")

    poses = []
    if os.path.exists(srt):
        poses = [q for q in (to_pose(r) for r in parse(srt)) if q]
        print(f"  {len(poses)} SRT poses")
    else:
        print("  no .SRT beside the video -- stills only, no poses")

    step = max(1, int(round(fps / a.hz)))
    first = int(a.start * fps)
    last = int(a.end * fps) if a.end else total
    by_idx = {p["frame_idx"]: p for p in poses}

    kept = []
    n = 0
    # Read sequentially and drop frames: seeking per still is far slower on
    # long-GOP H.265 and can land on the wrong frame.
    idx = -1
    while True:
        ok = cap.grab()
        if not ok:
            break
        idx += 1
        if idx < first:
            continue
        if idx >= last:
            break
        if (idx - first) % step:
            continue
        ok, img = cap.retrieve()
        if not ok:
            continue
        if a.scale != 1.0:
            img = cv2.resize(img, (int(w * a.scale), int(h * a.scale)),
                             interpolation=cv2.INTER_AREA)
        name = f"frame_{n:04d}.jpg"
        cv2.imwrite(os.path.join(out, name),
                    img, [cv2.IMWRITE_JPEG_QUALITY, a.quality])
        p = by_idx.get(idx + 1)  # SRT FrameCnt is 1-based
        rec = {"file": name, "video_frame": idx, "t_s": round(idx / fps, 3)}
        if p:
            rec.update({k: p[k] for k in
                        ("x", "y", "alt", "lat", "lon", "captured_at")
                        if k in p})
        kept.append(rec)
        n += 1
        if n % 20 == 0:
            print(f"  {n} stills ...", flush=True)

    cap.release()
    with open(os.path.join(out, "poses.json"), "w", encoding="utf-8") as f:
        json.dump(kept, f, indent=1)

    withpose = sum(1 for r in kept if "x" in r)
    print(f"  -> {n} stills in {out} ({withpose} with a pose)")
    if withpose > 1:
        d = sum(math.hypot(b["x"] - a_["x"], b["y"] - a_["y"])
                for a_, b in zip(kept, kept[1:]) if "x" in a_ and "x" in b)
        print(f"     track {d:.0f} m, mean spacing {d / (withpose - 1):.1f} m between stills")


if __name__ == "__main__":
    main()
