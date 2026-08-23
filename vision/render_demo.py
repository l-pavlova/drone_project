"""Render the annotated demo video from a real flight's stills.

    python vision/render_demo.py <stills_dir> [--out demo.mp4] [--hold 1.0]
                                 [--fps 10] [--model W] [--conf 0.25]
                                 [--width 1920] [--bays] [--refresh]

Pre-rendered on purpose. The alternative -- running the detector live during
the demo -- puts a CPU inference pass on the critical path of a presentation,
where a slow frame is not a slow frame, it is a stall in front of an audience.
Everything expensive happens here, once; what plays back is a file.

**Real time, and it stays real time.** These stills were cut at 1 Hz, so
`--hold 1.0` shows each for exactly one second and the video runs at the speed
the drone actually flew: 137 frames, ~2.3 minutes. The video's own frame rate
(`--fps`) is a separate thing -- each still is simply repeated `hold * fps`
times -- because a 1 fps MP4 scrubs badly and stutters in some players, while a
10 fps one holding each image for ten frames plays smoothly everywhere and is
the same footage at the same speed.

**What is drawn, and why each layer earns its place:**

  * **detection boxes** -- what the learned model found. This is the thing being
    demonstrated.
  * **projected bays, coloured by THIS frame's verdict** -- OFF by default
    (`--bays`). They show the product decision rather than just a car detector,
    computed by the same `bay_votes_from_dets` rule the server runs. But on this
    street the Sofiaplan bay data is visibly wrong: columns of bays land on the
    tram median and the pavement while the cars sit in rows it has no bays for
    (detection-to-nearest-bay median 4.35 m, 30% inside the 3 m radius, and no
    global shift fixes it). Drawn, they make the video argue about the bay data
    instead of showing the detector, so the default is off and the map -- which
    is where a bay verdict belongs -- carries that half of the story.
  * **a HUD** carrying frame number, flight time, yaw and its source, so the
    georeferencing is visible rather than implied -- the heading is estimated
    (tools/dji_yaw.py), and a demo that hides that is overclaiming.

Note the per-frame colour is the SINGLE-VIEW verdict, not the map's. The map
votes over every frame that saw a bay, so a bay can flicker here and still be
decided correctly there -- that difference is the multi-view vote doing its job
and is worth pointing at rather than hiding.

Detections are cached beside the video (`<out>.dets.json`), so re-rendering
with different drawing options costs seconds instead of a full inference pass.
Pass --refresh to recompute them.
"""
import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import cameras                                                  # noqa: E402
import cv2                                                      # noqa: E402
import numpy as np                                              # noqa: E402
import score_occupancy as so                                    # noqa: E402
from detect_occupancy import bay_votes_from_dets                # noqa: E402

sys.stdout.reconfigure(encoding="utf-8")

BOX = (60, 220, 255)        # BGR: detections, amber
FREE = (90, 210, 110)
OCC = (70, 80, 240)
HUD_BG = (28, 28, 30)
HUD_FG = (235, 235, 235)
BAY_ALPHA = 0.28            # bays are a translucent wash, not an outline -- see draw()


def detections(stills, poses, model_name, conf, imgsz, cache, refresh):
    """Per-frame boxes, computed once and cached to disk."""
    if os.path.exists(cache) and not refresh:
        with open(cache, encoding="utf-8") as fh:
            got = json.load(fh)
        if got.get("model") == model_name and got.get("conf") == conf:
            print(f"  detections from cache ({len(got['frames'])} frames)")
            return got["frames"]
    from ultralytics import YOLO
    model = YOLO(model_name)
    print(f"  running {os.path.basename(model_name)} over {len(poses)} frames "
          f"(cached afterwards)")
    frames = {}
    for n, pose in enumerate(poses):
        img = cv2.imread(os.path.join(stills, pose["file"]))
        if img is None:
            continue
        res = model.predict(img[:, :, ::-1], conf=conf, imgsz=imgsz,
                            verbose=False)[0]
        frames[pose["file"]] = [
            {"box": [round(c, 1) for c in b], "conf": round(c, 3)}
            for b, c in zip(res.boxes.xyxy.tolist(), res.boxes.conf.tolist())]
        if (n + 1) % 20 == 0:
            print(f"    {n + 1}/{len(poses)}")
    with open(cache, "w", encoding="utf-8", newline="") as fh:
        json.dump({"model": model_name, "conf": conf, "frames": frames}, fh)
    return frames


def hud(img, lines, scale):
    """A bar across the bottom. Drawn last so nothing overlaps it."""
    h, w = img.shape[:2]
    bar = int(46 * scale)
    cv2.rectangle(img, (0, h - bar), (w, h), HUD_BG, -1)
    x = int(18 * scale)
    for text, weight in lines:
        size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX,
                               0.62 * scale, weight)[0]
        cv2.putText(img, text, (x, h - int(bar / 2) + size[1] // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.62 * scale, HUD_FG, weight,
                    cv2.LINE_AA)
        x += size[0] + int(28 * scale)


def legend(img, scale, show_bays):
    """Swatches for what is ACTUALLY drawn, top-left. Not decoration -- see draw()."""
    x, y = int(24 * scale), int(24 * scale)
    row = int(34 * scale)
    pad = int(10 * scale)
    items = [(BOX, "detected car", True)]
    if show_bays:
        items += [(OCC, "bay: occupied", False), (FREE, "bay: free", False)]
    w = int(300 * scale)
    cv2.rectangle(img, (x - pad, y - pad), (x + w, y + row * len(items)),
                  HUD_BG, -1)
    for i, (col, label, boxed) in enumerate(items):
        cy = y + row * i + int(row * 0.35)
        sw = (x, cy, x + int(30 * scale), cy + int(16 * scale))
        if boxed:
            cv2.rectangle(img, sw[:2], sw[2:], col, max(2, int(3 * scale)))
        else:
            ov = img.copy()
            cv2.rectangle(ov, sw[:2], sw[2:], col, -1)
            cv2.addWeighted(ov, BAY_ALPHA, img, 1 - BAY_ALPHA, 0, img)
            cv2.rectangle(img, sw[:2], sw[2:], col, max(1, int(scale)))
        cv2.putText(img, label, (x + int(44 * scale), cy + int(15 * scale)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55 * scale, HUD_FG,
                    max(1, int(scale)), cv2.LINE_AA)


def draw(img, dets, verdicts, ring_px, pose, n, total, scale, show_bays):
    vis = img
    # Bays are drawn as translucent GROUND REGIONS, not as outlines.
    #
    # The first version outlined them, and on a street of parked cars that put a
    # rectangle round every car twice -- once for the detection, once for the bay
    # it sits in -- plus a rectangle on every empty bay. Ten cars read as twenty
    # objects and the video looked like the detector was wildly over-firing when
    # it was not (the busiest frame is 10 boxes on 10 real cars). A filled region
    # under a hollow box is unambiguous at a glance: the box is the thing found,
    # the wash is the ground it was found on.
    if show_bays and ring_px:
        overlay = vis.copy()
        drew = False
        for bay_id, ring in ring_px:
            occ = verdicts.get(bay_id)
            if occ is None:
                continue        # not fully in shot: it gets no vote, so no colour
            pts = np.array([[int(round(u)), int(round(v))] for u, v in ring],
                           dtype=np.int32)
            cv2.fillPoly(overlay, [pts], OCC if occ else FREE)
            drew = True
        if drew:
            cv2.addWeighted(overlay, BAY_ALPHA, vis, 1 - BAY_ALPHA, 0, vis)
        for bay_id, ring in ring_px:
            occ = verdicts.get(bay_id)
            if occ is None:
                continue
            pts = np.array([[int(round(u)), int(round(v))] for u, v in ring],
                           dtype=np.int32)
            cv2.polylines(vis, [pts], True, OCC if occ else FREE,
                          max(1, int(1.5 * scale)), cv2.LINE_AA)
    for d in dets:
        x0, y0, x1, y1 = (int(round(c)) for c in d["box"])
        cv2.rectangle(vis, (x0, y0), (x1, y1), BOX, max(3, int(4 * scale)))
        cv2.putText(vis, f"car {d['conf']:.2f}", (x0, max(y0 - 8, 18)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6 * scale, BOX,
                    max(1, int(2 * scale)), cv2.LINE_AA)
    lines = [
        (f"frame {n + 1}/{total}", 2),
        (f"t {pose.get('t_s', 0):.0f}s", 1),
        (f"yaw {math.degrees(pose['yaw']):.0f}deg ({pose.get('yaw_src', '?')})", 1),
        (f"{len(dets)} car(s) detected", 1),
    ]
    if show_bays:
        n_occ = sum(1 for v in verdicts.values() if v)
        lines.append((f"{n_occ}/{len(verdicts)} bays occupied this view", 1))
    hud(vis, lines, scale)
    legend(vis, scale, show_bays)
    return vis


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("stills")
    ap.add_argument("--out", default=None)
    ap.add_argument("--hold", type=float, default=1.0,
                    help="seconds each still is shown; 1.0 = real time, since "
                         "the stills were cut at 1 Hz")
    ap.add_argument("--fps", type=int, default=10,
                    help="video frame rate; each still is repeated hold*fps "
                         "times, so this changes smoothness, not speed")
    ap.add_argument("--width", type=int, default=1920)
    ap.add_argument("--model", default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "runs", "merged1", "weights", "best.pt"))
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--imgsz", type=int, default=1024)
    ap.add_argument("--cam", default="dji")
    # Bays are OFF by default. They are drawn from the Sofiaplan dataset, and on
    # this street that dataset is visibly wrong -- whole columns of bays land on
    # the tram median and the pavement while the cars are parked in rows it does
    # not cover (measured: detection-to-nearest-bay median 4.35 m, only 30%
    # inside the 3 m assignment radius, and no global shift fixes it). Drawing
    # them makes the video argue about the bay data instead of showing the
    # detector. --bays puts them back for when that IS the point.
    ap.add_argument("--bays", action="store_true",
                    help="overlay the projected parking bays (off by default)")
    ap.add_argument("--refresh", action="store_true")
    ap.add_argument("--limit", type=int, default=0)
    a = ap.parse_args()

    cam = cameras.get(a.cam)
    out = a.out or os.path.join(a.stills, "demo.mp4")
    poses = [p for p in json.load(open(os.path.join(a.stills, "poses.json"),
                                       encoding="utf-8"))
             if "yaw" in p and "x" in p]
    if a.limit:
        poses = poses[:a.limit]
    bays = so.load_all_bays()
    print(f"{len(poses)} frames, {len(bays)} bays, {cam!r}")

    dets = detections(a.stills, poses, a.model, a.conf, a.imgsz,
                      out + ".dets.json", a.refresh)

    repeat = max(1, int(round(a.hold * a.fps)))
    scale = a.width / cam.w
    size = (a.width, int(round(cam.h * scale)))
    writer = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*"mp4v"), a.fps, size)
    if not writer.isOpened():
        raise SystemExit(f"cannot open {out} for writing (codec unavailable?)")
    print(f"  {size[0]}x{size[1]} @ {a.fps} fps, each still held {a.hold}s "
          f"({repeat} video frames) -> {len(poses) * a.hold:.0f}s total")

    for n, pose in enumerate(poses):
        img = cv2.imread(os.path.join(a.stills, pose["file"]))
        if img is None:
            continue
        frame_dets = dets.get(pose["file"], [])
        scored = bay_votes_from_dets(
            [(d["box"], d["conf"]) for d in frame_dets], bays, pose, cam=cam)
        verdicts = {s["bay_id"]: s["occupied"] for s in scored}
        ring_px = []
        if a.bays:
            wanted = set(verdicts)
            for b in bays:
                if b["id"] in wanted:
                    ring_px.append((b["id"], [so.project(x, y, pose, cam=cam)
                                              for x, y in b["ring"]]))
        vis = draw(img, frame_dets, verdicts, ring_px, pose, n, len(poses),
                   1.0, a.bays)
        vis = cv2.resize(vis, size, interpolation=cv2.INTER_AREA)
        for _ in range(repeat):
            writer.write(vis)
        if (n + 1) % 20 == 0:
            print(f"    rendered {n + 1}/{len(poses)}")
    writer.release()
    mb = os.path.getsize(out) / 1e6
    print(f"\nwrote {out}  ({mb:.1f} MB, {len(poses) * a.hold:.0f}s)")
    print("Play it beside the map at http://localhost:5173 and start the paced")
    print("uplink with --rate 1.0 so the two advance together.")


if __name__ == "__main__":
    main()
