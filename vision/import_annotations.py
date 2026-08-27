"""Turn bay polygons drawn in makesense.ai into GROUND geometry.

Accepts makesense's **COCO JSON** and **VGG JSON** (polygons), its **CSV**, and its
**YOLO** export (pass the extracted directory of `<frame>.txt`). Draw POLYGONS
where you can: CSV, YOLO and VOC are rectangle-only, and a rectangle there
is axis-aligned *in the image* -- these frames are nadir at arbitrary drone yaw, so
a bay lies at an arbitrary angle and its bounding box is bigger than the bay and
points the wrong way. A rect export still locates a bay's CENTRE, so it degrades to
what `align_bays.py` does, and the importer says so out loud rather than quietly
accepting the worse geometry.

    python vision/import_annotations.py vision/data/annot_0035 --annot labels.json
    python vision/import_annotations.py vision/data/annot_0035 --annot labels.json --new-bays
    python vision/import_annotations.py --self-test

Someone draws each parking bay where it actually is, in the drone's own frames.
Every vertex is a pixel, `unproject()` turns a pixel into the point where its ray
meets z=0, and a bay's paint IS on that plane -- so a drawn outline becomes a real
ENU polygon with no scale or parallax assumption. That is the same reason
`paint_ground_control.py` uses paint rather than roofs.

Output is a `bay_corrections.py` sidecar of **replacement outlines**, not rigid
offsets: a drawn bay carries its own size and orientation, so this fixes rows whose
SHAPE is wrong, which a translate-and-rotate cannot. `data/block_bays.geojson` is
never edited.

**Draw the LANE, not the cars.** A bay is a piece of ground marked or usable for
parking, occupied or not. Outlining the cars instead makes every later occupancy
number circular -- the answer would be built into the question.

**Annotate a bay in ONE frame where you can.** The same bay appears in many frames;
duplicates are merged by ground position (`--merge-m`), and averaging two careful
outlines is fine, but it is wasted effort.

Unmatched drawings are kept, not dropped: a bay Sofiaplan never mapped is the single
most important thing this project has found, and `--new-bays` writes them out as a
GeoJSON of their own.
"""
import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import cameras                                                   # noqa: E402
import score_occupancy as so                                     # noqa: E402

sys.stdout.reconfigure(encoding="utf-8")


# ------------------------------------------------------------------ annotation IO
def parse_csv(path):
    """makesense's CSV rect export: label,x,y,w,h,image_name,image_w,image_h."""
    import csv
    out = {}
    with open(path, newline="", encoding="utf-8") as fh:
        for row in csv.reader(fh):
            if len(row) < 6:
                continue
            try:
                x, y, w, h = (float(v) for v in row[1:5])
            except ValueError:
                continue                        # a header line
            name = os.path.basename(row[5])
            out.setdefault(name, []).append(
                [(x, y), (x + w, y), (x + w, y + h), (x, y + h)])
    return out


def parse_yolo(path, stills):
    """makesense's YOLO export: a DIRECTORY of <frame>.txt, `cls cx cy w h`, all
    normalised to the image. Axis-aligned rects, same caveat as CSV.

    Image size is read from the JPEG itself rather than assumed: the normalisation
    is relative to whatever was uploaded, so a resized upload would silently scale
    every box. If the frame is missing we refuse rather than guess.
    """
    from PIL import Image
    out = {}
    names = [f for f in sorted(os.listdir(path)) if f.lower().endswith(".txt")]
    for fn in names:
        if fn.lower() in ("classes.txt", "labels.txt"):
            continue
        stem = os.path.splitext(fn)[0]
        img = None
        for ext in (".jpg", ".jpeg", ".png", ".JPG"):
            cand = os.path.join(stills, stem + ext)
            if os.path.exists(cand):
                img = cand
                break
        if img is None:
            print(f"  WARNING: no frame for {fn} in {stills} -- skipped")
            continue
        with Image.open(img) as im:
            W, H = im.size
        for line in open(os.path.join(path, fn), encoding="utf-8"):
            f = line.split()
            if len(f) < 5:
                continue
            try:
                cx, cy, w, h = (float(v) for v in f[1:5])
            except ValueError:
                continue
            x0, y0 = (cx - w / 2) * W, (cy - h / 2) * H
            x1, y1 = (cx + w / 2) * W, (cy + h / 2) * H
            out.setdefault(os.path.basename(img), []).append(
                [(x0, y0), (x1, y0), (x1, y1), (x0, y1)])
    return out


def parse_annotations(path, stills=None):
    """-> {frame_file: [[(u, v), ...], ...]}, from COCO/VGG JSON, CSV or YOLO dir."""
    if os.path.isdir(path):
        return parse_yolo(path, stills)
    if path.lower().endswith(".csv"):
        return parse_csv(path)
    doc = json.load(open(path, encoding="utf-8"))
    out = {}

    # --- COCO: images[] + annotations[].segmentation
    if isinstance(doc, dict) and "images" in doc and "annotations" in doc:
        names = {im["id"]: os.path.basename(im["file_name"]) for im in doc["images"]}
        for an in doc["annotations"]:
            seg = an.get("segmentation") or []
            if isinstance(seg, dict):        # RLE -- not something we can use
                continue
            for poly in seg:
                pts = [(poly[i], poly[i + 1]) for i in range(0, len(poly) - 1, 2)]
                if len(pts) >= 3:
                    out.setdefault(names.get(an["image_id"], "?"), []).append(pts)
            if not seg and an.get("bbox"):
                # An axis-aligned box is NOT a rotated bay, but it still locates one.
                x, y, w, h = an["bbox"]
                out.setdefault(names.get(an["image_id"], "?"), []).append(
                    [(x, y), (x + w, y), (x + w, y + h), (x, y + h)])
        return out

    # --- VGG: {"<file><size>": {filename, regions: [{shape_attributes}]}}
    if isinstance(doc, dict):
        for v in doc.values():
            if not isinstance(v, dict) or "filename" not in v:
                continue
            regions = v.get("regions") or []
            if isinstance(regions, dict):
                regions = list(regions.values())
            for r in regions:
                sa = r.get("shape_attributes", {})
                if sa.get("name") in ("polygon", "polyline"):
                    pts = list(zip(sa.get("all_points_x", []), sa.get("all_points_y", [])))
                elif sa.get("name") == "rect":
                    x, y, w, h = sa["x"], sa["y"], sa["width"], sa["height"]
                    pts = [(x, y), (x + w, y), (x + w, y + h), (x, y + h)]
                else:
                    continue
                if len(pts) >= 3:
                    out.setdefault(os.path.basename(v["filename"]), []).append(pts)
        if out:
            return out

    raise SystemExit(f"{path}: not a COCO or VGG JSON export I recognise")


# ------------------------------------------------------------------ geometry
def centroid(ring):
    return (sum(p[0] for p in ring) / len(ring),
            sum(p[1] for p in ring) / len(ring))


def to_ground(pts, pose, cam):
    ring = []
    for u, v in pts:
        g = so.unproject(u, v, pose, cam=cam)
        if g is None:
            return None
        ring.append((g[0], g[1]))
    return ring


def merge(items, max_m):
    """Cluster drawn rings whose centroids are within max_m; average each cluster."""
    used = [False] * len(items)
    out = []
    for i, (ring_i, ci, _f) in enumerate(items):
        if used[i]:
            continue
        group = [ring_i]
        used[i] = True
        for j in range(i + 1, len(items)):
            if used[j]:
                continue
            cj = items[j][1]
            if math.hypot(ci[0] - cj[0], ci[1] - cj[1]) <= max_m:
                used[j] = True
                group.append(items[j][0])
        if len(group) == 1:
            out.append((group[0], 1))
            continue
        # average vertex-wise, after aligning each ring's start to the first ring's
        n = min(len(g) for g in group)
        base = group[0][:n]
        acc = [list(p) for p in base]
        for g in group[1:]:
            g = _rotate_to_match(g[:n], base)
            for k in range(n):
                acc[k][0] += g[k][0]
                acc[k][1] += g[k][1]
        out.append(([(x / len(group), y / len(group)) for x, y in acc], len(group)))
    return out


def _rotate_to_match(ring, base):
    """Start `ring` at whichever vertex is nearest base[0], so averaging lines up."""
    best, bi = 1e18, 0
    for i in range(len(ring)):
        d = sum(math.hypot(ring[(i + k) % len(ring)][0] - base[k][0],
                           ring[(i + k) % len(ring)][1] - base[k][1])
                for k in range(len(base)))
        if d < best:
            best, bi = d, i
    return [ring[(bi + k) % len(ring)] for k in range(len(ring))]


def long_short(ring):
    e = sorted(math.hypot(ring[(i + 1) % len(ring)][0] - ring[i][0],
                          ring[(i + 1) % len(ring)][1] - ring[i][1])
               for i in range(len(ring)))
    return e[-1], e[0]


# ------------------------------------------------------------------ self-test
def self_test(stills, cam_name):
    """Project real bays into real frames, read them back, and demand they return.

    This is the gate. A drawn outline that unprojects wrongly does not look wrong --
    it produces a confident, plausible, wrong bay map, exactly the failure this
    project has already been bitten by twice.
    """
    cam = cameras.get(cam_name)
    w, h, _ = so._intrinsics(cam)
    poses = json.load(open(os.path.join(stills, "poses.json"), encoding="utf-8"))
    poses = [p for p in poses if "yaw" in p and "x" in p]
    bays = so.load_all_bays()
    worst = 0.0
    n = 0
    for pose in poses:
        for b in bays:
            px = [so.project(x, y, pose, cam=cam) for x, y in b["ring"]]
            if any(p is None for p in px):
                continue
            us = [p[0] for p in px]
            vs = [p[1] for p in px]
            if min(us) < 0 or max(us) >= w or min(vs) < 0 or max(vs) >= h:
                continue
            back = to_ground(px, pose, cam)
            if back is None:
                continue
            for (ax, ay), (bx, by) in zip(b["ring"], back):
                worst = max(worst, math.hypot(ax - bx, ay - by))
                n += 1
    print(f"round-tripped {n} vertices through project -> unproject")
    print(f"worst error: {worst * 1000:.6f} mm")
    if worst > 1e-6:
        sys.exit("FAIL -- a drawn polygon would not land where it was drawn")
    print("import_annotations self-test PASS")


# ------------------------------------------------------------------ main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("stills", nargs="?", help="folder from pick_frames.py")
    ap.add_argument("--annot", help="makesense export: COCO/VGG .json, .csv, or the "
                                    "extracted YOLO directory of <frame>.txt")
    ap.add_argument("--cam", default="dji")
    ap.add_argument("--merge-m", type=float, default=2.0,
                    help="drawings whose centres are within this are the same bay")
    ap.add_argument("--match-max", type=float, default=6.0,
                    help="how far a drawing may be from a Sofiaplan bay and still be it")
    ap.add_argument("--new-bays", action="store_true",
                    help="also write the drawings that match no mapped bay")
    ap.add_argument("--self-test", action="store_true")
    ap.add_argument("--out")
    a = ap.parse_args()

    if a.self_test:
        return self_test(a.stills or "pics/dji/stills/DJI_20260821163108_0035_D", a.cam)
    if not (a.stills and a.annot):
        sys.exit("need <stills> and --annot (or --self-test)")

    cam = cameras.get(a.cam)
    poses = {p["file"]: p for p in
             json.load(open(os.path.join(a.stills, "poses.json"), encoding="utf-8"))}
    drawn = parse_annotations(a.annot, a.stills)
    print(f"{sum(len(v) for v in drawn.values())} polygon(s) across "
          f"{len(drawn)} frame(s)")

    items = []
    skipped = []
    for fname, polys in drawn.items():
        pose = poses.get(fname)
        if pose is None:
            skipped.append(fname)
            continue
        for pts in polys:
            ring = to_ground(pts, pose, cam)
            if ring:
                items.append((ring, centroid(ring), fname))
    if skipped:
        print(f"  WARNING: no pose for {len(skipped)} frame(s): "
              f"{sorted(set(skipped))[:4]} -- were they renamed?")
    if not items:
        sys.exit("nothing could be unprojected -- check the frame names match poses.json")

    flat = [pts for polys in drawn.values() for pts in polys]
    axis_aligned = sum(1 for pts in flat
                       if len(pts) == 4
                       and len({round(q[0], 1) for q in pts}) == 2
                       and len({round(q[1], 1) for q in pts}) == 2)
    if flat and axis_aligned > len(flat) * 0.5:
        print(f"  WARNING: {axis_aligned}/{len(flat)} drawings are axis-aligned boxes "
              "-- the RECT tool.\n"
              "  These frames are nadir at arbitrary drone yaw, so a bay lies at an "
              "arbitrary angle in the\n"
              "  image and its bounding box is bigger than the bay and points the "
              "wrong way. The CENTRE is\n"
              "  usable, the outline is not, so this corrects where a bay SITS but not "
              "its shape.\n"
              "  Redraw with the POLYGON tool for the full correction.")

    merged = merge(items, a.merge_m)
    print(f"{len(items)} drawing(s) -> {len(merged)} distinct bay(s) after merging "
          f"within {a.merge_m} m")

    bays = so.load_all_bays()
    corr = {}
    orphans = []
    taken = set()
    for ring, votes in sorted(merged, key=lambda t: -t[1]):
        c = centroid(ring)
        best = (1e18, None)
        for b in bays:
            if str(b["id"]) in taken:
                continue
            d = math.hypot(c[0] - b["cx"], c[1] - b["cy"])
            if d < best[0]:
                best = (d, b)
        if best[1] is not None and best[0] <= a.match_max:
            bid = str(best[1]["id"])
            taken.add(bid)
            corr[bid] = {"ring": [[round(x, 3), round(y, 3)] for x, y in ring],
                         "moved_m": round(best[0], 2), "drawings": votes}
        else:
            orphans.append((ring, votes, best[0]))

    out = a.out or os.path.join(a.stills, "bay_outlines.json")
    json.dump({"source": os.path.basename(a.annot),
               "note": "replacement bay outlines in ENU metres, drawn on the frames",
               "bays": corr},
              open(out, "w", encoding="utf-8"), ensure_ascii=False, indent=1)

    moves = sorted(v["moved_m"] for v in corr.values())
    print(f"\n{len(corr)} matched a mapped bay -> {out}")
    if moves:
        print(f"  how far they moved: median {moves[len(moves) // 2]:.2f} m, "
              f"max {moves[-1]:.2f} m")
        sizes = [long_short(v["ring"]) for v in corr.values()]
        print(f"  drawn size: median {sorted(s[0] for s in sizes)[len(sizes) // 2]:.2f} x "
              f"{sorted(s[1] for s in sizes)[len(sizes) // 2]:.2f} m "
              f"(Sofiaplan's rectangles are 5.4 x 2.2)")
    print(f"{len(orphans)} matched NO mapped bay"
          + (f" (nearest was {min(o[2] for o in orphans):.1f}-"
             f"{max(o[2] for o in orphans):.1f} m away)" if orphans else ""))

    if orphans and a.new_bays:
        p = os.path.splitext(out)[0] + "_new.geojson"
        feats = []
        for i, (ring, votes, d) in enumerate(orphans):
            ll = [list(so_to_lonlat(x, y)) for x, y in ring]
            feats.append({"type": "Feature",
                          "properties": {"id": f"drawn_{i:04d}", "drawings": votes,
                                         "nearest_mapped_bay_m": round(d, 2),
                                         "source": "hand-drawn from drone imagery"},
                          "geometry": {"type": "Polygon",
                                       "coordinates": [ll + [ll[0]]]}})
        json.dump({"type": "FeatureCollection", "features": feats},
                  open(p, "w", encoding="utf-8"), ensure_ascii=False)
        print(f"  -> {p}  ({len(feats)} bays Sofiaplan does not map)")
    elif orphans:
        print("  (pass --new-bays to write them out)")

    print(f"\nuse it:  python vision/score_real.py {a.stills} --gt <gt.json> "
          f"--corrections {out} --sweep")


def so_to_lonlat(x, y):
    return so.ORIGIN[1] + x / so.MLON, so.ORIGIN[0] + y / so.MLAT


if __name__ == "__main__":
    main()
