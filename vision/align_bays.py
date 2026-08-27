"""Correct WHERE the bays are by NUDGING them, against the drone's own frames.

    python vision/align_bays.py pics/dji/stills/DJI_..._0035_D --out vision/data/align_dji_0035.html
    # ...drag each group onto its parking lane, Download JSON...
    python vision/score_real.py <stills> --gt <gt.json> --corrections vision/data/align_dji_0035.json

**The primary route is `pick_frames.py` + `import_annotations.py`** -- draw the bays
where they really are in an annotation tool and unproject the polygons. That gives a
bay its true SIZE and ORIENTATION, which nudging cannot: this file can only move
Sofiaplan's rectangle, so a row that is the wrong shape stays the wrong shape. Keep
this for the case where the published rectangles are right and merely displaced.

Sofiaplan's points are, as a population, in plausible kerbside places
(`vision/diag/bay_audit.py`), but on a given street the whole row can sit a car's
width off -- on flight 0035 бул. Джеймс Баучер's bays land on the tram tracks and
the pavement while every car is on the cobbles beside them. Labelling occupancy
against that geometry measures nothing, so the geometry is corrected first.

**The unit is a GROUP of bays that share one rigid move**, and you build the groups
by clicking rectangles in the frame. Two earlier schemes failed and are worth not
repeating:

  * *Per street.* On бул. Джеймс Баучер a street's two rows are drawn too far APART
    -- the kerbside row on the building frontage, the far row on the tram tracks,
    the carriageway between them. One translation moves both the same way, so
    fixing one breaks the other.
  * *Per row, cut into N equal-count segments along the street.* A frame shows only
    a handful of CONSECUTIVE bays out of a 33-bay row, so they nearly always land in
    the same segment and the control appears to do nothing. It also cannot express
    "these two before the crossing, those three after it", which is the actual shape
    of the error.

So a row (one street SIDE, split by which side of the OSM centerline a bay falls on
-- an outside reference, not something we are trying to prove) starts as one group,
and you split off whatever needs its own offset by clicking it into a new group.
Nothing stops you making a group of one, but resist it: an offset fitted bay by bay
can be fitted to the CARS, and then "are the cars in the bays" answers itself.

**Align to the LANE, never to the cars.** The anchor is a physical feature that is
there whether or not anyone parked today -- the kerb, the edge of the carriageway,
the tram rail, painted markings where they survive. If you slide the row until it
covers the cars, every accuracy number computed afterwards is circular. The page
says so, and it offers several frames per street precisely so the offset can be
checked where the row is EMPTY.

**Nothing is overwritten.** The output is a sidecar keyed by bay id; `data/`
keeps Sofiaplan's geometry untouched, and every consumer opts in with
`--corrections`. That keeps the correction falsifiable and reversible, the same
posture as street closures overriding the camera at read time rather than in
`bay_state`.
"""
import argparse
import base64
import io
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import cameras                                                   # noqa: E402
import score_occupancy as so                                     # noqa: E402

sys.stdout.reconfigure(encoding="utf-8")

from PIL import Image                                            # noqa: E402

DATA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "data")


def jacobian(pose, cam, w, h):
    """d(pixel)/d(ENU metre) at the frame centre, as [[dudx, dudy], [dvdx, dvdy]].

    The frames are nadir at ~30 m over a patch tens of metres across, so over the
    few metres a correction moves, the projection is linear to well under a pixel.
    That is what lets the browser redraw a dragged row without a camera model.
    """
    g = so.unproject(w / 2.0, h / 2.0, pose, cam=cam)
    if not g:
        return None
    p0 = so.project(g[0], g[1], pose, cam=cam)
    px = so.project(g[0] + 1.0, g[1], pose, cam=cam)
    py = so.project(g[0], g[1] + 1.0, pose, cam=cam)
    if not (p0 and px and py):
        return None
    return [[px[0] - p0[0], py[0] - p0[0]], [px[1] - p0[1], py[1] - p0[1]]]


def row_labels(bays, street):
    """{bay_id: "<street> - <side>"} -- which kerb of its street each bay sits on.

    A street's two rows are separate objects with separate errors (see the module
    docstring), and the split has to come from something independent: the sign of
    the cross product with the nearest OSM centerline segment. The label is turned
    into a compass point so it reads as a place rather than as a sign convention --
    OSM digitisation order is arbitrary, so "+" would mean opposite kerbs on two
    ways of the same street.
    """
    segs = _road_segs()
    out = {}
    for b in bays:
        st = street.get(str(b["id"]), "?")
        sn = _norm(st)
        # Prefer a segment of the bay's OWN street. The geometrically nearest road
        # at a junction is often the cross street, and taking it puts the two kerbs
        # of a N-S street on the "N" and "S" sides -- the same trap
        # make_bays.bearing_from_roads() avoids by name-matching first.
        best = (1e9, None)
        best_named = (1e9, None)
        for ax, ay, bx, by, nm in segs:
            dx, dy = bx - ax, by - ay
            L2 = dx * dx + dy * dy
            if L2 == 0:
                continue
            t = max(0.0, min(1.0, ((b["cx"] - ax) * dx + (b["cy"] - ay) * dy) / L2))
            qx, qy = ax + t * dx, ay + t * dy
            d = math.hypot(b["cx"] - qx, b["cy"] - qy)
            if d < best[0]:
                best = (d, (b["cx"] - qx, b["cy"] - qy))
            if nm and sn and (nm in sn or sn in nm) and d < best_named[0]:
                best_named = (d, (b["cx"] - qx, b["cy"] - qy))
        if best_named[1] is not None and best_named[0] <= 40.0:
            best = best_named
        if best[1] is None or best[0] < 0.5:
            out[str(b["id"])] = st
            continue
        ox, oy = best[1]
        side = ("N" if oy > 0 else "S") if abs(oy) >= abs(ox) else ("E" if ox > 0 else "W")
        out[str(b["id"])] = f"{st} - {side}"
    return out


_SEGS = None


def _road_segs():
    global _SEGS
    if _SEGS is None:
        _SEGS = []
        for ft in json.load(open(os.path.join(DATA, "block_roads.geojson"),
                                 encoding="utf-8"))["features"]:
            nm = _norm(ft["properties"].get("name"))
            pts = [so.to_enu(c[0], c[1]) for c in ft["geometry"]["coordinates"]]
            for i in range(len(pts) - 1):
                _SEGS.append((pts[i][0], pts[i][1], pts[i + 1][0], pts[i + 1][1], nm))
    return _SEGS


def _norm(s):
    """Same conservative form make_bays._norm_street uses."""
    s = (s or "").lower()
    for pre in ("бул.", "ул.", "пл."):
        s = s.replace(pre, " ")
    return " ".join(s.split())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("stills")
    ap.add_argument("--cam", default="dji")
    ap.add_argument("--frames-per-street", type=int, default=4)
    ap.add_argument("--width", type=int, default=1280, help="displayed frame width")
    ap.add_argument("--out")
    a = ap.parse_args()

    cam = cameras.get(a.cam)
    W, H, _ = so._intrinsics(cam)
    poses = json.load(open(os.path.join(a.stills, "poses.json"), encoding="utf-8"))
    poses = [p for p in poses if "yaw" in p and "x" in p]
    bays = so.load_all_bays()
    area = os.path.basename(a.stills.rstrip("/\\"))

    street = {str(f["properties"]["id"]): (f["properties"].get("mestopoloz") or "?")
              for f in json.load(open(os.path.join(DATA, "block_bays.geojson"),
                                      encoding="utf-8"))["features"]}
    row_of = row_labels(bays, street)

    # ---- which bays does each frame see, and how well centred are they
    seen = {}
    for pose in poses:
        vis = []
        for b in bays:
            ring = [so.project(x, y, pose, cam=cam) for x, y in b["ring"]]
            if any(p is None for p in ring):
                continue
            us = [p[0] for p in ring]
            vs = [p[1] for p in ring]
            if min(us) < 0 or max(us) >= W or min(vs) < 0 or max(vs) >= H:
                continue
            vis.append((b, ring))
        if vis:
            seen[pose["file"]] = (pose, vis)

    groups = {}
    for fname, (pose, vis) in seen.items():
        per_street = {}
        for b, ring in vis:
            per_street.setdefault(row_of.get(str(b["id"]), "?"), []).append((b, ring))
        for st, items in per_street.items():
            # `items` ranks the frame for this row; `vis` is what gets drawn, so the
            # OTHER row stays on screen (dimmed) while this one is dragged. Without
            # it you cannot see that a street's two rows are too far apart, which is
            # the error this tool exists to fix.
            groups.setdefault(st, []).append((fname, pose, items, vis))
    for st in groups:
        groups[st].sort(key=lambda t: -len(t[2]))          # most of THIS row in shot first
        groups[st] = groups[st][:a.frames_per_street]

    street_ids = {}
    for _fname, (_pose, vis) in seen.items():
        for b, _ring in vis:
            street_ids.setdefault(row_of.get(str(b["id"]), "?"), set()).add(str(b["id"]))

    print(f"{len(poses)} poses, {sum(len(v) for _, v in seen.items())} frames with bays")
    for st, fr in groups.items():
        print(f"  {st}: {len(street_ids.get(st, ()))} bays, {len(fr)} frames "
              f"for dragging (best shows {len(fr[0][2])} of this row)")

    # The correction's SCOPE (street_ids, above) is every bay this flight saw in that
    # row -- not just the ones visible in the frames chosen for dragging. A rigid
    # offset measured on 5 bays legitimately applies to the row they belong to; it
    # does NOT apply to the rest of a 1 km street the drone never overflew, so the
    # scope stops at the flight's own footprint and is written into the sidecar.
    scale = a.width / float(W)
    out_streets = []
    for st, frames in groups.items():
        ids = sorted(street_ids.get(st, set()))
        allb = [b for b in bays if str(b["id"]) in ids]
        cx = sum(b["cx"] for b in allb) / len(allb)
        cy = sum(b["cy"] for b in allb) / len(allb)
        fr_out = []
        for fname, pose, items, vis in frames:
            J = jacobian(pose, cam, W, H)
            if J is None:
                continue
            img = Image.open(os.path.join(a.stills, fname)).convert("RGB")
            img = img.resize((a.width, int(H * scale)), Image.LANCZOS)
            buf = io.BytesIO()
            img.save(buf, "JPEG", quality=80)
            fr_out.append({
                "file": fname,
                "img": base64.b64encode(buf.getvalue()).decode(),
                "w": img.width, "h": img.height,
                # J is in full-res px per metre; the page draws at `scale`
                "J": [[J[0][0] * scale, J[0][1] * scale],
                      [J[1][0] * scale, J[1][1] * scale]],
                "bays": [{"id": str(b["id"]), "g": row_of.get(str(b["id"]), "?"),
                          "enu": [[round(x, 3), round(y, 3)] for x, y in b["ring"]],
                          "px": [[round(u * scale, 1), round(v * scale, 1)]
                                 for u, v in ring]}
                         for b, ring in vis],
            })
        if fr_out:
            # Every bay of the row, so the page can group and re-centre them even
            # when a given frame shows only a few. Order is irrelevant -- groups are
            # built by clicking, not by position along the street.
            members = [{"id": str(b["id"]),
                        "cx": round(b["cx"], 3), "cy": round(b["cy"], 3)}
                       for b in allb]
            out_streets.append({"name": st, "n_bays": len(ids), "ids": ids,
                                "cx": round(cx, 3), "cy": round(cy, 3),
                                "members": members, "frames": fr_out})

    out = a.out or os.path.join("vision", "data", f"align_{area}.html")
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    stem = os.path.splitext(os.path.basename(out))[0]
    html = (PAGE.replace("__DATA__", json.dumps(out_streets, ensure_ascii=False))
                .replace("__AREA__", area).replace("__STEM__", stem))
    open(out, "w", encoding="utf-8").write(html)
    # A broken quote in the PAGE literal yields a file that opens fine and does
    # nothing -- the script dies at parse time and the page stays blank, while this
    # generator exits 0. Shipped exactly that once (2026-08-26), so the page is now
    # parsed before it is announced as written.
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "diag"))
    import check_page_js
    if check_page_js.check(out):
        sys.exit("the generated page does NOT parse -- see the error above")
    print(f"\nwrote {out}  ({os.path.getsize(out) / 1e6:.1f} MB, "
          f"{len(out_streets)} streets)")
    print("open it; drag each group onto its parking lane, clicking rectangles into "
          "a new group where one offset does not fit")
    print(f"then Download JSON -> {stem}.json")


PAGE = """<!doctype html><html><head><meta charset="utf-8">
<title>PARKDRONE - bay alignment: __AREA__</title>
<style>
 :root{--bg:#12141a;--fg:#e8eaf0;--mut:#8b93a7;--acc:#2f6feb;--warn:#e0a33e}
 *{box-sizing:border-box}
 body{margin:0;background:var(--bg);color:var(--fg);
      font:14px/1.5 ui-sans-serif,system-ui,"Segoe UI",sans-serif}
 header{position:sticky;top:0;z-index:9;background:#0d0f14;border-bottom:1px solid #262b36;
        padding:10px 16px;display:flex;gap:12px;align-items:center;flex-wrap:wrap}
 h1{font-size:15px;margin:0;font-weight:650;white-space:nowrap}
 button{background:#1c2029;color:var(--fg);border:1px solid #333a48;border-radius:7px;
        padding:6px 12px;cursor:pointer;font:inherit}
 button:hover{background:#252b36}
 button.on{background:var(--acc);border-color:var(--acc)}
 button.pri{background:var(--acc);border-color:var(--acc)}
 .row{display:flex;gap:8px;align-items:center;flex-wrap:wrap;padding:6px 16px}
 .mut{color:var(--mut)}
 .off{font-variant-numeric:tabular-nums;font-weight:650;
      background:#1c2029;border-radius:7px;padding:6px 12px;white-space:nowrap}
 .warn{color:var(--warn);font-size:12.5px;padding:2px 16px 4px;max-width:1150px}
 .swatch{display:inline-block;width:10px;height:10px;border-radius:3px;margin-right:6px}
 canvas{display:block;margin:8px 16px 40px;border-radius:10px;cursor:grab;
        max-width:calc(100vw - 32px)}
 canvas.drag{cursor:grabbing}
 kbd{background:#232834;border:1px solid #39414f;border-radius:4px;
     padding:0 5px;font:12px ui-monospace,monospace}
</style></head><body>
<header>
 <h1>bay alignment &middot; __AREA__</h1>
 <span id="streets" class="row" style="padding:0"></span>
 <span style="flex:1"></span>
 <span class="off" id="off"></span>
 <button onclick="reset()">reset group</button>
 <button class="pri" onclick="dl()">Download JSON</button>
</header>
<div class="warn"><b>Align to the LANE, not to the cars.</b>
 Use the kerb, the carriageway edge, a tram rail, or surviving paint &mdash; something that is there
 whether or not anyone parked today. Sliding bays onto the cars makes every later accuracy number
 circular.</div>
<div class="row">
 <span class="mut">group:</span><span id="groups"></span>
 <button onclick="newGroup()">+ new group</button>
 <button onclick="clearGroups()">clear row</button>
 <span class="mut" style="margin-left:10px">
  <b>Click a rectangle</b> to move it into the active group. Drag (or arrow keys) moves the whole
  active group. Use this when one offset will not fit a whole row &mdash; e.g. the bays before and
  after a crossing.</span>
</div>
<div class="row">
 <span class="mut">frame:</span><span id="frames"></span>
 <span class="mut" style="margin-left:12px">drag, or <kbd>&larr;&uarr;&darr;&rarr;</kbd> 0.1 m
  &middot; <kbd>shift</kbd> 1 m &middot; <kbd>[</kbd> <kbd>]</kbd> rotate 0.5&deg;</span>
</div>
<canvas id="c"></canvas>
<script>
const S = __DATA__, AREA = "__AREA__", STEM = "__STEM__";
const KEY = "parkdrone_align2_" + AREA;
const COL = ["#2ee06a", "#ffa83c", "#4fd6ff", "#ff74d0", "#ffe14f", "#a98bff"];
let ST = {};
try { ST = JSON.parse(localStorage.getItem(KEY) || "{}"); } catch (e) { ST = {}; }
ST.off = ST.off || {};      // "row#g" -> {dx,dy,rot}
ST.grp = ST.grp || {};      // bayId -> group index (missing = 0)
ST.ng = ST.ng || {};        // row -> number of groups
let si = 0, fi = 0, gi = 0;
const cv = document.getElementById("c"), cx = cv.getContext("2d");
const imgs = new Map();
const ROW = {}; for (const st of S) ROW[st.name] = st;

function cur() { return S[si]; }
function NG(row) { return Math.max(1, ST.ng[row] || 1); }
function grpOf(id) { return ST.grp[id] || 0; }
function members(row, g) { return ROW[row].members.filter(m => grpOf(m.id) === g); }
function centre(row, g) {
  const ms = members(row, g);
  if (!ms.length) return { cx: ROW[row].cx, cy: ROW[row].cy };
  return { cx: ms.reduce((a, m) => a + m.cx, 0) / ms.length,
           cy: ms.reduce((a, m) => a + m.cy, 0) / ms.length };
}
function offOf(row, g) {
  const k = row + "#" + g;
  if (!ST.off[k]) ST.off[k] = { dx: 0, dy: 0, rot: 0 };
  return ST.off[k];
}
function curOff() { return offOf(cur().name, gi); }
function save() { try { localStorage.setItem(KEY, JSON.stringify(ST)); } catch (e) {} }

function img(fr) {
  if (!imgs.has(fr.file)) {
    const im = new Image(); im.onload = draw;
    im.src = "data:image/jpeg;base64," + fr.img; imgs.set(fr.file, im);
  }
  return imgs.get(fr.file);
}

// A bay's drawn outline: rigid move in ENU, then the frame's local linear map.
// Used for BOTH drawing and hit-testing, so a click always means what it looks like.
function xform(fr, b) {
  const g = grpOf(b.id), o = offOf(b.g, g), C = centre(b.g, g);
  const th = o.rot * Math.PI / 180, c = Math.cos(th), s = Math.sin(th);
  const out = [];
  for (let i = 0; i < b.enu.length; i++) {
    const ex = b.enu[i][0] - C.cx, ey = b.enu[i][1] - C.cy;
    const rx = c * ex - s * ey + C.cx + o.dx, ry = s * ex + c * ey + C.cy + o.dy;
    const dx = rx - b.enu[i][0], dy = ry - b.enu[i][1];
    out.push([b.px[i][0] + fr.J[0][0] * dx + fr.J[0][1] * dy,
              b.px[i][1] + fr.J[1][0] * dx + fr.J[1][1] * dy]);
  }
  return out;
}
function inside(pts, x, y) {
  let on = false;
  for (let i = 0, j = pts.length - 1; i < pts.length; j = i++) {
    if ((pts[i][1] > y) !== (pts[j][1] > y) &&
        x < (pts[j][0] - pts[i][0]) * (y - pts[i][1]) / (pts[j][1] - pts[i][1]) + pts[i][0])
      on = !on;
  }
  return on;
}

function draw() {
  const st = cur(), fr = st.frames[fi];
  cv.width = fr.w; cv.height = fr.h;
  const im = img(fr);
  if (im.complete) cx.drawImage(im, 0, 0, fr.w, fr.h);
  else { cx.fillStyle = "#000"; cx.fillRect(0, 0, fr.w, fr.h); }

  cx.font = "600 12px system-ui";
  for (const b of fr.bays) {
    const sameRow = b.g === st.name, g = grpOf(b.id), active = sameRow && g === gi;
    const pts = xform(fr, b);
    cx.lineWidth = active ? 3.5 : sameRow ? 2 : 1.5;
    cx.strokeStyle = sameRow ? COL[g % COL.length] : "rgba(150,160,180,.4)";
    if (sameRow && !active) cx.globalAlpha = 0.65; else cx.globalAlpha = 1;
    cx.beginPath();
    pts.forEach((p, i) => i ? cx.lineTo(p[0], p[1]) : cx.moveTo(p[0], p[1]));
    cx.closePath(); cx.stroke();
    if (sameRow && NG(st.name) > 1) {
      cx.fillStyle = COL[g % COL.length];
      cx.fillText(String(g + 1), pts[0][0] + 4, pts[0][1] + 14);
    }
    cx.globalAlpha = 1;
  }
  const o = curOff();
  document.getElementById("off").textContent =
    cur().name + "  [group " + (gi + 1) + ", " + members(cur().name, gi).length + " bays]" +
    "   dx " + o.dx.toFixed(2) + " m   dy " + o.dy.toFixed(2) +
    " m   rot " + o.rot.toFixed(1) + "\u00b0";
}

function pxToEnu(du, dv) {
  const J = cur().frames[fi].J;
  const det = J[0][0] * J[1][1] - J[0][1] * J[1][0];
  return [(J[1][1] * du - J[0][1] * dv) / det, (-J[1][0] * du + J[0][0] * dv) / det];
}
let drag = null, moved = 0;
cv.addEventListener("mousedown", e => {
  drag = { x: e.offsetX, y: e.offsetY, o: Object.assign({}, curOff()) };
  moved = 0; cv.classList.add("drag");
});
addEventListener("mousemove", e => {
  if (!drag) return;
  const r = cv.getBoundingClientRect(), k = cv.width / r.width;
  const du = (e.clientX - r.left) * k - drag.x, dv = (e.clientY - r.top) * k - drag.y;
  moved = Math.max(moved, Math.abs(du) + Math.abs(dv));
  if (moved < 4) return;                      // still might be a click
  const p = pxToEnu(du, dv);
  const o = curOff(); o.dx = drag.o.dx + p[0]; o.dy = drag.o.dy + p[1];
  draw();
});
addEventListener("mouseup", e => {
  if (!drag) return;
  const wasClick = moved < 4;
  drag = null; cv.classList.remove("drag");
  if (wasClick) {
    // a click assigns the rectangle under the cursor to the active group
    const r = cv.getBoundingClientRect(), k = cv.width / r.width;
    const x = (e.clientX - r.left) * k, y = (e.clientY - r.top) * k;
    const fr = cur().frames[fi];
    for (const b of fr.bays) {
      if (b.g !== cur().name) continue;
      if (inside(xform(fr, b), x, y)) {
        ST.grp[b.id] = gi;
        break;
      }
    }
    tabs();
  }
  draw(); save();
});
addEventListener("keydown", e => {
  if (e.target.tagName === "INPUT") return;
  const o = curOff(), step = e.shiftKey ? 1.0 : 0.1;
  let hit = true;
  if (e.key === "ArrowLeft") o.dx -= step; else if (e.key === "ArrowRight") o.dx += step;
  else if (e.key === "ArrowUp") o.dy += step; else if (e.key === "ArrowDown") o.dy -= step;
  else if (e.key === "[") o.rot -= 0.5; else if (e.key === "]") o.rot += 0.5;
  else hit = false;
  if (hit) { e.preventDefault(); draw(); save(); }
});
function reset() { const o = curOff(); o.dx = o.dy = o.rot = 0; draw(); save(); }
function newGroup() {
  const row = cur().name, from = offOf(row, gi);
  ST.ng[row] = NG(row) + 1;
  const j = ST.ng[row] - 1;
  // The new group starts EMPTY but INHERITS the current offset, so a rectangle
  // clicked into it does not jump back to its uncorrected place -- you split off
  // the part that needs a different offset and refine from where you already are.
  ST.off[row + "#" + j] = { dx: from.dx, dy: from.dy, rot: from.rot };
  gi = j;
  save(); tabs(); draw();
}
function clearGroups() {
  const row = cur().name;
  for (const m of ROW[row].members) delete ST.grp[m.id];
  for (let j = 0; j < NG(row); j++) delete ST.off[row + "#" + j];
  ST.ng[row] = 1; gi = 0;
  save(); tabs(); draw();
}

function tabs() {
  const sd = document.getElementById("streets"); sd.innerHTML = "";
  S.forEach((st, i) => {
    const b = document.createElement("button");
    b.textContent = st.name + " (" + st.n_bays + ")";
    if (i === si) b.className = "on";
    b.onclick = () => { si = i; fi = 0; gi = 0; tabs(); draw(); };
    sd.appendChild(b);
  });
  const gd = document.getElementById("groups"); gd.innerHTML = "";
  for (let j = 0; j < NG(cur().name); j++) {
    const b = document.createElement("button");
    const sw = document.createElement("span");
    sw.className = "swatch";
    sw.style.background = COL[j % COL.length];
    b.appendChild(sw);
    b.appendChild(document.createTextNode(
        (j + 1) + " (" + members(cur().name, j).length + ")"));
    if (j === gi) b.className = "on";
    b.onclick = () => { gi = j; tabs(); draw(); };
    gd.appendChild(b);
  }
  const fd = document.getElementById("frames"); fd.innerHTML = "";
  cur().frames.forEach((fr, i) => {
    const b = document.createElement("button");
    b.textContent = fr.file.replace(/[^0-9]+/g, "") + " (" + fr.bays.length + ")";
    if (i === fi) b.className = "on";
    b.onclick = () => { fi = i; tabs(); draw(); };
    fd.appendChild(b); img(fr);
  });
}
function dl() {
  const out = { survey_area: AREA, aligned_at: new Date().toISOString(),
                note: "per-group rigid correction; aligned to lane features, not to cars",
                rows: {}, bays: {} };
  for (const st of S) {
    out.rows[st.name] = { groups: NG(st.name), n_bays: st.n_bays, offsets: [] };
    for (let j = 0; j < NG(st.name); j++) {
      const o = offOf(st.name, j), C = centre(st.name, j), ms = members(st.name, j);
      out.rows[st.name].offsets.push({ group: j, n_bays: ms.length,
        dx: +o.dx.toFixed(3), dy: +o.dy.toFixed(3), rot_deg: +o.rot.toFixed(2),
        cx: +C.cx.toFixed(3), cy: +C.cy.toFixed(3) });
      for (const m of ms)
        out.bays[m.id] = { dx: +o.dx.toFixed(3), dy: +o.dy.toFixed(3),
                           rot_deg: +o.rot.toFixed(2),
                           cx: +C.cx.toFixed(3), cy: +C.cy.toFixed(3),
                           street: st.name, group: j };
    }
  }
  const blob = new Blob([JSON.stringify(out, null, 1)], { type: "application/json" });
  const a = document.createElement("a");
  a.href = URL.createObjectURL(blob); a.download = STEM + ".json"; a.click();
}
tabs(); draw();
</script></body></html>"""


if __name__ == "__main__":
    main()
