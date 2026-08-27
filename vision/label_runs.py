"""Build a hand-labelling page for CARS PER CURB RUN on a real flight.

    python vision/label_runs.py pics/dji/stills/DJI_..._0035_D \
           --out vision/data/gt_runs_0035.html
    # ...count in the browser, click "Download JSON", save it next to the html...
    python vision/score_real.py <stills> --gt-runs vision/data/gt_runs_0035.json

**Why this exists beside `label_bays.py`.** That page asks "is a car inside this
rectangle", which is the right question for a simulated world and an answerable one
wherever bay paint survives. On Джеймс Баучер it is not answerable: two independent
annotation passes over flight 0035 agreed to **0.18 m across the row** and differed
by **1.95 m along it** -- about half a bay -- because the parking surface is
unmarked cobble and nothing in the image fixes where one bay ends and the next
begins. A per-bay label there is partly recording the annotator's guess at the phase
of the row.

Counting cars on a stretch of kerb has no such ambiguity. It is also exactly what
the run layer computes, so this is ground truth for the number actually being
quoted rather than a proxy for it.

**The unit is a SEGMENT, not a whole run.** A run can be 100 m long and the camera
footprint at 30 m is 45 x 25 m, so a whole run rarely fits in one frame; segments are
cut to `SEG_M` and each gets the view where it sits nearest the image centre. Counts
sum back to a per-run total, and a segment that is never fully in shot is recorded as
UNOBSERVED rather than as zero -- the same distinction `observed_fraction` carries
through the rest of the pipeline, and for the same reason: a stretch nobody looked at
is not empty parking.

**What is exact on a tile is the ALONG extent, and only that.** The blue end caps are
the segment's `s0` and `s1`, and a car counts when its middle falls between them. The
yellow band is a wide hint at which side of the street is meant -- it is not a
boundary, because the runs inherit Sofiaplan's 3-4 m cross-street error and the cars
routinely line up along the band's edge instead of inside it. Asking a human to judge
membership by that edge would be asking them to reproduce the error.

SKIP means a human looked and could not tell -- canopy, shadow, a vehicle half under
a tree -- and is excluded from the score rather than guessed, the same discipline as
an empty-vs-missing YOLO label file.
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
import curb_runs as cr                                           # noqa: E402
import score_occupancy as so                                     # noqa: E402

sys.stdout.reconfigure(encoding="utf-8")

from PIL import Image, ImageDraw                                 # noqa: E402

SEG_M = 18.0        # segment length: comfortably inside a 45 x 25 m footprint
BAND_M = 10.0       # full width of the band drawn on the tile. Wide ON PURPOSE:
#   the runs are built from Sofiaplan's rectangles, which sit 3-4 m off the real
#   kerb across the street, so a tight band lands on the tram tracks while the cars
#   line up along its edge -- observed directly in the first render. The band marks
#   WHICH STRETCH of kerb is being counted; it does not adjudicate membership. It is
#   deliberately NOT corrected by the per-run lateral fit, because that fit comes
#   from the detector and using it here would make the ground truth depend on the
#   thing it exists to judge. 10 m still cannot reach the far row: the two published
#   rows on Джеймс Баучер are 12.1 m apart.
TILE_W = 460        # rendered tile width in px


def segments(run, seg_m=SEG_M):
    """-> [(s0, s1)] covering the run in near-equal pieces of at most seg_m."""
    L = run["length_m"]
    n = max(1, int(math.ceil(L / seg_m)))
    step = L / n
    return [(i * step, min(L, (i + 1) * step)) for i in range(n)]


def point_at(run, s):
    line, cum = run["line"], run["s"]
    i = 0
    while i < len(cum) - 2 and cum[i + 1] < s:
        i += 1
    seg = (cum[i + 1] - cum[i]) or 1.0
    t = (s - cum[i]) / seg
    return (line[i][0] + t * (line[i + 1][0] - line[i][0]),
            line[i][1] + t * (line[i + 1][1] - line[i][1]))


def band_corners(run, s0, s1, half_w):
    """The parking strip for [s0, s1] as a ground quad."""
    a, b = point_at(run, s0), point_at(run, s1)
    dx, dy = b[0] - a[0], b[1] - a[1]
    d = math.hypot(dx, dy) or 1.0
    nx, ny = -dy / d * half_w, dx / d * half_w
    return [(a[0] + nx, a[1] + ny), (b[0] + nx, b[1] + ny),
            (b[0] - nx, b[1] - ny), (a[0] - nx, a[1] - ny)]


def best_view(poses, quad, cam):
    """The frame where this ground quad is fully in shot and nearest the centre."""
    w, h, _ = so._intrinsics(cam)
    cu, cv = w / 2.0, h / 2.0
    best = None
    for pose in poses:
        px = [so.project(x, y, pose, cam=cam) for x, y in quad]
        if any(p is None for p in px):
            continue
        us = [p[0] for p in px]
        vs = [p[1] for p in px]
        if min(us) < 0 or max(us) >= w or min(vs) < 0 or max(vs) >= h:
            continue
        off = math.hypot(sum(us) / len(us) - cu, sum(vs) / len(vs) - cv)
        if best is None or off < best[2]:
            best = (pose, px, off)
    return best


def tile(img, px, tile_w=TILE_W):
    """Crop around the band and rotate it flat, so the strip reads left-to-right.

    A rotated crop is not cosmetic: a 18 m segment lying diagonally needs a huge
    square tile to contain it, most of which is other people's street. Straightened,
    the same segment is a short wide strip and the cars in it are large enough to
    count at a glance.
    """
    us = [p[0] for p in px]
    vs = [p[1] for p in px]
    mid = (sum(us) / 4.0, sum(vs) / 4.0)
    # the band's own long axis, in pixels
    ax = ((px[1][0] + px[2][0]) / 2.0 - (px[0][0] + px[3][0]) / 2.0)
    ay = ((px[1][1] + px[2][1]) / 2.0 - (px[0][1] + px[3][1]) / 2.0)
    ang = math.degrees(math.atan2(ay, ax))
    L = math.hypot(ax, ay)
    wid = math.hypot(px[0][0] - px[3][0], px[0][1] - px[3][1])

    R = int(L / 2.0 + wid + 40)
    cx = min(max(mid[0], R), img.width - R)
    cy = min(max(mid[1], R), img.height - R)
    x0, y0 = int(cx - R), int(cy - R)
    sq = img.crop((x0, y0, int(cx + R), int(cy + R)))
    # Rotate about the SEGMENT's midpoint, not the square's centre. They coincide
    # only when the square needed no clamping against the frame edge; where it did,
    # rotating about the centre swung the segment off and the tile came back with
    # black bars where the crop had run past the image.
    mx, my = mid[0] - x0, mid[1] - y0
    sq = sq.rotate(ang, resample=Image.BICUBIC, center=(mx, my))

    pad = 18
    # The tile is taller than the band on purpose. The band is centred on
    # Sofiaplan's geometry, which sits a few metres off the real kerb, so cars
    # routinely straddle its lower edge -- and a tile cropped exactly to the band
    # cut them out of the picture entirely. A human cannot count what is not shown,
    # so the tile carries ~30% of the band's width as context on each side.
    hw, hh = L / 2.0 + pad, wid / 2.0 + max(pad, wid * 0.30)
    sub = sq.crop((int(mx - hw), int(my - hh),
                   int(mx + hw), int(my + hh))).convert("RGB")

    # DIM everything outside the band. The context margin above exists so a car
    # straddling the band's edge is still visible -- but on a street with parking on
    # both sides it also shows the FAR row, and an undimmed margin gave a counter no
    # way to tell "count this" from "this is only context". Measured consequence:
    # one 17 m segment came back with 10 cars, i.e. 1.7 m per car, because the four
    # cars across the tram tracks were counted too. Visible but obviously secondary
    # is the whole job here.
    top = (sub.height - wid) / 2.0
    bot = (sub.height + wid) / 2.0
    dim = sub.point(lambda p: int(p * 0.38))
    band = sub.crop((0, int(top), sub.width, int(bot)))
    dim.paste(band, (0, int(top)))
    sub = dim

    d = ImageDraw.Draw(sub)
    lw = max(2, int(sub.height / 55))
    d.rectangle([pad, (sub.height - wid) / 2.0,
                 sub.width - pad, (sub.height + wid) / 2.0],
                outline=(255, 200, 40), width=lw)
    # the run's own axis, so which SIDE of the street is being counted stays
    # unambiguous where two rows are close together
    d.line([(pad, sub.height / 2.0), (sub.width - pad, sub.height / 2.0)],
           fill=(255, 200, 40), width=max(1, lw // 2))
    # end caps: the ALONG extent is the part of this tile that is exact, and it is
    # the only part the count depends on
    for x in (pad, sub.width - pad):
        d.line([(x, 0), (x, sub.height)], fill=(60, 220, 255), width=lw)
    if sub.width > 4 and sub.height > 4:
        sub = sub.resize((tile_w, max(1, int(tile_w * sub.height / sub.width))),
                         Image.LANCZOS)
    buf = io.BytesIO()
    sub.save(buf, "JPEG", quality=80)
    return base64.b64encode(buf.getvalue()).decode()


PAGE = """<!doctype html><html><head><meta charset="utf-8">
<title>PARKDRONE - run ground truth: __AREA__</title>
<style>
 :root{--bg:#12141a;--fg:#e8eaf0;--mut:#8b93a7;--ok:#2ea043;--skip:#6b7280;--acc:#e0a020}
 *{box-sizing:border-box}
 body{margin:0;background:var(--bg);color:var(--fg);
      font:14px/1.5 ui-sans-serif,system-ui,"Segoe UI",sans-serif}
 header{position:sticky;top:0;z-index:9;background:#0d0f14;border-bottom:1px solid #262b36;
        padding:12px 18px;display:flex;gap:18px;align-items:center;flex-wrap:wrap}
 h1{font-size:15px;margin:0;font-weight:650;letter-spacing:.02em}
 .tally{color:var(--mut);font-variant-numeric:tabular-nums}
 .tally b{color:var(--fg)}
 button{background:#1c2029;color:var(--fg);border:1px solid #333a48;border-radius:7px;
        padding:7px 13px;cursor:pointer;font:inherit}
 button:hover{background:#252b36}
 button.pri{background:#2f6feb;border-color:#2f6feb}
 .hint{color:var(--mut);font-size:12.5px;padding:10px 18px 0;max-width:1100px}
 .hint b{color:var(--fg)}
 .grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(300px,1fr));
       gap:14px;padding:14px 18px 90px}
 .cell{border:2px solid #2a3040;border-radius:9px;overflow:hidden;background:#181c24}
 .cell img{width:100%;display:block;cursor:pointer}
 .cell[data-done="1"]{border-color:var(--ok)}
 .cell[data-done="skip"]{border-color:var(--skip);opacity:.55}
 .bar{display:flex;align-items:center;gap:8px;padding:6px 8px;
      font-size:11.5px;color:var(--mut);font-variant-numeric:tabular-nums}
 .bar .who{flex:1;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
 .n{min-width:2.2em;text-align:center;font-size:15px;font-weight:700;color:var(--fg)}
 .cell[data-done] .n{color:var(--acc)}
 .bar button{padding:2px 9px;border-radius:6px}
</style></head><body>
<header>
 <h1>cars per curb run &middot; __AREA__</h1>
 <span class="tally" id="t"></span>
 <span style="flex:1"></span>
 <button onclick="setAll(0)">all zero</button>
 <button onclick="setAll(null)">clear</button>
 <button class="pri" onclick="dl()">Download JSON</button>
</header>
<p class="hint">Count the cars <b>parked along this stretch of kerb</b>, between the two
 <b style="color:#3cdcff">blue end lines</b>, on the side of the street the yellow line runs
 down. The yellow band is <b>deliberately wide and is not a boundary</b> &mdash; Sofiaplan's
 geometry sits a few metres off the real kerb, so the cars often line up along the band's
 edge rather than inside it. What is exact is the <b>along-street extent</b>: the blue lines.
 Count a car if its <b>middle</b> falls between them. A car in the moving traffic lane, or on
 the far side of the street, is not parking here. Click the image to add one,
 <b>Only count cars in the BRIGHT strip.</b> Everything outside it is dimmed &mdash; it is
 context so you can see a car that straddles the edge, and this street has parking on both
 sides of the tram tracks, so the dimmed area often holds a whole second row that is
 <b>not</b> yours to count. A car half in the bright strip counts if its <b>middle</b> is inside.
 <b>Shift-click to subtract</b>. Use <b>skip</b> when you genuinely cannot tell (canopy,
 shadow) &mdash; skipped segments are excluded from the score, not counted as zero.
 Labels are kept in this browser as you go; click Download when finished.</p>
<div class="grid" id="g"></div>
<script>
const SEGS = __DATA__, AREA = "__AREA__", STEM = "__STEM__";
const KEY = "parkdrone_gtruns_" + AREA;
let V = {};
try { V = JSON.parse(localStorage.getItem(KEY) || "{}"); } catch (e) { V = {}; }

function tally() {
  let done = 0, cars = 0, sk = 0;
  for (const s of SEGS) { const v = V[s.id];
    if (v === "skip") { sk++; done++; }
    else if (typeof v === "number") { cars += v; done++; } }
  document.getElementById("t").innerHTML =
    `<b>${done}</b>/${SEGS.length} segments &nbsp;&middot;&nbsp; ` +
    `<b>${cars}</b> cars counted &nbsp;<b>${sk}</b> skipped`;
}
function save(){ try{ localStorage.setItem(KEY, JSON.stringify(V)); }catch(e){} tally(); }
function paint(el, id) {
  const v = V[id];
  const n = el.querySelector(".n");
  if (v === "skip") { el.dataset.done = "skip"; n.textContent = "-"; }
  else if (typeof v === "number") { el.dataset.done = "1"; n.textContent = v; }
  else { delete el.dataset.done; n.textContent = "?"; }
}
function bump(id, el, d) {
  const v = typeof V[id] === "number" ? V[id] : 0;
  const nx = v + d;
  if (nx < 0) delete V[id]; else V[id] = nx;
  paint(el, id); save();
}
function setAll(v) {
  for (const s of SEGS) { if (v === null) delete V[s.id]; else V[s.id] = v; }
  document.querySelectorAll(".cell").forEach(el => paint(el, el.dataset.id));
  save();
}
function dl() {
  const out = { survey_area: AREA, labelled_at: new Date().toISOString(),
                seg_m: __SEGM__, band_m: __BANDM__, segments: {} };
  for (const s of SEGS) if (V[s.id] !== undefined)
    out.segments[s.id] = { run_id: s.run_id, s0: s.s0, s1: s.s1,
                           count: V[s.id] === "skip" ? null : V[s.id],
                           skip: V[s.id] === "skip" };
  const blob = new Blob([JSON.stringify(out, null, 1)], { type: "application/json" });
  const a = document.createElement("a");
  a.href = URL.createObjectURL(blob); a.download = STEM + ".json"; a.click();
}
const g = document.getElementById("g");
for (const s of SEGS) {
  const el = document.createElement("div");
  el.className = "cell"; el.dataset.id = s.id;
  el.innerHTML = `<img src="data:image/jpeg;base64,${s.img}">
    <div class="bar"><span class="who">${s.run_id} &middot; ${s.street}</span>
      <button>&minus;</button><span class="n">?</span><button>+</button>
      <button class="sk">skip</button></div>`;
  const bs = el.querySelectorAll(".bar button");
  el.querySelector("img").onclick = (e) => bump(s.id, el, e.shiftKey ? -1 : 1);
  bs[0].onclick = () => bump(s.id, el, -1);
  bs[1].onclick = () => bump(s.id, el, 1);
  bs[2].onclick = () => { V[s.id] = V[s.id] === "skip" ? undefined : "skip";
                          if (V[s.id] === undefined) delete V[s.id];
                          paint(el, s.id); save(); };
  paint(el, s.id);
  g.appendChild(el);
}
tally();
</script></body></html>"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("stills")
    ap.add_argument("--cam", default="dji")
    ap.add_argument("--runs", help="curb_runs.geojson (default data/curb_runs.geojson)")
    ap.add_argument("--seg-m", type=float, default=SEG_M)
    ap.add_argument("--band-m", type=float, default=BAND_M)
    ap.add_argument("--every", type=int, default=1, help="use every Nth pose")
    ap.add_argument("--out")
    a = ap.parse_args()

    cam = cameras.get(a.cam)
    poses = json.load(open(os.path.join(a.stills, "poses.json"), encoding="utf-8"))
    poses = [p for p in poses if "yaw" in p and "x" in p][::a.every]
    runs = cr.load(a.runs)
    print(f"{len(runs)} verified runs, {len(poses)} poses")

    items, unobserved = [], 0
    cache = {}
    for r in runs:
        for k, (s0, s1) in enumerate(segments(r, a.seg_m)):
            quad = band_corners(r, s0, s1, a.band_m / 2.0)
            got = best_view(poses, quad, cam)
            if got is None:
                unobserved += 1
                continue
            pose, px, _off = got
            if pose["file"] not in cache:
                cache[pose["file"]] = Image.open(
                    os.path.join(a.stills, pose["file"])).convert("RGB")
            items.append({
                "id": f"{r['run_id']}#{k}", "run_id": r["run_id"],
                "street": r.get("street") or "?", "s0": round(s0, 2),
                "s1": round(s1, 2), "frame": pose["file"],
                "img": tile(cache[pose["file"]], px),
            })
    print(f"{len(items)} segment(s) fully in shot; {unobserved} never observed "
          f"(recorded as UNOBSERVED, not as zero)")
    if not items:
        sys.exit("no segment is fully in shot -- wrong stills folder or runs file?")

    area = os.path.basename(os.path.normpath(a.stills))
    out = a.out or os.path.join("vision", "data", f"gt_runs_{area}.html")
    stem = os.path.splitext(os.path.basename(out))[0]
    html = (PAGE.replace("__DATA__", json.dumps(items))
                .replace("__AREA__", area)
                .replace("__STEM__", stem)
                .replace("__SEGM__", repr(a.seg_m))
                .replace("__BANDM__", repr(a.band_m)))
    os.makedirs(os.path.dirname(out) or ".", exist_ok=True)
    open(out, "w", encoding="utf-8").write(html)
    print(f"wrote {out}  ({os.path.getsize(out) / 1e6:.1f} MB)")
    print("open it, count, then Download JSON next to the html")


if __name__ == "__main__":
    main()
