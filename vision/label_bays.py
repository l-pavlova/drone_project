"""Build a hand-labelling page for per-bay ground truth on a REAL flight.

    python vision/label_bays.py pics/dji/stills/DJI_..._0035_D --out vision/data/gt_0035.html
    # ...label in the browser, click "Download JSON", save it next to the html...
    python vision/score_real.py pics/dji/stills/DJI_..._0035_D --gt vision/data/gt_0035.json

There is no ground truth for any real flight, which is why no real accuracy number
exists and why the assignment radius cannot be tuned (widening it produces more
occupied bays, and nothing says whether they are the right ones). A human deciding
"is there a car in this bay" from the drone's own frames is the only source.

**One crop per bay, from its BEST view.** A bay is visible in many frames; the page
shows the frame where the bay sits closest to the image centre, because that is the
view with the least projection error and the least roof lean from neighbouring
buildings. The bay outline is drawn on the crop, so the judgement being asked is
exactly the one the scorer makes: *is a car inside this rectangle* -- not "is a car
nearby". If the rectangle is on a garden, that bay is FREE, and the label records a
real fact about the dataset rather than covering for it.

**The three answers are FREE, OCCUPIED and SKIP, and SKIP is not a missing label.**
Skip means a human looked and could not tell (canopy, shadow, half a car under a
tree). Those bays are excluded from the score rather than guessed, the same
discipline as an empty-vs-missing YOLO label file in `vision/data/*/LABELING.md`.

Everything is embedded in one self-contained HTML file -- no server, works offline,
and the labels are exported as a JSON download.
"""
import argparse
import base64
import io
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import bay_corrections                                           # noqa: E402
import cameras                                                   # noqa: E402
import score_occupancy as so                                     # noqa: E402

sys.stdout.reconfigure(encoding="utf-8")

from PIL import Image, ImageDraw                                 # noqa: E402


def best_views(poses, bays, cam):
    """{bay_id: (pose, ring_px)} -- the frame where each bay is nearest the centre."""
    w, h, _ = so._intrinsics(cam)
    cu, cv = w / 2.0, h / 2.0
    best = {}
    for pose in poses:
        for b in bays:
            ring = [so.project(x, y, pose, cam=cam) for x, y in b["ring"]]
            us = [p[0] for p in ring]
            vs = [p[1] for p in ring]
            if min(us) < 0 or max(us) >= w or min(vs) < 0 or max(vs) >= h:
                continue                    # the same "fully in shot" rule the scorer uses
            off = math.hypot(sum(us) / len(us) - cu, sum(vs) / len(vs) - cv)
            if b["id"] not in best or off < best[b["id"]][2]:
                best[b["id"]] = (pose, ring, off)
    return best


def crop_for(img, ring, pad_frac=1.1, size=260):
    us = [p[0] for p in ring]
    vs = [p[1] for p in ring]
    cx, cy = sum(us) / len(us), sum(vs) / len(vs)
    r = max(max(us) - min(us), max(vs) - min(vs)) * (0.5 + pad_frac)
    # Slide the window back inside the frame rather than letting it hang off the
    # edge: the bay itself is guaranteed in shot, but its padding is not, and a
    # crop with black bars is harder to judge than an off-centre one.
    r = min(r, img.width / 2.0, img.height / 2.0)
    cx = min(max(cx, r), img.width - r)
    cy = min(max(cy, r), img.height - r)
    box = (int(cx - r), int(cy - r), int(cx + r), int(cy + r))
    sub = img.crop(box).convert("RGB")
    d = ImageDraw.Draw(sub)
    pts = [(u - box[0], v - box[1]) for u, v in ring]
    d.line(pts + [pts[0]], fill=(255, 60, 60), width=max(2, int(sub.width / 90)))
    sub = sub.resize((size, size), Image.LANCZOS)
    buf = io.BytesIO()
    sub.save(buf, "JPEG", quality=82)
    return base64.b64encode(buf.getvalue()).decode()


PAGE = """<!doctype html><html><head><meta charset="utf-8">
<title>PARKDRONE - bay ground truth: __AREA__</title>
<style>
 :root{--bg:#12141a;--fg:#e8eaf0;--mut:#8b93a7;--free:#2ea043;--occ:#d94a3d;--skip:#6b7280}
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
 .hint{color:var(--mut);font-size:12.5px;padding:10px 18px 0}
 .grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(150px,1fr));
       gap:12px;padding:14px 18px 90px}
 .cell{border:2px solid #2a3040;border-radius:9px;overflow:hidden;background:#181c24;
       cursor:pointer;position:relative}
 .cell img{width:100%;display:block}
 .cell .lab{padding:4px 7px;font-size:11.5px;color:var(--mut);
            display:flex;justify-content:space-between;font-variant-numeric:tabular-nums}
 .cell[data-v="free"]{border-color:var(--free)}
 .cell[data-v="occ"]{border-color:var(--occ)}
 .cell[data-v="skip"]{border-color:var(--skip);opacity:.5}
 .cell[data-v] .lab{color:var(--fg)}
 .badge{position:absolute;top:6px;right:6px;padding:2px 7px;border-radius:20px;
        font-size:11px;font-weight:650;display:none}
 .cell[data-v="free"] .badge{display:block;background:var(--free)}
 .cell[data-v="occ"] .badge{display:block;background:var(--occ)}
 .cell[data-v="skip"] .badge{display:block;background:var(--skip)}
</style></head><body>
<header>
 <h1>bay ground truth &middot; __AREA__</h1>
 <span class="tally" id="t"></span>
 <span style="flex:1"></span>
 <button onclick="setAll('free')">all free</button>
 <button onclick="setAll(null)">clear</button>
 <button class="pri" onclick="dl()">Download JSON</button>
</header>
<p class="hint">Click a tile to cycle <b>free &rarr; occupied &rarr; skip &rarr; unset</b>.
 The question is <b>is a car inside the red rectangle</b> &mdash; that is exactly what the
 scorer asks. A rectangle lying on a garden is <b>free</b>. Use <b>skip</b> only when you
 genuinely cannot tell. Labels are kept in this browser as you go; click Download when done.</p>
<div class="grid" id="g"></div>
<script>
const BAYS = __DATA__, AREA = "__AREA__", STEM = "__STEM__";
const KEY = "parkdrone_gt_" + AREA;
let V = {};
try { V = JSON.parse(localStorage.getItem(KEY) || "{}"); } catch (e) { V = {}; }
const ORDER = [null, "free", "occ", "skip"];

function tally() {
  let f = 0, o = 0, s = 0;
  for (const b of BAYS) { const v = V[b.id];
    if (v === "free") f++; else if (v === "occ") o++; else if (v === "skip") s++; }
  document.getElementById("t").innerHTML =
    `<b>${f + o + s}</b>/${BAYS.length} labelled &nbsp;&middot;&nbsp; ` +
    `<b>${o}</b> occupied &nbsp;<b>${f}</b> free &nbsp;<b>${s}</b> skipped`;
}
function save() { try { localStorage.setItem(KEY, JSON.stringify(V)); } catch (e) {} tally(); }
function paint(el, id) {
  if (V[id]) { el.dataset.v = V[id]; el.querySelector(".badge").textContent =
      V[id] === "occ" ? "car" : V[id] === "free" ? "free" : "skip"; }
  else delete el.dataset.v;
}
function setAll(v) {
  for (const b of BAYS) { if (v) V[b.id] = v; else delete V[b.id]; }
  document.querySelectorAll(".cell").forEach(el => paint(el, el.dataset.id));
  save();
}
function dl() {
  const out = { survey_area: AREA, labelled_at: new Date().toISOString(),
                bays: {} };
  for (const b of BAYS) if (V[b.id]) out.bays[b.id] = V[b.id];
  const blob = new Blob([JSON.stringify(out, null, 1)], { type: "application/json" });
  const a = document.createElement("a");
  a.href = URL.createObjectURL(blob); a.download = STEM + ".json"; a.click();
}
const g = document.getElementById("g");
for (const b of BAYS) {
  const el = document.createElement("div");
  el.className = "cell"; el.dataset.id = b.id;
  el.innerHTML = `<span class="badge"></span><img src="data:image/jpeg;base64,${b.img}">
    <div class="lab"><span>${b.id}</span><span>${b.street}</span></div>`;
  el.onclick = () => {
    const i = ORDER.indexOf(V[b.id] || null);
    const nx = ORDER[(i + 1) % ORDER.length];
    if (nx) V[b.id] = nx; else delete V[b.id];
    paint(el, b.id); save();
  };
  paint(el, b.id);
  g.appendChild(el);
}
tally();
</script></body></html>"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("stills")
    ap.add_argument("--cam", default="dji")
    ap.add_argument("--every", type=int, default=1)
    ap.add_argument("--size", type=int, default=260, help="crop size in px")
    ap.add_argument("--corrections", help="align_bays.py sidecar; label against the "
                                          "CORRECTED rectangles")
    ap.add_argument("--out")
    a = ap.parse_args()

    cam = cameras.get(a.cam)
    poses = json.load(open(os.path.join(a.stills, "poses.json"), encoding="utf-8"))
    poses = [p for p in poses if "yaw" in p and "x" in p][::a.every]
    bays = so.load_all_bays()
    corr = bay_corrections.load(a.corrections)
    if corr:
        print(f"corrections: {a.corrections}")
        print(bay_corrections.summary(corr))
        bays = bay_corrections.apply_to(bays, corr)
    area = os.path.basename(a.stills.rstrip("/\\"))
    print(f"{len(poses)} poses, {len(bays)} bays, {cam!r}")

    best = best_views(poses, bays, cam)
    print(f"{len(best)} bays fully in shot at least once")

    meta = {str(b["id"]): b.get("street") or b.get("mestopoloz") or "" for b in bays}
    by_file = {}
    for bid, (pose, ring, off) in best.items():
        by_file.setdefault(pose["file"], []).append((bid, ring, off))

    items = []
    for fname in sorted(by_file):
        img = Image.open(os.path.join(a.stills, fname))
        for bid, ring, off in by_file[fname]:
            items.append({"id": str(bid), "street": meta.get(str(bid), ""),
                          "off": round(off), "frame": fname,
                          "img": crop_for(img, ring, size=a.size)})
    items.sort(key=lambda it: (it["street"], it["id"]))

    out = a.out or os.path.join("vision", "data", f"gt_{area}.html")
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    # The download must land on the name --gt expects, so it is derived from the
    # output file rather than from the stills folder (whose name is the DJI clip's).
    stem = os.path.splitext(os.path.basename(out))[0]
    html = (PAGE.replace("__DATA__", json.dumps(items))
                .replace("__AREA__", area)
                .replace("__STEM__", stem))
    open(out, "w", encoding="utf-8").write(html)
    # A broken quote in the PAGE literal yields a file that opens fine and does
    # nothing -- the script dies at parse time and the page stays blank, while this
    # generator exits 0. Shipped exactly that once (2026-08-26), so the page is now
    # parsed before it is announced as written.
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "diag"))
    import check_page_js
    if check_page_js.check(out):
        sys.exit("the generated page does NOT parse -- see the error above")
    print(f"wrote {out}  ({os.path.getsize(out) / 1e6:.1f} MB, {len(items)} bays)")
    print(f"open it in a browser, label, then Download JSON -> {stem}.json")


if __name__ == "__main__":
    main()
