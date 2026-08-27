# Annotating real bay geometry for flight 0035

22 frames, chosen by `vision/pick_frames.py` as a near-minimal set covering all
**88 bays** this flight saw. Full resolution, original filenames — do **not**
resize or rename them: `import_annotations.py` matches each annotation back to its
pose by filename and unprojects using real pixel coordinates.

`poses.json` here is the subset for these frames and must travel with them.

## What to draw

Outline each **parking bay** — the piece of ground marked or usable for parking —
as a **polygon** (4 points, following the bay's real orientation). A rectangle tool
gives an axis-aligned box, which is wrong for a bay lying at an angle in the frame.

**Draw the LANE, not the cars.** A bay is a bay whether or not something is parked
in it. Outlining the cars makes every later occupancy number circular: the answer
would be built into the question. Anchor on paint where it survives, otherwise on
the kerb and the carriageway edge.

**Draw each bay in ONE frame.** The same bay appears in several frames; duplicates
are merged by ground position (`--merge-m 2.0`) so a second outline is not an
error, just wasted effort.

**Draw bays Sofiaplan does not map, too.** They are kept, not dropped — with
`--new-bays` they become their own GeoJSON. On this block four cars in five are
attributed to no mapped bay, so this is the most valuable thing in the exercise.

## Steps

1. Upload the JPEGs to <https://makesense.ai> (Object Detection, one label: `bay`).
2. Draw polygons.
3. Export → **Single file in COCO JSON** (VGG JSON also works; CSV is accepted but
   see below).

### Why polygons and not the rect tool

makesense's **CSV, YOLO and VOC** exports are rectangle-only, and a rectangle there
means *axis-aligned in the image*. These frames are nadir at arbitrary drone yaw, so
a bay lies at an arbitrary angle and its bounding box is bigger than the bay and
points the wrong way. The importer accepts CSV and **warns loudly** rather than
quietly taking the worse geometry.

Measured on this flight (same bays exported both ways, shifted a known 2.5 m):

| | polygon | rect |
|---|---|---|
| centre recovered | 2.50 m | 2.50 m |
| drawn size (median) | **5.40 x 2.20 m** — exact | 5.47 x 2.37 m |
| area inflation | — | median 1.09x, p90 1.23x, **max 1.58x** |
| short side excess | — | median +0.17 m, **max +1.08 m** |
| bays inflated >30% | — | **8 of 83** |

So a rect export is not useless here — the *centre* is exact either way, which is
all the assignment rule uses today. But the short side is what a 2.2 m bay is made
of, and a bay widened by a metre catches cars parked beside it, which shows up as
false "occupied". Polygons cost the same effort and have neither problem.
4. ```bash
   python vision/import_annotations.py vision/data/annot_0035 \
          --annot <export.json> --new-bays
   ```
5. Then label occupancy against the corrected geometry and score:
   ```bash
   python vision/label_bays.py pics/dji/stills/DJI_20260821163108_0035_D \
          --corrections vision/data/annot_0035/bay_outlines.json \
          --out vision/data/gt_dji_0035.html
   python vision/score_real.py pics/dji/stills/DJI_20260821163108_0035_D \
          --gt vision/data/gt_dji_0035.json \
          --corrections vision/data/annot_0035/bay_outlines.json --sweep
   ```

## Gates

- `python vision/import_annotations.py --self-test` — 2,696 vertices round-tripped
  through `project` → `unproject`, worst error **0.000000 mm**. A drawn polygon
  that unprojects wrongly does not look wrong; it produces a confident, plausible,
  wrong bay map.
- End-to-end with a synthetic export (real bays shifted a known 2.5 m, projected to
  pixels, re-imported): recovered **median 2.50 m, max 2.50 m**, drawn size
  **5.40 × 2.20 m**, and applying it moved exactly those bays by 2.4996–2.5005 m.
