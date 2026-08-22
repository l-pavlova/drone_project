# The occupancy classifier — how `score_occupancy.py` decides a bay is taken

Algorithm reference for the heuristic that scores every sim survey. The
constants-and-failures record lives in CLAUDE.md; this is the mechanism, with
the units spelled out, because the thresholds are bare numbers in an 8-bit
colour space and nothing in the source says so.

**Verified 2026-08-22:** both golden fixtures still reproduce exactly —
`fmi_block` 100% (TP=17 TN=25 FP=0 FN=0, 42 of 45 bays classified, 3 uncovered)
and `fmi_block_4st` 100% (TP=49 TN=62 FP=0 FN=0, 111 of 111).

---

## 1. It is not a detector, and the difference is structural

| | asked | must find the car? |
|---|---|---|
| a learned detector (YOLO) | "where are the cars in this frame?" | **yes** |
| `classify()` | "does *this rectangle* look like empty asphalt?" | **no** |

`classify()` is *told* where every bay is — the four corners come from
`data/block_bays.geojson` and are projected into pixels through the drone's
pose. It never localises anything. The geometry does the hard half of the work,
which is why it reaches 100% on worlds where a COCO detector finds nothing.

It is also the reason the two numbers are not comparable: a detector that finds
80% of cars in a photograph is solving a strictly harder problem than a
classifier that scores 100% on a rectangle it was handed.

---

## 2. Four steps

**1. Project.** For each pose, each bay's four world-metre corners go through
`project()` (position, altitude, yaw, body roll/pitch, gimbal angles — see
CLAUDE.md **Camera model**). A bay whose box falls outside the 400×240 frame is
skipped.

**2. Crop the core** — not the whole bay, a shrunken centre. The two fractions
are separate because they buy different things:

```python
CORE_W   = 0.70   # across the WIDTH: clearance from the bay's own painted outline
CORE_LEN = 0.55   # along the LENGTH: keeps a neighbour car's overhang out
```

`CORE_W` is the binding one. On a 2.2 m bay the side lines run 0.98–1.10 m off
centre, so the older `INNER = 0.78` left only 0.12 m of clearance — under 2 px at
30 m — against a measured per-frame projection error reaching ~0.6 m. Paint then
leaks into the crop and pushes a *free* bay's chroma, brightness and std toward
the occupied side. 0.70 gives 0.21 m.

Pixels within `EDGE_MARGIN = 2` px of the frame edge are dropped, and a view is
only usable if `MIN_VIS = 0.70` of the core landed inside the frame.

**3. Five threshold tests. Any one firing means "occupied".**

**4. Vote across frames.** A bay is seen from several waypoints; fuller views
outrank partial ones, ties go to the view nearest the image centre. One bad
angle is survivable.

---

## 3. The five features, and their units

All five are computed in `region_stats()` over the RGB pixels of the core crop.
**Everything is in 8-bit channel levels, 0–255.** No perceptual space, no
normalisation, no physical unit.

```python
px      = img_arr[mask].astype(np.float32)     # N × 3, RGB
mx, mn  = px.max(axis=1), px.min(axis=1)       # per-pixel brightest / dimmest channel

chroma      = (mx - mn).mean()
brightness  = px.mean()
std         = px.std()
paint_frac  = ((mn > 140) & ((mx - mn) < 45)).mean()
dark_frac   = (px.mean(axis=1) < 55).mean()
```

| feature | what it measures | unit | fires when |
|---|---|---|---|
| `core_chroma` | mean per-pixel (max channel − min channel) | **levels, 0–255** | `> 12.7` |
| `core_dark_frac` | fraction of pixels with mean < 55 | fraction 0–1 | `> 0.02` |
| `core_paint_frac` | fraction that are bright **and** near-neutral (min > 140, spread < 45) | fraction 0–1 | `> 0.15` |
| `core_brightness` | mean of all channels | **levels, 0–255** | outside `82 … 102` |
| `core_std` | std-dev of all channel values | **levels** | `> 50` |

### What `chroma` actually is

Per pixel, the brightest channel minus the dimmest, then averaged:

| pixel | R,G,B | max−min |
|---|---|---|
| neutral asphalt | 120,120,120 | **0** |
| slightly warm tarmac (typical sim ground) | 124,120,116 | **8** |
| red car | 180,40,45 | **140** |
| white roof | 230,228,232 | **4** |

So `core_chroma > 12.7` means *the core's pixels are on average more than ~13
levels away from pure grey* — about **5% of full scale**. A low bar on purpose:
asphalt is very close to neutral, so almost any painted bodywork clears it.

It measures **colourfulness, not colour** — blind to hue, so red, blue and green
all read simply as "not grey". It is essentially saturation weighted by
brightness (HSV's `S × V`, before the division by V that would normalise it
away).

It is also **blind to white and black cars**, which is not an oversight: that is
what the other four tests are for. A white roof scores chroma ≈ 4 and is caught
by `paint_frac`; a dark car is caught by `dark_frac` or by falling out of the
brightness envelope.

---

## 4. Why the thresholds are fragile

Two measured findings, both in CLAUDE.md, and both landing on `chroma`.

**`chroma` is the only load-bearing test.** `vision/diag/classifier_ablation.py`
shows dropping `dark`, `paint` or `std` costs nothing on the calibrated worlds;
dropping `chroma` costs `fmi_block_4st` **100% → 89.2%**.

**A low sun breaks it, and no normalisation fixes it.** Under `--sun 135,25`
free-bay `core_chroma` rises **11.00 → 14.00**, crossing the 12.7 line and
turning free bays into false positives: the world falls to **44.1%** with 62 FPs
and recall still 100%.

The physical mechanism matters for the write-up. A shallower sun means
proportionally more of each surface's light arrives from the **tinted ambient
sky** rather than the neutral sun. The asphalt has not changed; the *spectrum*
lighting it has. And because `max − min` is an **absolute difference in levels**
rather than a ratio, no exposure or intensity normalisation undoes it — you would
have to correct the colour, which destroys the one signal the classifier depends
on. Measured across all six worlds, the best recovery is 93.7% on the sun worlds
at the cost of taking `fmi_block_4st` from 100% to 92.8%.

Two structural reasons it cannot be rescued locally: the free-core margin to the
brightness floor is only **3.5%** (85 against a floor of 82), and the scene is
three flat tones (ground 121 = 56% of pixels, road 50, pad 85), so a local ring
reference is bimodal and a frame-wide one is composition-dependent.

**So the summary claim is:** the classifier's single load-bearing test is a
5%-of-full-scale absolute difference in an 8-bit colour space, calibrated by hand
on one world, and it has no fair form that survives a lighting change. That is
the quantitative argument for the learned detector (TODO #5, `docs/training.md`).

---

## 5. Calibration provenance

The thresholds were hand-tuned on **`fmi_block_4st` only**, against its ground
truth. The free-core envelope on that world: chroma ≤ 12.0, dark 0, paint ≤ 0.10,
brightness 84.7–99.8, std ≤ 45.5; every occupied view clears `T_CHROMA` alone
(min 13.5).

100% on `fmi_block_4st` is therefore partly self-fulfilling, and only
`fmi_block`, `fmi_block_1km` and the lighting worlds are independent evidence.
`classify()` is deliberately the **only** swap point — projection, view
selection, voting and scoring stay unchanged when it is replaced.

---

## 6. Running it

```bash
python vision/score_occupancy.py [survey_area]      # default fmi_block_4st
python vision/diag/classifier_ablation.py           # which tests earn their place
```

The scorer prints the confusion matrix, accuracy, and the coverage declaration
from `coverage.json` beside it — an uncovered bay has two very different causes
(a bad pass, or an obstacle) and the bare count cannot tell them apart.
