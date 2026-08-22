# dji_0035 — the first hand-labelled real-footage dataset

**31 of 35 frames labelled, 108 cars.** Real DJI nadir video over the FMI block,
Sofia, flown 2026-08-21. This is the project's first ground truth that a human
drew rather than the simulator generated.

Drawing rules are in **[LABELING.md](LABELING.md)** — read that before adding to
this set. This file records what the dataset *is* and what it has already shown.

---

## Why it exists

Every label in this project up to now came free from `generate_world.py`: the
world generator placed the cars, so it knew where they were, and
`vision/dataset.py` just projected them into pixels. Perfect labels, zero cost,
and they cannot drift from the image because both come from the same placement
loop.

Real footage has no such loop. The truth has to be drawn by hand, and that makes
these 108 boxes the most expensive artefact in the repo per byte. They are
tracked in git; the JPEGs beside them are **not** (87 MB, and regenerable at any
time with `tools/dji_stills.py`).

## Source

| | |
|---|---|
| video | `pics/dji/DJI_20260821163108_0035_D.MP4`, 3840x2160 @ 59.94 fps, 136.5 s |
| telemetry | the `.SRT` beside it — lat/lon/alt **and wall-clock `captured_at`** on all 8179 frames; **no yaw, no gimbal angles** |
| stills | `tools/dji_stills.py`, 1 Hz -> 137 frames + `poses.json` |
| altitude | 30.1 m, essentially constant — the same altitude the sim is calibrated at |
| ground sample distance | **11.7 mm/px**; footprint 45.0 x 25.1 m; a 4.5 m car is ~385 px |

## Layout

```
vision/data/dji_0035/
├── images/train/   24 frames   (gitignored — regenerate with tools/dji_stills.py)
├── images/val/     11 frames
├── labels/train/   22 .txt     TRACKED
├── labels/val/      9 .txt     TRACKED
├── classes.txt     "car"
├── data.yaml       ultralytics spec
├── LABELING.md     the drawing rules
└── README.md       this file
```

Format is YOLO: one `.txt` per image, `0 cx cy w h`, normalised 0-1, single class
`car`, axis-aligned boxes. **Byte-identical in shape to `vision/dataset.py
--yolo`**, so sim frames and real frames can go into one training run with no
conversion step.

## Current state

```
[train] 24 images, 22 labelled, 2 not yet,  82 boxes, 3.7 per frame
[val]   11 images,  9 labelled, 2 not yet,  26 boxes, 2.9 per frame
total   31/35 labelled, 108 boxes, 0 format problems
```

Still unlabelled: `frame_0040`, `frame_0044` (train), `frame_0132`, `frame_0136`
(val).

Box sizes are consistent with real cars, which is the cheapest sanity check
there is: median long side **336 px in train / 294 px in val**, i.e. **3.9 m and
3.4 m** on the ground. Range 136-512 px — the small end is cars clipped by the
frame edge or half under canopy, the large end is vans and diagonal cars whose
axis-aligned box overshoots.

Validate with:

```bash
python vision/check_labels.py vision/data/dji_0035 --overlay 6
```

It checks what a hand-made file can get wrong and a generated one cannot — pixel
coordinates instead of normalised, wrong class index, zero-area boxes, boxes off
the edge, boxes too big or small to be a car — and draws the labels back onto
the frames into `check/`. Exits non-zero on any problem.

## The split is spatial, and that is load-bearing

`train` = frames 0-92, `val` = frames 96-136. **Not random, and it must not
become random.**

The flight goes out east, **doubles back over the same street**, then heads
northwest. Measured from `poses.json`: frames 55-85 sit within **3-6 m** of
frames 5-30 — the same parked cars, photographed twice. A random split would put
one photo of a car in train and a second photo of that same car in val, and the
model would score brilliantly by memorising it rather than by learning anything.

So train holds the eastward leg *and its return over the same ground*, and val
holds the northwest leg, which the drone never covered before.

## What these labels have already proved

Before they existed, the only number available for zero-shot YOLOv8m on this
footage was an eyeball estimate over three frames (~31% recall). With real
labels the same runs score properly — `vision/detect_real.py --labels`, IoU >=
0.5, over the 31 labelled frames and 108 true cars:

| | whole frame | 2x2 tiles |
|---|---|---|
| recall, class `car` | **28.7%** (31/108) | **30.6%** (33/108) |
| recall, any vehicle class | 34.3% (37/108) | 31.5% (34/108) |
| precision of `car` boxes | 56.4% (31/55) | 50.8% (33/65) |

Three things fall out, and only the labels could establish them:

- **~30% recall, ~50% precision.** Roughly a third of cars found, and half of
  what it calls a car is not one. Not a product, and not close.
- **Tiling to native scale does not rescue it** (28.7 -> 30.6%, and precision
  *drops*). Resolution is not the binding constraint; a car is ~385 px and the
  model still misses it.
- **The dominant failure has a name.** The tiled pass emits **224 `cell phone`
  detections at median confidence 0.78** — a dark car roof on pale pavement is a
  glossy rounded rectangle with a lighter inset, and COCO's best match for that
  is a phone. Same failure the sim showed as "a nadir car is a *tie* (0.19)",
  but here at high confidence, because the pixels are real.

The comparison worth keeping: on **Webots** frames the same model detects **0 of
52** cars. So the sim's 0% conflated two causes, and these labels separate them —
the renderer *was* a large part of that result, and the residual domain gap is
about viewpoint, colour and canopy.

## Known gaps in this data

- **No yaw.** The DJI SRT records position but not heading or gimbal angles, so
  `poses.json` cannot be projected to the ground yet. These labels are usable for
  **detector training** as they are; they are **not** yet usable for occupancy
  scoring, which needs a bay to project into. See the yaw-recovery problem in the
  session notes.
- **One flight, one street, one hour of one day.** Single sun angle, single
  season, heavy summer canopy. Anything trained on this alone will be brittle in
  exactly the ways `--sun`/`--shadows` was built to probe in the sim.
- **108 boxes is small.** Enough to fine-tune with sim data alongside, or to
  measure against. Not enough to train from scratch.
- **Canopy occlusion has no simulator analogue.** It is the single biggest
  occluder here and the sim cannot generate it, so it can only be learned from
  real frames like these.

## Extending it

Two more videos sit unlabelled beside this one (`..._0034_D.MP4`, 3.1 GB, and a
set of stills `..._0036`-`0059`). To add another flight:

```bash
python tools/dji_stills.py pics/dji/<video>.MP4 --hz 1
# copy every 4th frame into images/train|val, keeping the split SPATIAL
# label per LABELING.md, then:
python vision/check_labels.py vision/data/<new> --overlay 6
```

The every-4th-frame rate is deliberate: at 1 Hz the stills are 4.2 m apart
against a 25 m along-track footprint, about **6 looks at the same ground**, so
labelling every frame is mostly re-labelling the same cars. Every 4th gives ~16 m
spacing and ~1.5x overlap.
