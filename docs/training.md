# Learned car detector — training record

Experiment log for the whole-frame car detector (thesis stage 9, TODO #5). Every
number here was produced on this machine and can be reproduced from the commands
in §8; where a figure is weak, the caveat is stated next to it rather than in a
footnote.

**Last updated 2026-08-22.** Sim and real results are reported separately
throughout and are never averaged.

---

## 1. Setup

| | |
|---|---|
| framework | Ultralytics 8.4.11 on PyTorch 2.10.0+cpu — **CPU only**, no CUDA |
| architecture | YOLOv8 (see §7 for why, given v9–v12 and YOLO26 are all installed) |
| footage | `pics/dji/DJI_20260821163108_0035_D.MP4`, 3840×2160 @ 59.94 fps, 136.5 s |
| flown | 2026-08-21, FMI block Sofia, **constant 30.1 m** — the sim's calibration altitude |
| stills | `tools/dji_stills.py --hz 1` → 137 frames + `poses.json` |
| GSD | **11.7 mm/px**; footprint 45.0 × 25.1 m; a 4.5 m car is **~385 px** |
| labels | YOLO txt, single class `car`, axis-aligned, normalised |

Evaluation is **IoU ≥ 0.5** against hand-drawn boxes throughout, scored by
`vision/detect_real.py --labels`. Two recalls are distinguished where relevant:
class `car` alone, and any COCO vehicle class (`car`/`truck`/`bus`/`motorcycle`)
— a car boxed correctly but labelled `truck` is a different failure from a car
not found, and only the first is fixed trivially by fine-tuning.

---

## 2. Zero-shot baseline — COCO YOLOv8m on real nadir footage

31 hand-labelled frames, 108 cars, conf 0.10.

| | whole frame | 2×2 tiles |
|---|---|---|
| recall, class `car` | 28.7% (31/108) | 30.6% (33/108) |
| recall, any vehicle class | 34.3% (37/108) | 31.5% (34/108) |
| precision of `car` boxes | 56.4% (31/55) | 50.8% (33/65) |

Note the heuristic classifier this replaces is solving a *different and easier*
problem — it is handed each bay's rectangle and only judges whether it looks like
empty asphalt, so its 100% and a detector's recall are not comparable. Mechanism
and units: `docs/occupancy_classifier.md`.

**The comparison that matters: the same model detects 0 of 52 cars on Webots
frames** (`vision/detect_baseline.py`, both survey worlds). Real photographs of
the same task yield roughly a third. So the sim's 0% conflated two causes, and
**the renderer was a large part of it**; what remains is viewpoint, colour and
occlusion.

Three findings:

- **Resolution is not the binding constraint.** Tiling to native scale moves
  recall 28.7% → 30.6% and *lowers* precision. A car is ~385 px and is still
  missed.
- **The dominant failure has a name.** The tiled pass emits **224 `cell phone`
  detections at median confidence 0.78** — a dark car roof on pale pavement is a
  glossy rounded rectangle with a lighter inset, and COCO's closest match is a
  phone. The sim showed the same failure weakly ("a nadir car is a *tie*, 0.19");
  here it is confident, because the pixels are real.
- **Failures skew by body colour and tree canopy.** White and silver cars in open
  light are found; dark cars, and cars under canopy, are not. **Neither factor
  exists in the simulator**, which bounds how much sim data is worth generating.

---

## 3. Fine-tuning

Dataset `vision/data/dji_0035`: 22 train frames / 82 boxes, 9 val frames /
26 boxes. The split is **spatial, not random** — the flight doubles back over its
own street (frames 55–85 lie within 3–6 m of frames 5–30), so a random split
would score memorisation.

Common config: `yolov8s.pt`, imgsz 1024, batch 2 (= **11 optimizer steps per
epoch**), `degrees=180`, `flipud=0.5`, `fliplr=0.5`, seed 0, CPU.

Rotation and vertical flip are **0 by default in Ultralytics** and are enabled
deliberately: looking straight down there is no canonical "up", so they are
truthful expansions of a tiny dataset rather than distortions. They are the
reason 82 boxes is trainable at all.

### 3.1 `probe1` — the learning-rate failure

`optimizer="auto"`, which **ignores `lr0` entirely** and selected
`AdamW(lr=0.002)`.

| epoch | P | R | mAP50 | |
|---|---|---|---|---|
| 1 | 0.288 | 0.269 | 0.151 | warmup |
| 4 | 0.756 | 0.692 | **0.714** | **best** — last warmup epoch |
| 5 | 0.020 | 0.038 | 0.005 | **collapse**, first epoch at full LR |
| 16 | 0.003 | 0.269 | 0.003 | |
| 34 | 0.698 | 0.538 | 0.601 | early stop (patience 30) |

It peaked at epoch 4, collapsed on the first epoch after the 3-epoch warmup, and
spent 30 epochs climbing back without beating it. 35.5 min. **The default learning
rate is too large when an epoch is 11 optimizer steps.**

A second symptom of the same scale problem: validation metrics repeat in blocks
of ~3 epochs, because validation runs on the EMA weights and 11 updates per epoch
barely move the EMA. That curve must be read as a trend, never per epoch.

### 3.2 `probe2` — explicit learning rate

`optimizer=AdamW`, `lr0=0.0005` (a quarter of what `auto` chose),
`warmup_epochs=5`, `patience=40`, 80 epochs, 76.4 min.

| epoch | P | R | mAP50 |
|---|---|---|---|
| 1 | 0.716 | 0.462 | 0.522 |
| 20 | 0.918 | 0.923 | 0.891 |
| 52 | 0.915 | 0.846 | 0.900 |
| **66** | **0.947** | **0.846** | **0.921** |
| 80 | 0.733 | 0.885 | 0.884 |

No collapse; a smooth climb with a plateau from ~epoch 50. Weights:
`vision/runs/probe2/weights/best.pt`.

### 3.3 Like-for-like on the held-out split

9 val frames, 26 cars, conf 0.25, identical evaluator. **Both inference sizes are
given**, because the COCO checkpoints were trained at 640 and ours at 1024, and
quoting one number would flatter one of them:

| model | imgsz 640 R / P | imgsz 1024 R / P |
|---|---|---|
| COCO **yolov8m** | 42.3% / 73.3% | 57.7% / 75.0% |
| COCO **yolov8s** | 15.4% / 50.0% | 23.1% / 46.2% |
| **fine-tuned (probe2)** | 76.9% / 83.3% | **92.3% / 80.0%** |

The fair comparison is against **yolov8s**, the same architecture, each at its own
training size: recall **15.4% → 92.3%** from 82 training boxes.

**Correction, 2026-08-22.** Every figure here was first measured at
`detect_real.py`'s default `--imgsz 640` while both fine-tuned models were trained
at 1024 — understating them by ~15 points of recall. The superseded numbers were
yolov8m 50.0/56.5, yolov8s 19.2/50.0, probe2 80.8/63.6 at conf 0.10. **Always pass
`--imgsz` explicitly.**

### 3.4 Operating point

Ultralytics reports P/R at its best-F1 confidence, not at the 0.10 used above.
Sweeping confidence on the same 26 cars:

| conf | recall | precision |
|---|---|---|
| **0.25** | **92.3%** | **80.0%** |
| 0.40 | 76.9% | 90.9% |
| 0.60 | 15.4% | 100% |
| 0.75 | 0.0% | — |

(imgsz 1024, the training size. The earlier 640 sweep read 80.8/63.6 at conf 0.10,
76.9/83.3 at 0.25, 50.0/100 at 0.40.)

**conf 0.25 is probe2's operating point** — and note how sharply it falls away
above 0.4: at 0.6 it finds 4 cars of 26. That fragility is itself a finding, and
§5c shows the model trained on 4.6× the data does not share it.

**Caveat on all of §3.3–3.4: 26 boxes.** One car is ±3.8% of recall. The
19% → 81% gap is far too large to be noise, but the individual figures are not
precise to better than roughly ±8%. §4 supersedes them on a larger sample.

---

## 4. Pre-labelling round, and a better measurement

`vision/prelabel.py` ran `probe2/best.pt` at conf 0.25 over the 106 frames not
yet labelled. 7 were then **excluded as non-nadir** — the gimbal tilts up over the
last seconds of the flight (0130–0134 oblique, 0136 with the horizon in shot), and
nothing in the SRT signals it: `rel_alt` reads a flat 30.1 m and the file carries
no gimbal angles. They are held in `vision/data/dji_0035_r2/excluded_oblique/`.

The remaining **99 frames were corrected by hand**. None were used in training, so
comparing prediction against correction is a far larger held-out evaluation than
the 26-box val set:

| | count |
|---|---|
| true cars | 386 |
| predicted boxes | 392 |
| kept as-is (IoU ≥ 0.5) | 358 |
| — of which the box needed nudging (IoU 0.5–0.85) | 8 |
| deleted (false positive) | 34 |
| drawn (missed car) | 28 |
| **edits per frame** | **0.6**, against 3.9 cars per frame |

**Labour:** 62 corrections instead of 386 boxes drawn from scratch.

### The two halves are not equally trustworthy

| group | frames | cars | precision | recall | |
|---|---|---|---|---|---|
| `train` (frames 1–95) | 74 | 299 | 91.0% | 95.0% | **inflated — do not quote** |
| `val` (frames 97–129) | 25 | 87 | **92.5%** | **85.1%** | spatially held out |

The 95% is near-duplicate leakage: the model trained on frames 0, 4, 8 … 92, and
these sit *between* them — 4 m apart with a 25 m footprint, i.e. the same parked
cars from a slightly different angle.

**The honest figures are 85.1% recall / 92.5% precision on 87 cars.** They
supersede §3.3's 80.8%/63.6% (3.3× the sample) and revise it upward — the small
val set was pessimistic within its own noise.

---

## 5. Dataset state

| set | frames | boxes | provenance |
|---|---|---|---|
| `vision/data/dji_0035` | 31 | 108 | **hand-drawn from scratch** |
| `vision/data/dji_0035_r2` | 99 | 386 | model pre-labelled, hand-corrected |
| **total** | **130** | **494** | |

**`dji_0035` should be preserved as a clean benchmark.** The r2 labels are partly
the model's own opinion: where it was wrong it was corrected, but where it was
*confidently* wrong in a plausible way it may have carried the reviewer with it,
and retraining on that reinforces the model's own bias. The 108 original boxes are
untouched by any pre-labelling and are the only set free of that circularity.

### How spatial the split actually is

`vision/make_split.py` measures the separation instead of asserting it. For the
`--val-from 96` split used throughout, the nearest train frame to each val frame:

| val frame | nearest train frame | distance |
|---|---|---|
| 0096 | 0092 | **14.5 m** — footprints overlap |
| 0100 | 0092 | 43.1 m |
| 0104 … 0136 | 0092 | 51.8 … 96.4 m |

**One val frame of eleven is marginally contaminated.** 14.5 m is inside the
~25 m along-track footprint, so `frame_0096` and `frame_0092` photograph some of
the same tarmac; the other ten are 43–96 m clear. `--suggest` shows `>= 92` (29.2 m)
and `>= 100` (28.6 m) would both have been fully clean. Not enough to invalidate
anything, but it is the kind of claim that should be measured rather than asserted,
which is why the script now prints it and warns below 25 m.

---

## 5b. The data pipeline — which script runs at each hop

Every hop is a **file on disk**; nothing is passed in memory between stages, so
any step can be re-run or inspected on its own.

| # | hop | script | in → out |
|---|---|---|---|
| 1 | video → stills + poses | **`tools/dji_stills.py`** | `<flight>.MP4` + `.SRT` → `pics/dji/stills/<flight>/frame_####.jpg` + `poses.json` |
| 1a | └ SRT parsing | **`tools/dji_srt.py`** (imported, also a CLI) | subtitle blocks → `{x, y, alt, lat, lon, captured_at}`; `rebase()`/`--as-now` for replay |
| 2 | pick frames + split | **`vision/make_split.py`** | stills → `vision/data/<name>/images/{train,val}` + `split.json` (every Nth, **spatial** split, separation measured) |
| 3 | draw boxes | **makesense.ai** (external, browser) | images → YOLO `.txt` per image |
| 3b | screen for nadir | **`vision/check_nadir.py`** | stills dir → which frames are oblique. **Run before hop 2** — flight 0034 was cut, split and pre-labelled before anyone noticed it was oblique end to end |
| 4 | validate | **`vision/check_labels.py`** | dataset dir → format check + boxes redrawn into `check/` |
| 5 | zero-shot baseline | **`vision/detect_real.py --labels`** | model + frames + labels → recall/precision at IoU ≥ 0.5 |
| 6 | fine-tune | **`vision/train_detector.py`** | `data.yaml` → `vision/runs/<name>/weights/best.pt` |
| 7 | pre-label ONE split | **`vision/prelabel.py`** | `best.pt` + a dataset's `images/<split>` → predictions as `.txt` in `labels/<split>`. Never `val`: pre-labelling the frames you measure against bakes the model's opinion into its own yardstick |
| 8 | correct | **makesense.ai** | predictions → corrected `.txt` (overwrites `labels/`) |
| 9 | validate again | **`vision/check_labels.py`** | as (4) |
| 10 | merge sets | **`vision/merge_datasets.py`** | labelled sets → `vision/data/dji_merged` + `provenance.json` (splits preserved, hand-drawn frames flagged) |
| 11 | retrain | **`vision/train_detector.py`** | merged `data.yaml` → `runs/merged1/` |

Sim-side equivalents, for contrast: **`vision/dataset.py`** replaces hops 1–3
entirely (labels come free from the world generator) and
**`vision/detect_baseline.py`** is hop 5 for Webots frames.

### How Ultralytics finds the labels

Nothing points at the label folder. Ultralytics takes each image path and swaps
`images` → `labels` and `.jpg` → `.txt`:

```
images/train/frame_0000.jpg   →   labels/train/frame_0000.txt
```

That is why the folder must be named exactly `labels` beside `images`, and why a
mistake there surfaces as "no labels found" rather than as a useful error.

### What a label line becomes

```
0 0.316274 0.138365 0.047170 0.136268
```

class 0, normalised centre/size → scaled to pixels at `imgsz` → **augmented with
the image** (a `degrees=180` rotation rotates the box and re-fits an axis-aligned
one) → matched against predictions → `box_loss` / `cls_loss` / `dfl_loss`.

An **empty** `.txt` is read as a *verified-empty* frame and used as a negative
example (the trainer logs it as a "background"); a **missing** one means the frame
is skipped entirely. The merged train set has 2 backgrounds, which is why the
distinction was enforced at labelling time.

### The whole chain, reproducible (2026-08-22)

Hops 2 and 10 were inline session code until 2026-08-22 — the only part of the
pipeline that could not be re-run from the repo, and unluckily the part holding the
most important methodological choice. Both are now scripts, and both do more than
the inline code did:

**`vision/make_split.py`** — subset selection and the spatial split, and it
**measures the separation rather than asserting it**. `--suggest` scans every
candidate split point and reports the minimum train↔val distance each would give;
choosing a cut below the ~25 m along-track footprint prints a warning, because
below that a train frame and a val frame photograph the same tarmac. The result is
recorded in `split.json`. Running it on flight 0035 is what revealed that the
`--val-from 96` split has one marginally contaminated frame (§5).

**`vision/merge_datasets.py`** — merges labelled sets, **preserving each source's
split** (pooling and re-splitting would silently undo the spatial work) and writing
**`provenance.json`**, which records per frame where it came from and whether it was
`hand_drawn` or model-pre-labelled. `--benchmark` marks the clean sources and
`--benchmark-only-val` can force them into val. That record is what made the
probe2-vs-merged1 comparison in §5c interpretable at all.

Verified against the hand-built set: identical filenames and identical label
contents in both splits, differing only in Ultralytics' generated `.cache` files.

Both take `--prefix`, because **every flight's stills start at `frame_0000`** and a
second flight merged without one would silently collide.

---

## 5c. Retraining on the merged set (`merged1`)

`yolov8s.pt`, 96 train frames / 381 boxes, 34 val frames / 113 boxes, 26 epochs,
imgsz 1024, batch 2 (48 optimizer steps per epoch), `AdamW lr0=0.0005`, warmup 5.
**108.8 min, no collapse**, best mAP50 **0.884** at epoch 16.

That is not comparable to probe2's 0.921 — different val sets. Scored instead on
the **9 pure hand-drawn frames** (26 cars), the only labels no pre-labelling ever
touched, at imgsz 1024:

| conf | probe2 R / P | merged1 R / P |
|---|---|---|
| 0.25 | **92.3%** / 80.0% | 88.5% / 60.5% |
| 0.40 | 76.9% / 90.9% | **88.5%** / 82.1% |
| 0.60 | 15.4% / 100% | **84.6%** / 88.0% |
| 0.75 | 0.0% / — | **61.5%** / 100% |

At each model's own best operating point: **probe2 @0.25 = 92.3/80.0 (F1 0.857)**
against **merged1 @0.60 = 84.6/88.0 (F1 0.863)**. Effectively tied, trading recall
for precision.

**So 4.6× the training data did not buy a clearly better model on this benchmark.**
That is the honest reading and it should not be dressed up. Three things qualify it:

- **merged1 is far more robust to the confidence threshold.** From 0.25 to 0.60 its
  recall moves 88.5% → 84.6%; probe2's collapses 92.3% → 15.4%. More data made the
  model *confident*, not just accurate, and a model that only works inside a narrow
  confidence band is fragile in deployment. This is the reason to prefer merged1.
- **The larger comparison is biased toward probe2.** On the full 34-frame merged val
  probe2 also leads (86.7/89.1 vs 84.1/66.9 at conf 0.25) — but 25 of those 34 frames
  carry labels probe2 itself drew and a human corrected, so it is credited wherever
  it was confidently wrong and the correction agreed. Exactly the circularity §5
  warns about, and `provenance.json` is what makes it detectable.
- **26 cars cannot separate F1 0.857 from 0.863.** One car is ±3.8%.

**The real conclusion is that the benchmark is now the bottleneck.** Nine frames can
show that fine-tuning works; they cannot rank two decent models. The next honest step
is a held-out set from a *different flight* (video 0034), labelled from scratch with
no pre-labelling.

---

## 5d. Flight 0034 is unusable, and the screen that now catches it

**The whole of flight 0034 (4.7 min, 16812 frames) is OBLIQUE** — the gimbal
points forward, every frame carries sky and horizon. It was cut to 175 stills, a
measured spatial split was designed for it, and 53 frames were pre-labelled
before anyone looked at the imagery. The dataset was deleted; the stills remain
in `pics/dji/stills/`.

**Nothing in the telemetry says so.** The SRT has no gimbal angles and `rel_alt`
reads a flat ~29 m for the whole cruise, exactly as it does on the nadir flight.
This is the same trap as 0035's last seven frames, at the scale of a whole clip.

`vision/check_nadir.py` is the screen. A nadir frame's top strip is ground; an
oblique frame's is sky, which is bright and blue-dominant:

| | brightness (top 12%) | blue − red |
|---|---|---|
| nadir (0035) | 110–140 | −17 to −29 |
| oblique (0034) | 205–216 | +26 to +47 |

The populations are nowhere near each other, so a threshold is enough and a real
horizon detector would be false precision. Results: **0034 175/175 oblique**,
0035 1/137. **Run it before cutting a split, not after.**

It is a screen, not a proof, and its limits are known: it flags only the extreme
case. Of the seven 0035 tail frames excluded by eye it catches one — 0130–0135 are
tilted but not yet showing sky — and `DJI_..._0053_D.JPG` sits just under the
brightness threshold at 178.5. Use `--list` and look at anything borderline.

**The 23 standalone JPEGs** in `pics/dji/` are mostly nadir (20 of 23 pass), but
they are not a drop-in benchmark either: 4096×3072 (4:3, not the video's 16:9),
EXIF carries only absolute altitude (639–712 m, so relative height is unknown and
varies), and they sit at y −156…−34, south of everything else. Usable with work,
not for free.

**Net: this session produced no new nadir training data**, and the benchmark
bottleneck identified in §5c stands. The cheapest fix is another flight with the
gimbal locked down.

---

## 6. Known limits of this data

- **No yaw.** The DJI SRT records position and (since 2026-08-22) wall-clock
  `captured_at`, but no `gb_yaw` and no gimbal angles. Real poses therefore cannot
  be projected to the ground: this data can train a **detector**, but cannot yet
  score **occupancy**. The street is cobblestone and unmarked, so
  `vision/diag/paint_align.py` has nothing to register against — recovery will need
  building corners or crosswalks.
- **One flight, one street, one hour of one day.** Single sun angle, single season,
  heavy summer canopy.
- **Canopy occlusion has no simulator analogue** and is the biggest occluder here,
  so it can only be learned from real frames.
- **The intrinsics differ from the sim's.** 73.7° H / 45.4° V against the
  Mavic2Pro proto's 45° — a fourth copy of the camera model in the project.

---

## 7. Why YOLOv8, when v9–v12 and YOLO26 are installed

A comparability choice, not a technical one.

- The supervisor's `ground-vehicles-localization` runs `YOLO("yolov8m.pt")`
  zero-shot on real Sofia footage. That is the directly comparable prior result,
  and a number from another architecture does not address it.
- Changing architecture and training regime together would confound the one
  measurement these runs exist to make.
- v8 → v11 is worth roughly 1–3 mAP on COCO. The effect under study is a
  19% → 85% domain shift on 494 boxes. Architecture is not the bottleneck.
- **Licensing does not improve by moving within Ultralytics** — v9, v10, YOLO11,
  YOLO12, YOLO26 and Ultralytics' own RT-DETR are all AGPL-3.0. Only leaving for
  `transformers` RT-DETR / D-FINE (Apache-2.0) changes that, which is a separate
  axis (TODO #5).

An architecture sweep (`--model yolo11s.pt` etc., same script and dataset) is a
clean experiment **once the validation set is large enough for a 2-point mAP
difference to rise above noise.** It is not, yet.

**One inconsistency to fix before publication:** the zero-shot baseline in §2 is
**v8m** while the fine-tune is **v8s**. §3.3 re-measures v8s zero-shot (19.2%) so
the like-for-like exists, but §2's headline numbers should be re-run on v8s if
they are quoted beside a v8s fine-tune.

---

## 8. Reproducing

The full chain for a new flight, hop by hop (numbers match §5b):

```bash
# 1  video -> stills + poses
python tools/dji_stills.py pics/dji/<video>.MP4 --hz 1

# 2  choose the split from the geometry, THEN build the set
python vision/make_split.py pics/dji/stills/<flight> --every 4 --suggest
python vision/make_split.py pics/dji/stills/<flight> --every 4 --val-from N \
       --out vision/data/<name> --prefix f<flight>_

# 3b screen for nadir BEFORE investing any labelling effort
python vision/check_nadir.py pics/dji/stills/<flight>

# 3  label images/{train,val} in makesense.ai   (rules: LABELING.md)
# 4  validate
python vision/check_labels.py vision/data/<name> --overlay 6

# 5  baseline -- ALWAYS pass --imgsz; the 640 default is not the training size
python vision/detect_real.py vision/data/<name>/images/val --model yolov8s.pt \
       --imgsz 1024 --conf 0.25 --labels vision/data/<name>/labels/val

# 6  train
python vision/train_detector.py --data vision/data/<name>/data.yaml \
       --epochs 80 --imgsz 1024 --batch 2 --name <run>

# 7  pre-label the rest   (8 correct in makesense, 9 validate as above)
python vision/prelabel.py pics/dji/stills/<flight> \
       --model vision/runs/<run>/weights/best.pt --conf 0.25 \
       --out vision/data/<next> --prefix f<flight>_ \
       --skip vision/data/<name>/labels/train --skip vision/data/<name>/labels/val

# 10 merge, flagging which sources are hand-drawn
python vision/merge_datasets.py vision/data/<name> vision/data/<next> \
       --out vision/data/<merged> --benchmark vision/data/<name>

# 11 retrain
python vision/train_detector.py --data vision/data/<merged>/data.yaml \
       --epochs 26 --imgsz 1024 --batch 2 --name <run2>
```

Runs are seeded (`seed=0`). Training outputs are gitignored — the inputs (labels,
script, seed) are tracked, so a run is reproducible rather than archived.

---

## 2026-08-23 — the detector reaches the live map, and what the test run measured

`vision/runs/merged1/weights/best.pt` (yolov8s, 130 real frames / 366 boxes, imgsz 1024,
**mAP50 0.873 / P 0.945 / R 0.766**) is now the occupancy backend for real footage. Full
mechanism in CLAUDE.md 2c-2e; the numbers a reader of this file wants:

**On the 137 stills of flight 0035** (conf 0.25, imgsz 1024): 473 detections, **3.5 per frame**
(median 3, max 10), confidence p10 0.33 / p50 0.68 / p90 0.83, and only 20 overlapping pairs at
IoU > 0.3. The hand labels average 2.8 boxes/frame, so the model fires about 25% more often than a
human labelled — and the busiest frame, inspected directly, is **10 boxes on 10 real cars**. The
detector is not over-firing; it is finding cars in rows the labelling pass sampled at 1-in-4.

**The bays are the limiting factor, not the model.** Distance from each detection to the nearest
Sofiaplan bay centroid: **median 4.35 m** (p10 1.68, p90 11.16), with only **30% inside** the 3 m
assignment radius. A grid search over +/-6 m finds the best possible global correction reaches
median 3.47 m and 42 of 115 — at 4.5 m, about a car length, which is a fit to the structure of
parked-car spacing rather than a GPS bias. A real bias would show as a tight cluster at a small
offset; this is a broad spread. The overlays say the same thing visually: whole columns of mapped
bays sit on the tram median and the pavement while the cars are parked in rows the dataset does not
cover.

**So the honest reading of the real-world occupancy result is: the detector works, the
georeferencing works, and the BAY DATA is what limits how much of a real street this can score.**
That is a different bottleneck from the sim (where every car is in a bay by construction) and it is
the thing to say out loud when the sim's 98.4% and any real number are put side by side. It also
re-prioritises the next task: hand-labelling the ~86 bays under this flight would measure the
classifier, but the more informative measurement may be how many parked cars on this block are in
a mapped bay at all.

**Provenance is now recorded** (migration `0010`): 674 detector observations from this run, 102
carrying a `det_score` (avg 0.641), zero heuristic colour statistics. Sim and real results can
therefore never be silently averaged, which was the standing rule and is now enforced by the
schema rather than by discipline.

---

## 2026-08-25 — two more flights, a colour-channel bug, and the bay data answering back

Two new DJI clips over the same block, both ~30 m nadir but flown at **12:53 midday** where flight
0035 was 16:31: `DJI_20260825125321_0074_D` (138 s, 139 stills) and `DJI_20260825125614_0075_D`
(22 s, 23 stills). Extracted, yaw-recovered, screened and scored end to end.

**The detector generalises across the light.** On 0074's nadir frames `merged1` gives confidence
p10 0.33 / p50 0.63 / p90 0.80, against 0.33 / 0.68 / 0.83 recorded on 0035 — the same distribution
under a different sun. 0075, which flies a street of parked cars, gives **4.3 detections per frame**
against 0035's 3.5. Inspected by eye: on the 0075 frames every parked car carries a box and the one
skip is a loaded truck; on 0074 the low density (1.6/frame) is scene content, since that flight
hovers over canopy, gardens and rooftops for much of its length. No hand labels exist for either
flight, so these are *distributions and eye checks*, not a recall number.

**A colour-channel bug, found by two paths disagreeing on the same frames.** `detect_occupancy.py`
and `detect_real.py` returned 108 and 173 detections over the identical 108 nadir frames of 0074 at
identical settings. Cause: **ultralytics reads a numpy array as BGR** — the cv2 convention it trains
with — and `detect_occupancy` built its array from PIL `.convert("RGB")`, so the model saw every
real frame with red and blue swapped. Three call sites were affected and are fixed:

| file | input | was |
|---|---|---|
| `vision/detect_occupancy.py` | PIL RGB array | swapped |
| `vision/render_demo.py` | cv2 BGR, then reversed to RGB | swapped |
| `.../vision_worker/vision/scoring.py` (**the live backend**) | PIL RGB from S3/replay | swapped |

`prelabel.py` and `detect_real.py` use `cv2.imread` and were always right, which is why the
*training* labels and every reported recall/precision figure are unaffected. The live map, however,
was running the detector on colour-swapped frames. **Measured cost on flight 0035: 473 -> 599
detections (+27%)**, and `detect_occupancy` now agrees with `detect_real` to the exact count on both
new flights (100 and 173) — that agreement is the check that the fix is right. In `scoring.py` the
reversal is applied only to what the detector is handed; `img_arr` stays RGB, because that is the
contract the heuristic's crops depend on.

**The nadir screen now runs inside the occupancy pass.** 22% of flight 0074 is oblique (the gimbal
tilts to the horizon at frames 81-83 and from 111 to the end, confirmed by eye). An oblique frame
does not fail to project — it silently places its cars tens of metres away, with full confidence.
`check_nadir.is_oblique()` was split out of that script's `main()` and `detect_occupancy.run()` drops
flagged frames (`--no-screen` opts out). On 0035 this removes exactly one frame and reproduces the
88 bays already on record.

**And the bay data answers the standing question, harder than 0035 did.** Neither new flight
produces a single occupied bay — **0 of 49 on 0074, 0 of 9 on 0075, from 273 detections**. That is
not the vote failing, and 0035 is the positive control that proves it: there, an occupied bay reads
**11/13, 10/12, 7/7, 6/6** views — a car in a mapped bay is seen in nearly every look at it. On 0074
the best bay in the flight manages **6 of 21**, then 2/5, 2/7, 2/22; on 0075 all nine bays have
**zero** occupied views. That is the signature of cars parked *beside* the mapped geometry rather
than in it.

Distances agree: detection-to-nearest-bay median **8.95 m on 0075** (p10 5.90 — not one of 91
detections inside the 3 m assignment radius) and **4.81 m on 0074** (34 of 173 inside), against
4.35 m and 30% on 0035. **It is not a yaw error**: correcting yaw alone moves the 0075 median only
9.01 -> 8.19 m and 0074's 4.81 -> 4.68. A pure translation does help 0075 a lot (-7, -8 m -> median
2.69 m, 51 of 100 within 3 m), but 0074's best is a different direction (+3, -9) for much less, and
the two flights are three minutes apart — so there is no shared bias to correct, and a per-street
offset is a statement about how that street was drawn, not about our georeferencing.
The `real_align.py` overlay for 0075 shows it directly and is the picture to put in the thesis:
**painted, occupied parking bays are plainly visible in the photograph** along the kerb, and the
Sofiaplan rectangles for that frame sit on a garden and under tree canopy instead.

**CAVEAT ADDED 2026-08-26 — this conclusion is INDICATED, NOT SETTLED, and was first written too
firmly.** What is solid: the pose is self-consistent to ~1 m (the same car detected in >=3 frames
unprojects to the same ground point: median scatter 0.94 m on 0075, 1.08 m on 0074, p90 ~2.5 m), OSM
building footprints project onto their real buildings (`vision/diag/ref_align.py`, flight 0074
frame 57), and the vote is demonstrably working. What was over-claimed: "there is no shared bias to
correct". Self-consistency is **blind to a constant per-flight offset**, and the grid search says one
would take 0075 from 0 to 51 of 100 detections on-bay. The argument that the two flights want
different shifts does NOT separate a per-flight GPS bias from a per-street data offset, because the
two flights are on different streets — those two explanations are indistinguishable in that test.
Settling it needs an **absolute** check against flat ground control (painted bay markings, which
unlike a roof have no parallax); until that is done, no real-world accuracy number should be quoted
against these bays.

**Datasets built, pre-labelled, waiting on correction.** `vision/data/dji_0074` (36 frames, 60
boxes) and `vision/data/dji_0075` (23 frames, 100 boxes) — pre-labelled with `merged1` at conf 0.25,
`check_labels.py` clean, 8 deliberately-empty label files on 0074. Both are **train-only**, and that
is forced by the geometry rather than chosen: `make_split --suggest` finds **no** split point on
0074 reaching the 25 m along-track footprint, because the flight hovers and doubles back over its
own ground throughout. They are benchmarked against `dji_merged`'s hand-drawn val set, whose frames
are **68 m (0074) and 238 m (0075)** away — checked before building, since training on frames that
overlap a benchmark leaks it.

Three tool fixes fell out and are worth knowing:
- `detect_real.py` decided "is this a vehicle" by **COCO class index**, but a fine-tuned single-class
  model calls `car` index 0, which is COCO's `person`. Every per-frame count, size statistic and
  overlay colour was wrong for our own detector while `--labels` scoring (already matching on the
  name) was right — so the bug was invisible in every reported recall/precision figure. Now matched
  by name.
- `make_split.py --exclude` is repeatable: a real flight's non-nadir stretches are not one
  contiguous block. And `--suggest` ran *before* exclusions were applied, so it reported split
  points among frames that were about to be dropped.
- `make_split.py` printed "min train-val distance: inf m (>= the ~25 m footprint: the split is
  genuinely spatial)" for a train-only dataset. A vacuous `inf` must not borrow that reassurance; it
  now says there is no val split and what to benchmark against instead.

**2026-08-26 — ground control run, and the control flight settles the method.**
`vision/diag/paint_ground_control.py` unprojects painted road markings (paint is ON the ground
plane, so unlike a roof it has no parallax) into an ENU paint map, rasters the Sofiaplan bay
outlines on the same grid, and slides one over the other. `--self-test` recovers injected shifts
exactly (0.00 m on four vectors) and gates the result.

Run on both flights through the **identical** mask, so the extractor's own imperfections cancel and
the comparison carries the argument:

| flight | overlap at (0,0) | peak | headroom | peak/median | occupancy result |
|---|---|---|---|---|---|
| **0035** (control) | 1204 | 1337 @ 4.03 m | **9.9%** | 1.55x — flat | 11 bays occupied, 11/13 views |
| **0075** | 19 | 133 @ 4.51 m | **85.7%** | 3.98x — sharp | 0 bays occupied |

**The georeferencing method is sound.** On 0035 the bays already sit on the paint: the best
available shift buys 9.9% over doing nothing, on a peak barely above the search median. That is what
"no offset" looks like, and it is the positive control the earlier argument lacked — a systematic
error in the pose chain would have shown here too.
**On 0075 there IS a real disagreement**, ~4.5 m, and the bays plainly do not sit on that street's
paint. But 4.5 m does **not** explain the occupancy result: the detections' median distance to the
nearest bay there is 8.95 m, so correcting it would still leave most cars unassigned. So the honest
reading of that street is BOTH — a few metres of geometry disagreement, and cars genuinely parked
away from the mapped bays.

**What the paint extractor cost, and its limit.** A brightness-and-saturation threshold is useless
here: it marked 2.9% of the frame, because a white car roof is bright and grey exactly like paint,
and produced a map that was a solid blob (peak 1.43x = meaningless). Removing blobs by erosion, then
masking the detector's own car boxes (a car OUTLINE survives erosion and is a bay-sized rectangle
lying exactly where the question is — left in, the method would have been assuming its answer), then
keeping only elongated components, got the peak to 3.98x. The adversary specific to this footage is
**midday sun through summer canopy**, which scatters dappled highlights that are bright, grey and
thin — paint by every test except length. The mask now finds the strong markings (it picks the zebra
crossing cleanly) but **misses worn bay lines**, so the absolute offsets above are indicative; the
0035-vs-0075 CONTRAST is the load-bearing result, not the metre values.
