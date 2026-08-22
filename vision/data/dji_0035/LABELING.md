# Labelling spec — dji_0035 (real DJI nadir footage, FMI block)

35 stills to label by hand. This is the first REAL labelled data in the project;
everything before it came free from the simulator, where the world generator
already knew where every car was.

## Format — YOLO .txt, one file per image, same as the sim

One `.txt` per image, same stem, in `labels/<split>/`. One line per car:

    0 <x_center> <y_center> <width> <height>

all four numbers **normalised 0–1** by image size (3840 x 2160). Class is always
`0` (= `car`). An image with no cars gets an **empty .txt file**, not a missing
one — a missing file reads as "unlabelled", an empty one as "verified empty",
and the difference matters when the model is scored.

This is byte-identical in shape to what `vision/dataset.py --yolo` emits for the
sim worlds, deliberately: it means sim frames and real frames can be poured into
one training run without a conversion step.

## One class, axis-aligned boxes

**Only `car`.** Do not split into car/van/truck/SUV — occupancy asks "is this bay
taken", and a van takes a bay exactly as a hatchback does. Extra classes would
split the training signal for no gain.

**Axis-aligned boxes** (the normal kind that snap to the image edges), not
rotated ones. Cars here sit at whatever angle the street runs, so a diagonal car
gets a box with a fair amount of road in the corners. That is accepted: it
matches the sim labels, and it is what stock YOLO consumes. (Rotated boxes are a
different model head — `yolov8-obb` — and a decision worth revisiting later, not
now.)

## What counts as a car

Label it if it is a **four-wheeled road vehicle**: cars, vans, SUVs, small
trucks, pickups. All of them class `0`.

Do **not** label: motorcycles, scooters, bicycles, wheelie bins, market stalls,
skips, or the little shack roofs that YOLO keeps calling trucks.

## Where to put the box

**Around the whole car as you judge it to be**, not just the bright roof panel.

- **Off-centre cars show their sides.** The camera has a 73.7 deg horizontal
  field of view, so a car near the frame edge is seen at an angle and you can see
  down its flank. Include the visible side in the box — the car's real outline in
  this picture, not the roof rectangle.
- **Leaves in front of a car do not shrink the car.** Tree canopy is the single
  biggest occluder in this footage. If you can tell a car is there, box its full
  outline as you estimate it, including the parts with leaves crossing over.
  The leaves are in front of it, not instead of it.
- **Cut off by the image edge**: box the part that is in frame. If **less than
  about half** the car is in frame, skip it. (The sim drops boxes below 55%
  visible; same spirit.)
- **If you cannot tell whether it is a car, skip it.** Fewer clean labels beat
  more guessed ones. This is ground truth — its only value is being right.

## Cases you will actually hit, decided in advance

| you see | do |
|---|---|
| car under a fitted cover / tarp | **label it** — it is a vehicle occupying the space |
| car in a private driveway or courtyard, not on the street | **label it** — the detector's job is finding cars; whether a car sits in a *mapped bay* is decided later, by geometry, not by you |
| car moving down the road | **label it** — a car is a car; motion is filtered downstream |
| car half under a dense tree, clearly a car | **label it**, full estimated outline |
| dark shape under a tree, might be a car, might be shadow | **skip** |
| the same car in the next frame | **label it again** — every frame is labelled independently |

## Which frames, and why these ones

`tools/dji_stills.py` cut 137 stills at 1 Hz, ~4.2 m apart, against a 25 m
along-track footprint — about **6 looks at the same ground**. Labelling all 137
would be mostly re-labelling the same cars.

These 35 are **every 4th frame** (~16 m apart, ~1.5x overlap): enough variety,
little duplication.

The train/val split is **spatial, not random, and that is load-bearing.** The
flight goes out east, **doubles back over the same street**, then heads
northwest. Frames 55–85 sit within 3–6 m of frames 5–30 — the *same parked
cars*. A random split would put one photo of a car in train and another photo of
the same car in val, and the model would score brilliantly by memorising it.

- `train/` = frames 0–92 (the eastward leg and its return over the same ground)
- `val/`   = frames 96–136 (the northwest leg, ground the drone never covered before)

## Suggested tool

`labelImg` writes this format natively — set **Save format: YOLO** and point the
save directory at `labels/train` (then `labels/val`). Note it needs PyQt5 and may
not install cleanly on Python 3.13; `Label Studio`, `CVAT` or `X-AnyLabeling` are
fine alternatives as long as the export is YOLO txt with a single class.

Whatever you use, point `classes.txt` / the tool's class list at exactly one
class named `car`, so it writes `0` and not some other index.

## When you are done

`python vision/check_labels.py vision/data/dji_0035` validates the files and
draws the boxes back onto the frames, so the labels get checked the same way the
sim's do — by looking at them.
