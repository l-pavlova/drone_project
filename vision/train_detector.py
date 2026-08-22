"""Fine-tune YOLO on the hand-labelled real footage.

    python vision/train_detector.py [--data vision/data/dji_0035/data.yaml]
                                    [--model yolov8s.pt] [--epochs 100]
                                    [--imgsz 1280] [--batch 2] [--name run1]

This is a PROBE, not a product.  The dataset is 82 training boxes over 22
frames from a single flight, and the val set is 26 boxes -- one car either way
moves recall by 3.8%, so anything under ~10% of difference is noise.  What it
can answer is whether fine-tuning moves the needle at all, which is what decides
whether labelling several hundred more boxes is worth the hours.

Augmentation is set for NADIR imagery specifically, and the settings are the
interesting part:

  * `degrees=180` -- ultralytics defaults to 0.  Straight down, there is no
    canonical "up": the drone's heading is arbitrary and a car may point any
    way in the frame.  Full rotation is therefore a FREE and truthful expansion
    of a tiny dataset, not a distortion of it.  This is the single biggest lever
    available here.
  * `flipud=0.5` -- also 0 by default, for the same reason: a vertical flip of a
    ground-facing photo is another legitimate view.  (In side-on COCO imagery it
    would mean an upside-down car, which is why the default is off.)
  * `scale`/`mosaic` stay near default -- altitude is fixed at 30.1 m, so scale
    variation is mild in reality, but a little helps the model tolerate the
    altitude drift a real patrol will have.

`degrees` and `flipud` are the reason this is worth trying at all on 22 images.

**The learning rate must be set EXPLICITLY, and the default is wrong here
(measured, run `probe1`, 2026-08-22).** With `optimizer="auto"` ultralytics
ignores `lr0` entirely and chose `AdamW(lr=0.002)`. On this dataset an epoch is
22 images at batch 2 = **11 optimizer steps**, and the run peaked at **epoch 4 --
the last epoch of the 3-epoch warmup -- then collapsed from mAP50 0.714 to 0.005
on the first epoch at full LR**, and spent 30 epochs climbing back to exactly
where it had been before early stopping fired. So `optimizer` and `lr0` are
passed explicitly here, at a quarter of what `auto` picked, with a longer warmup.

A second symptom of the same scale problem: validation metrics repeat in blocks
of ~3 epochs, because val runs on the EMA weights and 11 updates per epoch barely
move the EMA. The val curve is both lagged and quantised, so read it in trend,
not per epoch.
"""
import argparse
import os
import sys
import time

sys.stdout.reconfigure(encoding="utf-8")
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default=os.path.join(ROOT, "vision", "data",
                                                   "dji_0035", "data.yaml"))
    ap.add_argument("--model", default="yolov8s.pt")
    ap.add_argument("--epochs", type=int, default=100)
    ap.add_argument("--imgsz", type=int, default=1280)
    ap.add_argument("--batch", type=int, default=2)
    ap.add_argument("--name", default="dji0035")
    ap.add_argument("--workers", type=int, default=0,
                    help="0 avoids the Windows dataloader-spawn overhead, which "
                         "dominates when an epoch is only 11 iterations")
    ap.add_argument("--device", default="cpu")
    ap.add_argument("--lr0", type=float, default=0.0005,
                    help="initial LR. NOT the ultralytics default -- see below")
    ap.add_argument("--optimizer", default="AdamW",
                    help="explicit, so 'auto' cannot silently pick the LR")
    ap.add_argument("--warmup", type=float, default=5.0)
    ap.add_argument("--patience", type=int, default=40)
    a = ap.parse_args()

    from ultralytics import YOLO

    print(f"fine-tuning {a.model} on {a.data}")
    print(f"  imgsz {a.imgsz}, batch {a.batch}, {a.epochs} epochs, device {a.device}")

    t0 = time.time()
    model = YOLO(a.model)
    model.train(
        data=a.data,
        epochs=a.epochs,
        imgsz=a.imgsz,
        batch=a.batch,
        workers=a.workers,
        device=a.device,
        project=os.path.join(ROOT, "vision", "runs"),
        name=a.name,
        exist_ok=True,
        # --- nadir-specific augmentation; see the module docstring ---
        degrees=180.0,      # heading is arbitrary looking straight down
        flipud=0.5,         # a vertical flip is another legitimate nadir view
        fliplr=0.5,
        # --- small-dataset hygiene ---
        optimizer=a.optimizer,
        lr0=a.lr0,
        warmup_epochs=a.warmup,
        patience=a.patience,  # stop if val stops improving
        val=True,
        plots=True,
        seed=0,             # a probe that cannot be reproduced proves nothing
    )
    print(f"\ntrained in {(time.time() - t0) / 60:.1f} min")
    print(f"weights: vision/runs/{a.name}/weights/best.pt")


if __name__ == "__main__":
    main()
