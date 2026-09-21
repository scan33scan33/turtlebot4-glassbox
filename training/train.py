#!/usr/bin/env python3
"""
train.py — train a YOLOv9-tiny (nano) detector on your few-class dataset.

    python train.py --data data.yaml --epochs 120 --imgsz 416

Transfer-learns from COCO-pretrained yolov9t (~2M params, the smallest YOLOv9).
Output: runs/detect/<name>/weights/best.pt  -> feed to export_for_oak.py

Train on a GPU (Colab/Kaggle) — nano on CPU is slow. `imgsz` matches your OAK
preview (416) so training and deployment resolutions agree.
"""
import argparse
from ultralytics import YOLO


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default="data.yaml")
    ap.add_argument("--model", default="yolov9t.pt", help="yolov9t (nano) | yolov9s | ...")
    ap.add_argument("--epochs", type=int, default=120)
    ap.add_argument("--imgsz", type=int, default=416, help="match the OAK preview (416); or 640")
    ap.add_argument("--batch", type=int, default=16, help="-1 = auto-fit to GPU memory")
    ap.add_argument("--device", default=None, help="0 / 0,1 / cpu / mps (default: auto)")
    ap.add_argument("--name", default="oak_nano")
    args = ap.parse_args()

    model = YOLO(args.model)                       # COCO-pretrained -> transfer learn
    model.train(
        data=args.data, epochs=args.epochs, imgsz=args.imgsz, batch=args.batch,
        device=args.device, name=args.name, patience=30,
        # modest augmentation — good for a small, domain-specific dataset:
        hsv_h=0.015, hsv_s=0.5, hsv_v=0.4, degrees=0.0, translate=0.1, scale=0.4,
        fliplr=0.5, mosaic=1.0, close_mosaic=15,
    )
    model.val()
    print(f"\nbest weights: runs/detect/{args.name}/weights/best.pt")
    print("Next: python export_for_oak.py --weights runs/detect/%s/weights/best.pt" % args.name)


if __name__ == "__main__":
    main()
