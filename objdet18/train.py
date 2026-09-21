#!/usr/bin/env python3
"""
train.py — train on the merged COCO+Objects365 18-class dataset.

    python train.py --model yolov9t.pt --imgsz 640 --epochs 100 --batch 64

Defaults to yolov9t (nano, gen2/RVC2-deployable). For a stronger model on this
much data, try yolov9s / yolov8s. For OAK-D deployment, train (or later export)
at --imgsz 416 to match the camera preview.
"""
import argparse
from ultralytics import YOLO


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default="data.yaml")
    ap.add_argument("--model", default="yolov9t.pt")
    ap.add_argument("--imgsz", type=int, default=640, help="640 general; 416 to match the OAK")
    ap.add_argument("--epochs", type=int, default=100)
    ap.add_argument("--batch", type=int, default=64, help="-1 = auto-fit GPU memory")
    ap.add_argument("--device", default=None, help="0 / 0,1,2,3 / cpu")
    ap.add_argument("--name", default="coco_o365_18")
    args = ap.parse_args()

    YOLO(args.model).train(
        data=args.data, epochs=args.epochs, imgsz=args.imgsz, batch=args.batch,
        device=args.device, name=args.name, patience=25, cache=False,
        hsv_h=0.015, hsv_s=0.7, hsv_v=0.4, fliplr=0.5, mosaic=1.0, close_mosaic=10,
    )
    print(f"\nbest: runs/detect/{args.name}/weights/best.pt")
    print("OAK deploy: python ../training/export_for_oak.py --weights <best.pt> --imgsz 416")


if __name__ == "__main__":
    main()
