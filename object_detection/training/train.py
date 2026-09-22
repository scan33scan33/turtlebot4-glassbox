#!/usr/bin/env python3
"""
train.py — train a YOLO detector for the OAK-D camera.

Usage:
    # Few-class fine-tuning (e.g. ball + person):
    python train.py --data datasets/few_class.yaml --epochs 120 --imgsz 416

    # 18-class indoor robot detector (COCO + Objects365 subset):
    python train.py --data datasets/coco_o365_18.yaml --model yolov9t.pt --imgsz 416 --epochs 100 --batch 32

Defaults to yolov9t (nano, RVC2/gen2 deployable). For OAK-D deployment, train (or
later export) at --imgsz 416 to match the camera preview resolution.
Output: runs/detect/<name>/weights/best.pt -> feed to export_for_oak.py
"""
import argparse
from ultralytics import YOLO


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default="datasets/few_class.yaml", help="dataset YAML path")
    ap.add_argument("--model", default="yolov9t.pt", help="yolov9t (nano) | yolov9s | yolov8s | ...")
    ap.add_argument("--epochs", type=int, default=100)
    ap.add_argument("--imgsz", type=int, default=416, help="416 to match OAK preview; or 640")
    ap.add_argument("--batch", type=int, default=32, help="-1 = auto-fit GPU memory")
    ap.add_argument("--device", default=None, help="0 / 0,1 / cpu / mps (default: auto)")
    ap.add_argument("--name", default="detector")
    args = ap.parse_args()

    model = YOLO(args.model)
    model.train(
        data=args.data, epochs=args.epochs, imgsz=args.imgsz, batch=args.batch,
        device=args.device, name=args.name, patience=25, cache=False,
        hsv_h=0.015, hsv_s=0.5, hsv_v=0.4, degrees=0.0, translate=0.1, scale=0.4,
        fliplr=0.5, mosaic=1.0, close_mosaic=10,
    )
    print(f"\nbest weights: runs/detect/{args.name}/weights/best.pt")
    print(f"OAK deploy: python export_for_oak.py --weights runs/detect/{args.name}/weights/best.pt --imgsz {args.imgsz}")


if __name__ == "__main__":
    main()
