#!/usr/bin/env python3
"""
Load / inspect / render the per-drive planner datasets written by
tb4_claude_nav.py's DriveRecorder (datasets/drive_*/step_*.npz).

Each step npz holds: scan_ranges, cost_grid (HxW, inf=blocked), raw_path (Nx2
cells, pre-smoothing), smoothed_path (Mx2 cells), and a json `meta` string
(pose, goal, lookahead, command, detections, scan params, run_program, current_cmd).

Usage:
    python3 dataset_tools.py                 # list drives + step counts
    python3 dataset_tools.py <drive_dir>     # print summary of a drive
    python3 dataset_tools.py <drive_dir> --render   # render every step to PNG
    python3 dataset_tools.py <drive_dir> --video    # stitch a camera|BEV MP4
"""
import os, sys, json, glob
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
DATASET_DIR = os.path.join(HERE, "datasets")


def list_drives(root=DATASET_DIR):
    drives = sorted(glob.glob(os.path.join(root, "drive_*")))
    for d in drives:
        steps = len(glob.glob(os.path.join(d, "step_*.npz")))
        end = os.path.join(d, "END.json")
        status = json.load(open(end)).get("status", "?") if os.path.exists(end) else "in-progress"
        print(f"{os.path.basename(d):28s} {steps:4d} steps  [{status}]")
    return drives


def load_step(npz_path):
    z = np.load(npz_path, allow_pickle=False)
    return dict(scan_ranges=z["scan_ranges"], cost_grid=z["cost_grid"],
                raw_path=z["raw_path"], smoothed_path=z["smoothed_path"],
                meta=json.loads(str(z["meta"])))


def load_drive(drive_dir):
    meta = json.load(open(os.path.join(drive_dir, "meta.json")))
    steps = sorted(glob.glob(os.path.join(drive_dir, "step_*.npz")))
    return meta, steps


def summarize(drive_dir):
    meta, steps = load_drive(drive_dir)
    print(f"drive: {drive_dir}")
    print(f"  start goal: {meta.get('goal_odom')}  start pose: {meta.get('start_pose')}")
    print(f"  grid {meta.get('grid_cells')}@{meta.get('grid_res')}m  inflate {meta.get('inflate_r')}  steps: {len(steps)}")
    for s in steps:
        st = load_step(s); m = st["meta"]
        nd = len(m.get("detections", []))
        print(f"  step {m['step']:4d}  raw={m['raw_path_len']:3d} smooth={m['smoothed_path_len']:2d} "
              f"cmd v={m['cmd']['v']:.2f} w={m['cmd']['w']:+.2f}  dets={nd}  {m.get('current_cmd') or ''}")


def _bev_ui(step):
    """BEV matching the live website (reuses tb4_claude_nav.render_lidar_png).
    Returns a BGR image, or None if tb4_claude_nav can't be imported (off-robot)."""
    try:
        import cv2
        from types import SimpleNamespace
        sys.path.insert(0, HERE)
        from tb4_claude_nav import render_lidar_png
        m = step["meta"]; sp = m["scan_params"]
        scan = SimpleNamespace(ranges=step["scan_ranges"], angle_min=sp["angle_min"],
                               angle_increment=sp["angle_increment"],
                               range_min=sp["range_min"], range_max=sp["range_max"])
        png = render_lidar_png(scan, step["smoothed_path"].tolist(),
                               tuple(m["goal_local"]) if m.get("goal_local") else None,
                               m.get("detections", []),
                               tuple(m["look_local"]) if m.get("look_local") else None,
                               m.get("goal_mode", "move"))
        return cv2.imdecode(np.frombuffer(png, np.uint8), cv2.IMREAD_COLOR)
    except Exception:
        return None


def _bev_costgrid(step, grid_cells, up=4):
    """Fallback BEV (no tb4_claude_nav needed): cost grid + raw/smoothed paths +
    goal + robot. Forward(+x) is UP."""
    import cv2
    g = step["cost_grid"]; H, W = g.shape; half = grid_cells // 2
    img = np.full((H, W, 3), 18, np.uint8)                 # free = near-black
    soft = (~np.isinf(g)) & (g > 0)
    inten = np.clip(g / max(g[soft].max(), 1e-6), 0, 1) if soft.any() else g
    img[soft] = np.stack([np.zeros_like(inten), 80*inten, 150*inten], -1)[soft].astype(np.uint8)
    img[np.isinf(g)] = (40, 40, 130)                       # blocked = dark red (BGR)
    img = cv2.resize(img, (W*up, H*up), interpolation=cv2.INTER_NEAREST)
    def p(rc):
        r, c = int(rc[0]), int(rc[1]); return (c*up, (H-1-r)*up)
    for path, color in [(step["raw_path"], (0, 255, 255)), (step["smoothed_path"], (0, 230, 0))]:
        pts = [p(rc) for rc in path.tolist()]
        for i in range(len(pts)-1):
            cv2.line(img, pts[i], pts[i+1], color, 2, cv2.LINE_AA)
        for q in pts:
            cv2.circle(img, q, 2, color, -1)
    gc = step["meta"].get("goal_cell")
    if gc: cv2.circle(img, p(gc), 6, (0, 0, 255), -1)      # goal red
    cv2.circle(img, p((half, half)), 6, (255, 160, 30), -1)  # robot
    return img


def render_step(step, out_png, grid_res, grid_cells, up=4, image_path=None):
    """Camera | BEV frame for a recorded step, with a bottom strip showing the
    running ToyScript skill (current_cmd). BEV matches the live UI when
    render_lidar_png is importable, else a cost-grid fallback."""
    import cv2
    m = step["meta"]
    bev = _bev_ui(step)
    if bev is None:
        bev = _bev_costgrid(step, grid_cells, up)
    cv2.putText(bev, f"step {m['step']}  v={m['cmd']['v']:.2f} w={m['cmd']['w']:+.2f}",
                (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
    if image_path and os.path.exists(image_path):
        cam = cv2.imread(image_path)
        if cam is not None:
            cam = cv2.resize(cam, (bev.shape[0], bev.shape[0]))
            bev = np.hstack([cam, bev])
    # bottom strip: the TASK (program) being run + the current action (skill)
    prog = m.get("run_program") or ""
    cmd  = m.get("current_cmd") or "idle"
    strip = np.zeros((46, bev.shape[1], 3), np.uint8)
    if prog:
        cv2.putText(strip, "TASK: " + str(prog)[:52], (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.48,
                    (140, 235, 140), 1, cv2.LINE_AA)
    cv2.putText(strip, "> " + str(cmd)[:66], (8, 39), cv2.FONT_HERSHEY_SIMPLEX, 0.52,
                (90, 220, 220), 1, cv2.LINE_AA)
    cv2.imwrite(out_png, np.vstack([bev, strip]))


def render_drive(drive_dir):
    meta, steps = load_drive(drive_dir)
    outdir = os.path.join(drive_dir, "render"); os.makedirs(outdir, exist_ok=True)
    for s in steps:
        st = load_step(s)
        out = os.path.join(outdir, os.path.basename(s).replace(".npz", ".png"))
        img = st["meta"].get("image")
        img_path = os.path.join(drive_dir, img) if img else None
        render_step(st, out, meta["grid_res"], meta["grid_cells"], image_path=img_path)
    print(f"rendered {len(steps)} steps -> {outdir}")


def render_video(drive_dir, fps=8):
    """Stitch the per-step (camera | BEV + skill) frames into a single MP4.
    Uses ffmpeg/H.264 (yuv420p) for broad player compatibility — OpenCV's mp4v
    writer produces files that render as a green screen in QuickTime."""
    import subprocess, shutil
    meta, steps = load_drive(drive_dir)
    fdir = os.path.join(drive_dir, "render"); os.makedirs(fdir, exist_ok=True)
    for i, s in enumerate(steps):
        st = load_step(s)
        img = st["meta"].get("image")
        ip = os.path.join(drive_dir, img) if img else None
        render_step(st, os.path.join(fdir, f"f_{i:05d}.png"),
                    meta["grid_res"], meta["grid_cells"], image_path=ip)
    out = os.path.join(drive_dir, "drive.mp4")
    if shutil.which("ffmpeg"):
        subprocess.run(["ffmpeg", "-y", "-framerate", str(fps),
                        "-i", os.path.join(fdir, "f_%05d.png"),
                        "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "23", out],
                       check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        print(f"wrote {out} (H.264, {len(steps)} frames @ {fps}fps)")
    else:
        import cv2
        fr0 = cv2.imread(os.path.join(fdir, "f_00000.png")); h, w = fr0.shape[:2]
        wr = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h))
        for i in range(len(steps)):
            wr.write(cv2.imread(os.path.join(fdir, f"f_{i:05d}.png")))
        wr.release()
        print(f"wrote {out} (mp4v fallback, {len(steps)} frames)")


if __name__ == "__main__":
    args = sys.argv[1:]
    if not args:
        list_drives()
    elif "--video" in args:
        render_video(args[0])
    elif "--render" in args:
        render_drive(args[0])
    else:
        summarize(args[0])
