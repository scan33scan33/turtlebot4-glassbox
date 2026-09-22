#!/usr/bin/env python3
"""
tb4_claude_nav.py — YOLO + Lidar A* navigator for TurtleBot4 with detection overlay

Vision pipeline:
  1. YOLOv8n detects objects with bounding boxes (80 COCO classes)
  2. Text description → keyword match against YOLO detections
     → multiple matches: pick highest confidence
  3. Matched object pixel x → bearing; lidar range at bearing → metric distance
  4. Goal tracked in odom frame → A* on lidar grid → cmd_vel path following
  5. On arrival → loop back to step 1

Dependencies:
    pip install ultralytics flask opencv-python numpy

Usage:
    source /opt/ros/jazzy/setup.bash
    python3 tb4_claude_nav.py
    # Open http://localhost:5000
"""
import os, json, math, time, threading, heapq, re, glob, subprocess
import numpy as np
import toyscript
import cv2
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan, Image, BatteryState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from vision_msgs.msg import Detection3DArray
from irobot_create_msgs.msg import DockStatus, HazardDetectionVector
try:                                        # Create 3 speaker (startup chime)
    from irobot_create_msgs.msg import AudioNoteVector, AudioNote
    from builtin_interfaces.msg import Duration as _AudioDur
    _HAS_AUDIO = True
except Exception:
    _HAS_AUDIO = False
from irobot_create_msgs.action import Dock, Undock
from std_srvs.srv import Empty
from flask import Flask, Response, jsonify, request, render_template

# ── Config ────────────────────────────────────────────────────────────────────
FLASK_PORT    = 5000
GRID_RES      = 0.05      # m/cell
GRID_CELLS    = 120       # 120×120 = 6 m × 6 m (robot at centre)
ROBOT_R       = 0.18      # hard collision radius (m): cells this close to an obstacle are impassable
INFLATE_R     = 0.30      # soft-cost falloff distance (m): A* is penalised for hugging walls
COST_SCALE    = 8.0       # soft-cost steepness per metre (≈ Nav2 cost_scaling_factor)
COST_WEIGHT   = 1.5       # max soft penalty added to move cost at the hard edge
DRIVE_SPEED   = 0.22      # m/s forward (cruise; scaled by cos(heading error))
TURN_SPEED    = 0.6       # rad/s
SCAN_TURN_SPEED = 0.30    # rad/s — slow sweep while scanning so the 6Hz detector keeps up
GOAL_TOL      = 0.25      # m
PUSH_GOAL_TOL = 0.10      # m — push drives all the way onto the target (vs stopping short)
PUSH_THROUGH_SPEED = 0.15 # m/s straight-push to finish a close ball at the wall
PUSH_WALL_V        = 0.16 # m/s forward while pushing the ball to the wall
PUSH_WALL_STEER    = 1.4  # rad/s of yaw per rad of ball bearing (keep ball centred)
PUSH_WALL_ESCAPE   = 0.60 # rad (~34deg) — ball bearing past this = escaped sideways
PUSH_WALL_STANDOFF = 0.55 # m behind the ball for the forward-loop reposition
# Push-the-ball-to-a-HUMAN-SET-goal (PUSH_TO_GOAL). The robot lines up on the
# goal↔ball line BEHIND the ball, then shoves it along that line, re-lining-up
# after every nudge — so it converges on the goal instead of drifting off it.
PUSH_STANDOFF   = 0.55    # m — how far behind the ball the robot lines up. Must
                          #     exceed GOAL_TOL + PUSH_ALIGN_BACK or a go_to that
                          #     "arrives" still fails the line-up check and loops.
PUSH_APPROACH   = 1.10    # m — go_to aims this far behind the ball, so the final
                          #     dock has room to servo the lateral error out
PUSH_GOAL_DONE  = 0.30    # m — ball this close to the goal counts as delivered
PUSH_LINE_TOL   = 0.25    # m — ball this far off the push line = stop & re-line-up
PUSH_ALIGN_LAT  = 0.28    # m — lateral slack for "already behind the ball" (must
                          #     exceed GOAL_TOL or go_to can never satisfy it)
PUSH_ALIGN_BACK = 0.26    # m — the robot must be at least this far behind the ball
PUSH_GOAL_V     = 0.14    # m/s forward while pushing to the goal
PUSH_GOAL_STEER = 1.4     # rad/s of yaw per rad of ball bearing (keep it centred)
PUSH_GOAL_LINE_K = 0.6    # extra yaw gain on the heading error to the goal, so the
                          #     shove stays on the line despite a sloppy line-up
PUSH_GOAL_LAT_K  = 0.5    # aim this far back toward the line per metre the ball is
                          #     off it, so a shove curves the ball back onto the line
PUSH_DOCK_STOP   = 0.36   # m — the dock stops this far short of the ball (contact is
                          #     ~ROBOT_R + a ball radius ≈ 0.29 m, so this is the
                          #     closest it dares creep before shoving)
PUSH_DOCK_CARROT = 0.45   # m — carrot ahead on the line the dock steers toward
PUSH_DOCK_ROOM   = PUSH_APPROACH - GOAL_TOL   # m — least distance behind the ball the
                          #     dock needs to servo a full GOAL_TOL of error out
PUSH_DOCK_V      = 0.25   # m/s — fastest the dock creeps (scaled down near the ball)
PROG_M          = 0.010   # m of odom travel that counts as progress (see _push_segment)
PUSH_GOAL_ALIGN = 0.12    # rad — heading error tolerated before starting a push
PUSH_SEG_TIME   = 25.0    # s — one push segment, then re-find & re-line-up
FOLLOW_STANDOFF = 1.0     # m — never approach a person closer than this
FOLLOW_STANDOFF_CLOSE = 0.7  # m — for the small/low targets in _FOLLOW_CLOSE: at 1 m
                             #     the 416-px preview loses them (see _follow_params)
PUSH_CLEAR_R  = 0.35      # m — radius around a push target cleared of obstacle cost so
                          #     A* can path INTO the object instead of routing around it
HEADING_TOL   = 0.20      # rad
LOOKAHEAD_M   = 0.50      # m — A* look-ahead
REPLAN_HZ     = 3.0       # replan often so the lookahead advances smoothly
PLAN_TIMEOUT  = 4.0       # give up (status 'blocked: no path') after this long with no A* path
STALL_TIMEOUT = 2.5       # stop if commanded to move but no odom progress this long (pinned/stuck)
VIZ_SIZE      = 400       # px — lidar canvas
# COCO 80-class order (matches the depthai YOLO config mappings). The driver
# publishes class_id as the numeric index into this list.
COCO_LABELS = ["person","bicycle","car","motorbike","aeroplane","bus","train",
    "truck","boat","traffic light","fire hydrant","stop sign","parking meter",
    "bench","bird","cat","dog","horse","sheep","cow","elephant","bear","zebra",
    "giraffe","backpack","umbrella","handbag","tie","suitcase","frisbee","skis",
    "snowboard","sports ball","kite","baseball bat","baseball glove","skateboard",
    "surfboard","tennis racket","bottle","wine glass","cup","fork","knife","spoon",
    "bowl","banana","apple","sandwich","orange","broccoli","carrot","hot dog",
    "pizza","donut","cake","chair","sofa","pottedplant","bed","diningtable","toilet",
    "tvmonitor","laptop","mouse","remote","keyboard","cell phone","microwave","oven",
    "toaster","sink","refrigerator","book","clock","vase","scissors","teddy bear",
    "hair drier","toothbrush"]
# Create 3 hazard types (irobot_create_msgs/HazardDetection). BUMP/WHEEL_DROP
# trigger an emergency stop; BACKUP_LIMIT fires constantly near the limit so it's ignored.
HAZARD_NAMES  = {0: "backup_limit", 1: "bump", 2: "cliff", 3: "stall",
                 4: "wheel_drop", 5: "obstacle"}
HAZARD_STOP   = {1, 2, 4}        # bump, cliff, wheel_drop → halt navigation
NN_TOPIC      = "/oakd/nn/spatial_detections"     # on-camera YOLO spatial (OAK-D VPU)
DET_CONF_MIN  = 0.3       # drop detections below this confidence (cut false positives)
CAM_TOPIC     = "/oakd/rgb/preview/image_raw"      # OAK-D RGB preview
CAM_FX        = 325.95    # OAK-D 416 preview intrinsics (from camera_info)
CAM_CX        = 212.43
DEPTH_TOPIC   = "/oakd/stereo/image_raw"   # raw stereo depth (16UC1 mm) for p25 distance
IMG_REFRESH_HZ = 2.0       # 2 fps
# ─────────────────────────────────────────────────────────────────────────────

# `template_folder` is relative to this module's directory (Flask's root_path),
# NOT to the process cwd — the UI resolves from wherever the navigator is
# started, including the systemd units and `ros2 launch`. templates/index.html
# is committed, so a missing file means a broken checkout: render_template then
# raises TemplateNotFound naming the path it searched, which is far more useful
# than serving a placeholder page that hides the problem.
app = Flask(__name__, template_folder="templates")

_lock  = threading.Lock()
_state = {
    # ROS data
    "scan":               None,
    "scan_t":             0.0,    # wall time of last /scan (lidar self-heal)
    "img_t":              0.0,    # wall time of last camera frame (OAK self-heal)
    "odom_t":             0.0,    # wall time of last /odom (base-link health)
    "frame_rgb":          None,   # latest raw RGB (for overlay)
    "frame_jpg":          None,   # latest JPEG with detection overlay
    "odom_x":             0.0,
    "odom_y":             0.0,
    "odom_yaw":           0.0,
    # Battery
    "battery_voltage":    0.0,
    "battery_percentage": 0.0,
    # Docking
    "docked":             None,   # None = unknown, True/False from /dock_status
    "dock_busy":          False,  # a Dock/Undock action is in flight
    "hazards":            [],     # active hazards [{type, where}] from /hazard_detection
    "last_hazard_t":      0.0,    # wall time of last hazard msg (for auto-clear)
    # ToyScript program runner
    "run_active":         False,
    "run_program":        "",
    "run_log":            [],
    "cmd_history":        [],   # rolling last-5 [{cmd,result,running}] for the UI
    "run_error":          None,
    "last_command":       "",   # the plain-language command the user gave
    "match_score":        0.0,  # matcher's token-overlap strength for that command
    "plan":               [],   # [{phrase,verb}] the matched program will carry out
    # On-camera YOLO (OAK-D VPU) — spatial detections in robot-local frame
    "detections":         [],     # [{label, conf, x_loc, y_loc, dist, x_px, y_px, w_px, h_px}]
    "cmd_sent":           None,   # last twist actually published {v, w, t} (recorder ground truth)
    "use_lidar_dist":     True,   # True: dist=min(OAK stereo, lidar@bearing); False: raw OAK
    "target_color":       None,   # colour-targeted command: only match balls of this colour
    # Follow-command parameters, all resolved from the spoken phrase in ONE place
    # (_follow_params) and consumed by NavRobot.follow — so programs/follow.toy
    # and the skill cannot disagree about what was asked for.
    "follow_target":      None,   # 'follow the X' -> COCO object to follow (None = person)
    "follow_standoff":    None,   # m to stop short of it (1.0 big targets, 0.7 small)
    "follow_scan_first":  False,  # step-and-stare sweep before following (small targets)
    "follow_fast":        False,  # lidar + velocity predictor (small targets only)
    # Navigation
    "destination":        "",
    "status":             "idle",
    "nav_active":         False,
    "goal_mode":          "move",  # "move" = stop short of obstacles; "push" = drive into the target
    "goal_odom":          None,
    "goal_local":         None,
    "look_local":         None,   # current lookahead target in robot frame (for BEV)
    "path":               [],
    "match_info":         "",     # how target was found
    "direct_goal":        False,  # True = goal_odom was set directly (skip vision)
    # Ball-push task (PUSH_TO_GOAL) — the goal is for the BALL, not the robot
    "ball_goal":          None,   # human-set odom (x, y) the ball should be pushed to
    "ball_mark":          None,   # last known ball position (odom x, y)
    "push_from":          None,   # stance point on the goal↔ball line (odom x, y)
    # Dataset recording (UI-controlled)
    "recording":          False,
    "record_drive":       "",     # current drive dir name
    "record_steps":       0,
    # Viz
    "lidar_png":          None,
}

ros_node = None


def _get(k):
    with _lock: return _state[k]

def _set(**kw):
    with _lock: _state.update(kw)


# ── Math helpers ──────────────────────────────────────────────────────────────
def _quat_to_yaw(q):
    return math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))


def _wrap_angle(a):
    """Angle (rad) wrapped to (-pi, pi]."""
    return (a + math.pi) % (2 * math.pi) - math.pi


def _odom_to_local(pt, ox, oy, oyaw):
    """odom point -> robot/base_link frame (x=forward, y=left). The inverse of
    the local->odom rotation every skill uses: rotate the odom delta by -yaw."""
    dx, dy = pt[0] - ox, pt[1] - oy
    return (dx * math.cos(oyaw) + dy * math.sin(oyaw),
            -dx * math.sin(oyaw) + dy * math.cos(oyaw))


# ── Ball-push geometry ────────────────────────────────────────────────────────
# "Find a line between the goal and the robot with the ball in between, then
# push." These are the pure functions behind PUSH_TO_GOAL: they take odom (x, y)
# points and know nothing about ROS, so they are unit-testable on their own.

def push_line(ball, goal):
    """Unit vector the ball has to travel along (ball → goal) and that distance."""
    dx, dy = goal[0] - ball[0], goal[1] - ball[1]
    d = math.hypot(dx, dy)
    if d < 1e-6:
        return (1.0, 0.0), 0.0
    return (dx / d, dy / d), d


def push_stance(ball, goal, standoff=PUSH_STANDOFF):
    """Where the robot must stand to push `ball` to `goal`: `standoff` behind the
    ball, ON the goal↔ball line — robot / ball / goal collinear, ball in the
    middle. This is the point the robot drives to before every shove."""
    (ux, uy), _ = push_line(ball, goal)
    return (ball[0] - ux * standoff, ball[1] - uy * standoff)


def stance_error(robot, ball, goal):
    """Split the robot's offset from the ball into `along` (positive = in FRONT of
    the ball, i.e. on the goal side — the wrong side to push from) and `lateral`
    (off the push line). Returns (along, lateral, need_reposition)."""
    (ux, uy), _ = push_line(ball, goal)
    rx, ry = robot[0] - ball[0], robot[1] - ball[1]
    along = rx * ux + ry * uy
    lateral = abs(-rx * uy + ry * ux)
    return along, lateral, (along > -PUSH_ALIGN_BACK or lateral > PUSH_ALIGN_LAT)


def line_signed_offset(point, ball, goal):
    """Signed perpendicular distance of `point` from the push line through
    `ball`→`goal`: positive = LEFT of the direction of travel, negative = right.
    `ball` is the line's ANCHOR (where the ball was when the shove started), so a
    ball that veers sideways shows up here instead of dragging the line with it."""
    (ux, uy), _ = push_line(ball, goal)
    px, py = point[0] - ball[0], point[1] - ball[1]
    return -px * uy + py * ux


def line_offset(point, ball, goal):
    """Unsigned version of line_signed_offset — how far off the line `point` is."""
    return abs(line_signed_offset(point, ball, goal))


# ── Occupancy grid ────────────────────────────────────────────────────────────
def build_grid(scan: LaserScan) -> np.ndarray:
    """Return a float cost field (robot frame, robot at centre):
      ∞                      → impassable (within ROBOT_R of an obstacle)
      COST_WEIGHT·exp(-k·d)  → soft penalty that decays with clearance d, so A*
                               prefers the middle of open space over skimming
                               walls (∞..0 over ROBOT_R..INFLATE_R)
      0                      → fully clear
    """
    half = GRID_CELLS // 2
    occ  = np.zeros((GRID_CELLS, GRID_CELLS), np.uint8)
    ang  = np.arange(len(scan.ranges)) * scan.angle_increment + scan.angle_min
    rng  = np.array(scan.ranges, np.float32)
    ok   = np.isfinite(rng) & (rng >= scan.range_min) & (rng <= scan.range_max)
    # The RPLIDAR is mounted yaw +90° from base_link (TF base_link->rplidar_link
    # = +1.571 rad: base_x = -lidar_y, base_y = +lidar_x). Rotate the scan into
    # base_link so the planner's obstacle map matches the robot's actual frame
    # (and the BEV display). Without this the whole grid is 90° off reality.
    xs   = -rng[ok] * np.sin(ang[ok])   # forward (base_link +x)
    ys   =  rng[ok] * np.cos(ang[ok])   # left    (base_link +y)
    rs   = np.clip((half + xs / GRID_RES).astype(int), 0, GRID_CELLS - 1)
    cs   = np.clip((half + ys / GRID_RES).astype(int), 0, GRID_CELLS - 1)
    occ[rs, cs] = 1

    # Distance (m) from every cell to the nearest obstacle.
    free   = np.where(occ == 0, 255, 0).astype(np.uint8)
    dist_m = cv2.distanceTransform(free, cv2.DIST_L2, 3) * GRID_RES

    cost = np.zeros((GRID_CELLS, GRID_CELLS), np.float32)
    cost[dist_m <= ROBOT_R] = np.inf
    soft = (dist_m > ROBOT_R) & (dist_m < INFLATE_R)
    cost[soft] = COST_WEIGHT * np.exp(-COST_SCALE * (dist_m[soft] - ROBOT_R))
    cost[half-1:half+2, half-1:half+2] = 0.0   # robot cell always free
    return cost


# ── A* ────────────────────────────────────────────────────────────────────────
def astar(grid: np.ndarray, start: tuple, goal: tuple):
    R, C = grid.shape
    gr, gc = goal
    if not (0 <= gr < R and 0 <= gc < C) or math.isinf(grid[gr, gc]):
        return None
    sr, sc = start
    h    = lambda r, c: math.hypot(r-gr, c-gc)
    heap = [(h(sr,sc), 0.0, sr, sc)]
    best = {(sr,sc): 0.0}
    came = {}
    seen = set()
    MOVES = [(-1,0,1.),(1,0,1.),(0,-1,1.),(0,1,1.),
             (-1,-1,1.414),(-1,1,1.414),(1,-1,1.414),(1,1,1.414)]
    while heap:
        _, g, r, c = heapq.heappop(heap)
        if (r,c) in seen: continue
        seen.add((r,c))
        if (r,c) == (gr,gc):
            path, cur = [], (gr,gc)
            while cur in came: path.append(cur); cur = came[cur]
            path.append((sr,sc)); return path[::-1]
        for dr, dc, move in MOVES:
            nr, nc = r+dr, c+dc
            if 0<=nr<R and 0<=nc<C and (nr,nc) not in seen and not math.isinf(grid[nr,nc]):
                # move cost + the soft proximity penalty of the cell we enter →
                # equal-length routes are no longer tied; the one with more
                # clearance wins, so the planner stops skimming walls.
                ng = g + move + float(grid[nr,nc])
                if ng < best.get((nr,nc), 1e18):
                    best[(nr,nc)] = ng; came[(nr,nc)] = (r,c)
                    heapq.heappush(heap, (ng+h(nr,nc), ng, nr, nc))
    return None


def _line_of_sight(grid: np.ndarray, a: tuple, b: tuple, max_cost: float = math.inf) -> bool:
    """True if the straight segment a→b stays within cost ≤ max_cost (Bresenham).
    Hard obstacles (∞) always block; a finite max_cost also stops a shortcut from
    cutting through the high-penalty band near walls, preserving A*'s clearance."""
    (r0, c0), (r1, c1) = a, b
    dr, dc = abs(r1-r0), abs(c1-c0)
    sr = 1 if r0 < r1 else -1
    sc = 1 if c0 < c1 else -1
    err = dr - dc
    r, c = r0, c0
    while True:
        if math.isinf(grid[r, c]) or grid[r, c] > max_cost:
            return False
        if (r, c) == (r1, c1):
            return True
        e2 = 2*err
        if e2 > -dc: err -= dc; r += sr
        if e2 <  dr: err += dr; c += sc


def smooth_path(grid: np.ndarray, path: list) -> list:
    """String-pull: collapse the 8-connected A* path to the fewest straight
    segments by keeping only vertices that need a turn. Open space (clear line
    of sight start→goal) therefore collapses to a single straight line. Shortcuts
    may not cut through the high-penalty band near walls (max_cost), so the
    clearance the cost field bought isn't smoothed away."""
    if not path or len(path) < 3:
        return path
    keep = COST_WEIGHT         # only hard obstacles (∞) block a shortcut; lets the
                               # smoother straighten through the soft band (A* still
                               # prefers clearance) — avoids 16-vertex zig-zag detours
    out = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not _line_of_sight(grid, path[i], path[j], keep):
            j -= 1
        out.append(path[j])
        i = j
    return out


# ── Lidar visualisation ───────────────────────────────────────────────────────
def render_lidar_png(scan, path, goal_local, detections=None, look_local=None, goal_mode='move',
                     ball_goal_local=None, ball_local=None, push_from_local=None) -> bytes:
    img   = np.full((VIZ_SIZE, VIZ_SIZE, 3), (18,18,18), dtype=np.uint8)
    cx = cy = VIZ_SIZE // 2
    scale   = VIZ_SIZE / (GRID_CELLS * GRID_RES)

    def w2p(x, y):
        # base_link (x=forward, y=left) -> screen: forward up, +y (left) on the LEFT.
        return int(cx - y*scale), int(cy - x*scale)

    for d in range(-6, 7):
        cv2.line(img, (int(cx+d*scale),0), (int(cx+d*scale),VIZ_SIZE), (35,35,35), 1)
        cv2.line(img, (0,int(cy+d*scale)), (VIZ_SIZE,int(cy+d*scale)), (35,35,35), 1)
    for rm in (1, 2, 3):
        cv2.circle(img, (cx,cy), int(rm*scale), (40,40,40), 1)

    if scan:
        ang = np.arange(len(scan.ranges))*scan.angle_increment + scan.angle_min
        rng = np.array(scan.ranges, np.float32)
        ok  = np.isfinite(rng)&(rng>=scan.range_min)&(rng<=scan.range_max)
        # Same base_link transform as build_grid (RPLIDAR yaw +90°): fwd=-r·sinθ,
        # left=r·cosθ — drawn through the corrected w2p so it matches goal/path/dets.
        xs_draw = -rng[ok] * np.sin(ang[ok])
        ys_draw =  rng[ok] * np.cos(ang[ok])
        for x, y in zip(xs_draw, ys_draw):
            cv2.circle(img, w2p(x, y), 2, (0,200,255), -1)

    if path and len(path) > 1:
        half = GRID_CELLS // 2
        pts  = [w2p((r-half)*GRID_RES, (c-half)*GRID_RES) for r,c in path]
        for i in range(len(pts)-1):
            cv2.line(img, pts[i], pts[i+1], (0,255,120), 2)

    # Lookahead target (the moving local aim point) — small yellow dot.
    if look_local:
        lp = w2p(*look_local)
        cv2.circle(img, lp, 5, (0, 220, 255), -1)
        cv2.circle(img, lp, 8, (0, 220, 255), 1)

    # Final destination (static, locked to the world goal) — distinct green
    # crosshair + diamond, clearly different from the yellow lookahead.
    if goal_local:
        gp = w2p(*goal_local)
        col, lbl = ((40, 140, 255), "PUSH") if goal_mode == 'push' else ((50, 255, 50), "GOAL")
        cv2.drawMarker(img, gp, col, cv2.MARKER_DIAMOND, 18, 2)
        cv2.drawMarker(img, gp, col, cv2.MARKER_CROSS, 26, 1)
        cv2.putText(img, lbl, (gp[0]+12, gp[1]-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, col, 1, cv2.LINE_AA)

    # Camera (OAK-D) spatial detections — magenta markers at their robot-frame
    # (x_loc=forward, y_loc=left) position, same frame as the lidar points.
    for det in (detections or []):
        if det.get('x_loc') is None:
            continue
        px = w2p(det['x_loc'], det['y_loc'])
        cv2.circle(img, px, 7,  (255, 0, 255), -1)      # magenta fill
        cv2.circle(img, px, 10, (255, 255, 255), 1)     # white ring
        cv2.putText(img, f"{det.get('label','?')} {det.get('conf',0):.0%}",
                    (px[0]+9, px[1]-6), cv2.FONT_HERSHEY_SIMPLEX, 0.38,
                    (255, 0, 255), 1, cv2.LINE_AA)

    # Ball-push task layer (PUSH_TO_GOAL): the human-set goal, the marked ball,
    # the stance point on the goal↔ball line, and the line the ball must travel.
    if ball_goal_local:
        bg = w2p(*ball_goal_local)
        if ball_local:                               # the push line: ball -> goal
            bl = w2p(*ball_local)
            for i in range(0, 20, 2):                # dashed (cv2 has no dashes)
                t0, t1 = i / 20.0, min(1.0, (i + 1) / 20.0)
                cv2.line(img, (int(bl[0] + (bg[0]-bl[0])*t0), int(bl[1] + (bg[1]-bl[1])*t0)),
                              (int(bl[0] + (bg[0]-bl[0])*t1), int(bl[1] + (bg[1]-bl[1])*t1)),
                         (255, 200, 0), 1)
        cv2.drawMarker(img, bg, (255, 200, 0), cv2.MARKER_DIAMOND, 20, 2)
        cv2.drawMarker(img, bg, (255, 200, 0), cv2.MARKER_TILTED_CROSS, 12, 2)
        cv2.putText(img, "BALL GOAL", (bg[0]+13, bg[1]+4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (255, 200, 0), 1, cv2.LINE_AA)
    if ball_local:
        bp = w2p(*ball_local)
        cv2.circle(img, bp, 6, (0, 140, 255), -1)
        cv2.circle(img, bp, 9, (255, 255, 255), 1)
        cv2.putText(img, "BALL", (bp[0]+11, bp[1]+14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.36, (0, 140, 255), 1, cv2.LINE_AA)
    if push_from_local:
        sp = w2p(*push_from_local)
        cv2.circle(img, sp, 7, (230, 230, 230), 1)
        cv2.putText(img, "STANCE", (sp[0]+10, sp[1]-8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.34, (200, 200, 200), 1, cv2.LINE_AA)

    cv2.circle(img, (cx,cy), 8, (30,100,255), -1)
    cv2.arrowedLine(img, (cx,cy), (cx,cy-22), (200,200,255), 2, tipLength=0.4)
    cv2.putText(img, "1m", (cx+int(scale)+3, cy-3),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (60,60,60), 1)

    _, buf = cv2.imencode('.png', img)
    return bytes(buf)


# ── Detection overlay ─────────────────────────────────────────────────────────
# Named color palette for tagging detections. The box's dominant colour is
# snapped to the nearest of these (Lab distance); anything else picks the closest.
_NAMED_RGB = {
    'red': (210,25,25), 'pink': (250,150,190), 'orange': (245,140,20), 'yellow': (240,220,40),
    'green': (45,165,60), 'blue': (35,85,215), 'purple': (150,55,190), 'black': (30,30,30),
}
_NAMED_LAB = {n: cv2.cvtColor(np.uint8([[list(rgb)]]), cv2.COLOR_RGB2LAB)[0, 0].astype(float)
              for n, rgb in _NAMED_RGB.items()}

# COCO_CLASSES used to live here: a byte-identical copy of COCO_LABELS above (the
# same 80 labels, in the same order — and models/nn_*.json held three more copies).
# Use COCO_LABELS. The detector's class_id vocabulary and the follow-target
# vocabulary are the same vocabulary and must not be able to drift.

_FOLLOW_SYNONYM = {   # everyday word -> the model's label
 'couch':'sofa','tv':'tvmonitor','television':'tvmonitor','monitor':'tvmonitor','plant':'pottedplant',
 'potted plant':'pottedplant','table':'diningtable','dining table':'diningtable','motorcycle':'motorbike',
 'airplane':'aeroplane','plane':'aeroplane','phone':'cell phone','cellphone':'cell phone',
 'ball':'sports ball','puppy':'dog','doggy':'dog','kitty':'cat','fridge':'refrigerator',
 'teddy':'teddy bear','bike':'bicycle'}

# Words that mean "follow". This is the ONLY list that decides whether a phrase is
# a follow command — and since programs/follow.toy is the only follow program, it
# also decides which phrases reach it. 'track' used to appear only in the .toy
# `# triggers:` lines and not here; that drift is what made follow_dog.toy
# impossible to merge ("track the dog" reached a dog only via that file's literal
# trigger, because _parse_follow_target returned None and /run fell through).
_FOLLOW_VERBS = (' follow ',' trail ',' tail ',' chase ',' track ',' come with ',
                 ' stay with ',' keep near ',' keep close ')

# Small, low targets that need the ball treatment: a closer standoff AND a
# step-and-stare SCAN_FOR before following begins. Both come from the same
# physics — at 1 m the 416-px preview loses a ~40 cm ball, and FOLLOW's
# re-acquire is a CONTINUOUS spin, which the OAK detects poorly through (motion
# blur at low fps), so it sails straight past. Anything not listed gets the
# person/dog treatment: start following at once (FOLLOW's own sweep re-finds a
# big target fine) at a 1 m standoff. This is the one place to add a newly-tuned
# small object; only the ball case is field-proven, so keep the list honest.
_FOLLOW_CLOSE = {'sports ball','apple','bottle','cup','wine glass','frisbee','teddy bear'}

# Words that mean "follow it hard": use the lidar + velocity predictor
# (follow_ball_fast) instead of the plain camera loop. Honoured ONLY for
# _FOLLOW_CLOSE targets — the predictor gates on _lidar_ball(), which hunts a
# close convex blob sticking out nearer than the wall behind it. That is a sensor
# model for a ~40 cm object on the floor; it is wrong for a person or a dog, and
# unnecessary, since the camera holds those in frame at ~2 fps anyway.
_FOLLOW_FAST_WORDS = ('aggressive','aggressively','fast','faster','quick','quickly',
                      'hard','predict','race','sprint','track','tracking')

def _norm_phrase(text):
    """Lowercase, drop non-letters, pad with spaces so words can be matched whole."""
    return ' ' + re.sub(r'\s+',' ', re.sub(r'[^a-z ]',' ',(text or '').lower())) + ' '

def _parse_named_target(text):
    """The COCO object named in `text` (the model's own label, incl. synonyms
    couch->sofa), else None. Deliberately does NOT require a follow verb — the
    callers decide whether the phrase is a follow command."""
    t = _norm_phrase(text)
    for word, label in sorted(list(_FOLLOW_SYNONYM.items()) + [(c, c) for c in COCO_LABELS],
                              key=lambda kv: -len(kv[0])):
        if ' ' + word + ' ' in t:
            return label
    return None


def _parse_follow_target(text):
    """For a 'follow ...' command, the COCO object to follow; None if the phrase
    is not a follow command, or names nothing ("follow me" -> None -> person)."""
    t = _norm_phrase(text)
    if not any(v in t for v in _FOLLOW_VERBS):
        return None
    return _parse_named_target(text)


def _parse_follow_fast(text):
    """True if the phrase asks to follow hard (see _FOLLOW_FAST_WORDS)."""
    t = _norm_phrase(text)
    return any((' ' + w + ' ') in t for w in _FOLLOW_FAST_WORDS)

def _is_close_target(name):
    """Does this target need the ball treatment (close standoff + scan first)?

    Accepts both the COCO label ('sports ball', what _parse_follow_target
    returns) and the everyday names a hand-written .toy or the detector's own
    aliasing produces ('ball', 'apple') — _live_det already treats a ball and an
    apple as the same object, so the follow parameters must agree with it.
    """
    n = (name or '').lower()
    return n in _FOLLOW_CLOSE or 'ball' in n

def _follow_params(text):
    """Resolve a spoken phrase into everything `programs/follow.toy` needs.

    ONE place decides follow behaviour. It used to be spread over four .toy files
    (follow / follow_dog / follow_ball / follow_ball_aggressive), each hard-coding
    a target and a standoff — and two of them were unreachable, because /run's
    generic-follow special case fires before the matcher for any phrase naming a
    COCO object. Now: one program, one table, and the target-specific knowledge
    (standoff, whether to scan first, whether to predict) lives here where it can
    be unit-tested instead of being encoded in DSL text.

    Returns {'target','standoff','scan_first','fast'}. target None means the
    phrase named nothing ("follow me"), which follow.toy reads as 'person'.
    """
    target = _parse_follow_target(text)
    fast   = _parse_follow_fast(text)
    if target is None and fast:
        # "predict the ball" asks for the fast treatment but uses no follow verb,
        # so _parse_follow_target found nothing. Honour it only when a small/low
        # target is actually named — that is the only case the predictor's lidar
        # model is valid for, so a stray 'fast' elsewhere stays a no-op.
        named = _parse_named_target(text)
        if _is_close_target(named):
            target = named
    close  = _is_close_target(target)
    return {
        'target':     target,
        'standoff':   FOLLOW_STANDOFF_CLOSE if close else FOLLOW_STANDOFF,
        'scan_first': close,
        'fast':       close and fast,
    }


def _parse_color(text):
    """First palette colour named in `text` (for colour-targeted commands), else None."""
    words = set(re.findall(r'[a-z]+', (text or '').lower()))
    for c in _NAMED_RGB:
        if c in words:
            return c
    return None


def _detect_color(frame, x_px, y_px, w_px, h_px):
    """Dominant colour name of a detection box (centre 60%), snapped to the named
    palette via nearest Lab. `frame` is BGR (cv2 order); box is in the 416 preview
    frame. None if no frame / empty patch."""
    if frame is None:
        return None
    H, W = frame.shape[:2]
    sx, sy = W / 416.0, H / 416.0
    cx, cy = x_px * sx, y_px * sy
    hw, hh = w_px * sx * 0.30, h_px * sy * 0.30
    x0, x1 = max(0, int(cx - hw)), min(W, int(cx + hw))
    y0, y1 = max(0, int(cy - hh)), min(H, int(cy + hh))
    if x1 <= x0 or y1 <= y0:
        return None
    med = np.median(frame[y0:y1, x0:x1].reshape(-1, 3), axis=0).astype(np.uint8)
    lab = cv2.cvtColor(med.reshape(1, 1, 3), cv2.COLOR_BGR2LAB)[0, 0].astype(float)
    return min(_NAMED_LAB, key=lambda n: float(np.sum((lab - _NAMED_LAB[n]) ** 2)))


def _add_detection_obstacles(grid, dets, goal_local, excl_r=0.40, obj_r=0.12):
    """Mark each detected object as a no-go disk in the robot-frame cost grid,
    EXCEPT the one at the goal (the push target). Lets A* route around the OTHER
    balls/objects -- even ones the lidar can't see -- while still driving into the
    target. Robot frame matches build_grid (row = fwd, col = left)."""
    half = GRID_CELLS // 2
    rad = int((obj_r + ROBOT_R) / GRID_RES)
    Y, X = np.ogrid[:GRID_CELLS, :GRID_CELLS]
    for d in dets:
        xl, yl = d.get('x_loc'), d.get('y_loc')
        if xl is None or yl is None:
            continue
        if goal_local and math.hypot(xl - goal_local[0], yl - goal_local[1]) < excl_r:
            continue                                   # this one is the target
        r = int(half + xl / GRID_RES); c = int(half + yl / GRID_RES)
        grid[(Y - r) ** 2 + (X - c) ** 2 <= rad * rad] = np.inf


def render_image_with_detections(frame_rgb, detections) -> bytes:
    if frame_rgb is None:
        ph = np.full((250,250,3), (18,18,18), dtype=np.uint8)
        cv2.putText(ph, 'waiting...', (50,130),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (80,80,80), 1)
        _, buf = cv2.imencode('.jpg', ph, [cv2.IMWRITE_JPEG_QUALITY, 75])
        return bytes(buf)
    
    img = frame_rgb.copy()
    # depthai gives the 2D pixel bbox (center + size) in the preview frame via
    # _nn_cb; draw a box + "label conf% dist" for each detection.
    for det in detections:
        if 'x_px' not in det:
            continue
        x, y = int(det['x_px']), int(det['y_px'])
        w, h = int(det['w_px']), int(det['h_px'])
        x1, y1 = max(0, x - w//2), max(0, y - h//2)
        x2, y2 = min(img.shape[1], x + w//2), min(img.shape[0], y + h//2)
        color = (0, 255, 0)
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
        col = (det.get('color') or '') if det.get('label') != 'person' else ''   # never colour-label a person
        label = f"{col} {det['label']} {det['conf']:.0%} {det.get('dist', 0):.1f}m".strip()
        ytxt = y1 - 6 if y1 > 16 else y2 + 14
        cv2.putText(img, label, (x1, ytxt),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)

    _, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 75])
    return bytes(buf)


# ── ROS node ──────────────────────────────────────────────────────────────────
def _lidar_range_at_bearing(scan, bearing_rad, window_deg=3.0):
    """Lidar range (m) toward a base_link bearing (rad, +left), or None.
    Gives object distance from the laser when OAK stereo depth is junk
    (smooth/low objects read the background). base_link->lidar per build_grid:
    base_x=-r*sin(ang), base_y=+r*cos(ang) => ang = atan2(-cos, sin)."""
    if scan is None or not scan.ranges:
        return None
    target = math.atan2(-math.cos(bearing_rad), math.sin(bearing_rad))
    angs = np.arange(len(scan.ranges)) * scan.angle_increment + scan.angle_min
    rng  = np.asarray(scan.ranges, np.float32)
    dang = np.abs((angs - target + np.pi) % (2*np.pi) - np.pi)
    sel  = (dang <= math.radians(window_deg)) & np.isfinite(rng) & (rng >= scan.range_min) & (rng <= scan.range_max)
    if not np.any(sel):
        return None
    return float(np.median(rng[sel]))


class NavNode(Node):
    def __init__(self):
        super().__init__('tb4_claude_nav')
        qos_s = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                           durability=DurabilityPolicy.VOLATILE,
                           history=HistoryPolicy.KEEP_LAST, depth=1)
        qos_r = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.VOLATILE,
                           history=HistoryPolicy.KEEP_LAST, depth=1)
        qos_cam = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE,
                             history=HistoryPolicy.KEEP_LAST, depth=5)
        self._depth = None
        self.create_subscription(LaserScan,        '/scan',                       self._scan_cb,  qos_s)
        self.create_subscription(Image,            CAM_TOPIC,                     self._img_cb,   qos_cam)
        self.create_subscription(Image,            DEPTH_TOPIC,                   self._depth_cb, qos_cam)
        self.create_subscription(Odometry,         '/odom',                       self._odom_cb,  qos_r)
        self.create_subscription(Detection3DArray, NN_TOPIC,                      self._nn_cb,    qos_s)
        self.create_subscription(BatteryState,     '/battery_state',              self._battery_cb, qos_s)
        self.create_subscription(DockStatus,       '/dock_status',                self._dock_cb,  qos_s)
        self.create_subscription(HazardDetectionVector, '/hazard_detection',       self._hazard_cb, qos_s)
        self._pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.audio_pub = self.create_publisher(AudioNoteVector, '/cmd_audio', 10) if _HAS_AUDIO else None
        self._dock_ac   = ActionClient(self, Dock,   '/dock')
        self._undock_ac = ActionClient(self, Undock, '/undock')
        self._motor_cli = self.create_client(Empty, '/start_motor')

    def restart_lidar_motor(self):
        """Spin the RPLIDAR motor back up (the TB4 stops it on dock and does
        not always restart it on undock). Blocks briefly to ensure the
        /start_motor server is matched, then fires the request."""
        try:
            if not self._motor_cli.wait_for_service(timeout_sec=2.0):
                self.get_logger().warning('lidar self-heal: /start_motor not available')
                return False
            self._motor_cli.call_async(Empty.Request())
            self.get_logger().info('lidar self-heal: called /start_motor (scan stale)')
            return True
        except Exception as e:
            self.get_logger().warning(f'lidar self-heal: start_motor failed: {e}')
            return False

    def restart_camera(self):
        """Relaunch the OAK driver (run_oakd.sh, detached) — recovers a frozen
        OAK-D Lite (USB2 stalls leave it publishing nothing). True if spawned."""
        try:
            sh = os.path.join(os.path.dirname(os.path.abspath(__file__)), os.path.join('object_detection', 'run_oakd.sh'))
            subprocess.Popen(['bash', sh], stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL, stdin=subprocess.DEVNULL,
                             start_new_session=True)
            self.get_logger().info('camera self-heal: relaunching OAK (run_oakd.sh)')
            return True
        except Exception as e:
            self.get_logger().warning(f'camera self-heal: relaunch failed: {e}')
            return False

    def _scan_cb(self, msg: LaserScan):
        with _lock:
            _state['scan']      = msg
            _state['scan_t']    = time.time()
            _ox, _oy, _oyaw = _state['odom_x'], _state['odom_y'], _state['odom_yaw']
            _loc = lambda pt: (_odom_to_local(pt, _ox, _oy, _oyaw) if pt else None)
            _state['lidar_png'] = render_lidar_png(
                msg, _state['path'], _state['goal_local'], _state['detections'],
                _state['look_local'], _state['goal_mode'],
                ball_goal_local=_loc(_state['ball_goal']),
                ball_local=_loc(_state['ball_mark']),
                push_from_local=_loc(_state['push_from']))

    def _img_cb(self, msg: Image):
        try:
            arr = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.width, -1)
            if msg.encoding.lower() == 'rgb8':
                arr = arr[:, :, ::-1]
            _set(frame_rgb=arr, img_t=time.time())
        except Exception as e:
            self.get_logger().warning(f'img: {e}')

    def _depth_cb(self, msg: Image):
        try:
            if '16' in msg.encoding:
                self._depth = np.frombuffer(bytes(msg.data), np.uint16).reshape(msg.height, msg.width)
        except Exception:
            pass

    def _p25_depth(self, x_px, y_px, w_px, h_px):
        """p25 of stereo depth (m) over the box; maps 416 preview -> depth via
        center-square-crop geometry. None if no depth/too few valid pixels."""
        d = self._depth
        if d is None:
            return None
        H, W = d.shape
        sc = H / 416.0; xoff = (W - H) / 2.0
        cx = xoff + x_px * sc; cy = y_px * sc
        hw = (w_px / 2.0) * sc; hh = (h_px / 2.0) * sc
        x0 = max(0, int(cx - hw)); x1 = min(W, int(cx + hw))
        y0 = max(0, int(cy - hh)); y1 = min(H, int(cy + hh))
        v = d[y0:y1, x0:x1]; v = v[v > 0]
        if v.size < 10:
            return None
        return float(np.percentile(v, 25)) / 1000.0

    def _nn_cb(self, msg: Detection3DArray):
        """On-camera YOLOv8 spatial detections (Detection3DArray, oakd optical
        frame). class_id is a numeric COCO index string mapped to a name; distance
        comes from p25 of the stereo depth over the box (see _p25_depth)."""
        dets = []
        frame = _get('frame_rgb')
        for det in msg.detections:
            if not det.results:
                continue
            hyp = det.results[0].hypothesis
            if float(hyp.score) < DET_CONF_MIN:        # drop low-confidence false positives
                continue
            p   = det.results[0].pose.pose.position
            cam_x, cam_y = float(p.z), float(-p.x)        # OAK stereo (forward, left)
            cam_dist = math.hypot(cam_x, cam_y)
            cid = str(hyp.class_id)
            try:
                label = COCO_LABELS[int(cid)]
            except (ValueError, IndexError):
                label = cid
            # depthai stuffs the 2D pixel bbox into Detection3D.bbox (center +
            # size, in the 416×416 preview frame) — use it for the image overlay.
            bc, bs = det.bbox.center.position, det.bbox.size
            x_px=float(bc.x); y_px=float(bc.y); w_px=float(bs.x); h_px=float(bs.y)
            bearing = -math.atan((x_px - CAM_CX) / CAM_FX)   # real preview intrinsics
            ld = _lidar_range_at_bearing(_get("scan"), bearing)
            p25 = self._p25_depth(x_px, y_px, w_px, h_px)
            if _get("use_lidar_dist"):
                # p25 of stereo depth over the box (correct box->depth align) reads
                # the ball true even when the spatial NN reports the wall behind it.
                if p25 is not None:
                    dist = p25
                else:
                    cands = [dd for dd in (cam_dist, ld) if dd is not None and math.isfinite(dd) and dd >= ROBOT_R]
                    dist = min(cands) if cands else cam_dist
                x_loc = dist * math.cos(bearing)
                y_loc = dist * math.sin(bearing)
            else:
                x_loc, y_loc, dist = cam_x, cam_y, cam_dist   # raw OAK stereo
            dets.append(dict(
                label=label,
                conf=float(hyp.score),
                x_loc=x_loc, y_loc=y_loc,
                dist=float(dist),
                cam_dist=float(cam_dist), lidar_dist=(float(ld) if ld is not None else None),
                p25_dist=(float(p25) if p25 is not None else None),
                x_px=float(bc.x), y_px=float(bc.y),
                w_px=float(bs.x), h_px=float(bs.y),
                color=(_detect_color(frame, x_px, y_px, w_px, h_px) if label != 'person' else None),
            ))
        _set(detections=dets)

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose
        _set(odom_x=p.position.x, odom_y=p.position.y,
             odom_yaw=_quat_to_yaw(p.orientation), odom_t=time.time())

    def _battery_cb(self, msg: BatteryState):
        _set(battery_voltage=float(msg.voltage),
             battery_percentage=float(msg.percentage))

    def _dock_cb(self, msg: DockStatus):
        _set(docked=bool(msg.is_docked))

    def _hazard_cb(self, msg: HazardDetectionVector):
        """Collision/cliff/wheel-drop feedback. BUMP/CLIFF/WHEEL_DROP trigger an
        emergency stop of navigation; the active list drives the UI banner."""
        hz, emergency = [], False
        # In push mode a BUMP is intended (we're driving into the target), so it
        # doesn't stop us — but a STALL (object won't budge) does, to avoid pushing
        # an immovable object forever. Cliff/wheel-drop always stop.
        stop_set = ({2, 3, 4} if _get('goal_mode') == 'push' else HAZARD_STOP)
        for d in msg.detections:
            if d.type == 0:                       # backup_limit: constant near limit, ignore
                continue
            where = (d.header.frame_id or '').replace('bump_', '').replace('_', ' ')
            hz.append({'type': HAZARD_NAMES.get(d.type, str(d.type)), 'where': where})
            if d.type in stop_set:
                emergency = True
        _set(hazards=hz, last_hazard_t=time.time())
        if emergency and _get('nav_active'):
            self.stop()
            kinds = ', '.join(sorted({h['type'] for h in hz}))
            _set(nav_active=False, path=[], goal_local=None, goal_odom=None,
                 status=f'⚠ {kinds} — stopped')

    def send_twist(self, linear: float, angular: float):
        msg = TwistStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x  = float(linear)
        msg.twist.angular.z = float(angular)
        self._pub.publish(msg)
        # Ground truth for the dataset recorder: servo-driven skills (scan/
        # follow/push_through) bypass the planner, so the planner-derived cmd
        # in the log reads v=w=0 while the robot is actually moving.
        _set(cmd_sent=dict(v=float(linear), w=float(angular), t=time.time()))

    def stop(self):
        self.send_twist(0.0, 0.0)

    # ── Docking actions ──────────────────────────────────────────────────────
    def _send_goal(self, client, goal, accept_timeout=3.0):
        """Send an action goal and block until accepted, relying on the
        MultiThreadedExecutor (spinning in another thread) to service the
        futures. Returns True if the goal was accepted, else False."""
        if not client.wait_for_server(timeout_sec=accept_timeout):
            self.get_logger().warning('dock action server unavailable')
            return False
        fut = client.send_goal_async(goal)
        ev, box = threading.Event(), {}

        def _cb(f):
            box['gh'] = f.result()
            ev.set()

        fut.add_done_callback(_cb)
        if not ev.wait(accept_timeout):
            return False
        gh = box.get('gh')
        return gh is not None and gh.accepted

    def dock(self):
        """Stop nav, then send a Dock goal. Returns True if accepted."""
        self.stop()
        return self._send_goal(self._dock_ac, Dock.Goal())

    def undock(self):
        """Stop nav, then send an Undock goal. Returns True if accepted."""
        self.stop()
        return self._send_goal(self._undock_ac, Undock.Goal())


# ── Vision matching ───────────────────────────────────────────────────────────
def _keyword_match(description: str, detections: list) -> list:
    """Return detections whose label overlaps with any word in the description."""
    words = re.split(r'\W+', description.lower())
    matches = []
    for d in detections:
        label = d['label'].lower().replace('_', ' ')
        if any(w in label or label in w for w in words if len(w) > 2):
            matches.append(d)
    return matches


def find_target(description: str) -> tuple:
    """
    Returns (gx_loc, gy_loc, found: bool, info: str) — goal in robot-local frame.

    Distance + bearing come straight from the OAK-D's on-camera spatial
    detection (stereo depth), so no lidar-bearing estimate is needed.

    Priority:
      1. YOLO keyword match → single hit: use directly
      2. YOLO keyword match → multiple hits: pick nearest
    """
    with _lock:
        detections = list(_state['detections'])

    kw = _keyword_match(description, detections)
    if not kw:
        return 0.0, 0.0, False, 'not found'

    matched = kw[0] if len(kw) == 1 else min(kw, key=lambda d: d['dist'])
    info    = f"YOLO: {matched['label']} ({matched['conf']:.0%}, {matched['dist']:.2f} m)"
    return matched['x_loc'], matched['y_loc'], True, info


# ── Dataset recorder ────────────────────────────────────────────────────────
DATASET_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "datasets")


class DriveRecorder:
    """One directory per drive; per replan saves the full planner pipeline —
    raw lidar scan, the cost grid, the raw A* path, the smoothed path, plus
    pose/goal/lookahead/command/detections — as a step_*.npz (+ manifest.jsonl)
    for offline review and model training. All I/O is best-effort; a failure
    here must never disturb the control loop."""
    def __init__(self):
        self.dir  = None
        self.step = 0

    @property
    def active(self):
        return self.dir is not None

    def start(self, goal_odom, pose):
        try:
            ts = time.strftime("%Y%m%d_%H%M%S")
            self.dir = os.path.join(DATASET_DIR, f"drive_{ts}")
            os.makedirs(self.dir, exist_ok=True)
            self.step = 0
            with open(os.path.join(self.dir, "meta.json"), "w") as f:
                json.dump(dict(start_time=ts,
                               goal_odom=list(goal_odom) if goal_odom else None,
                               start_pose=list(pose), grid_res=GRID_RES,
                               grid_cells=GRID_CELLS, robot_r=ROBOT_R,
                               inflate_r=INFLATE_R, cost_scale=COST_SCALE,
                               cost_weight=COST_WEIGHT, drive_speed=DRIVE_SPEED,
                               turn_speed=TURN_SPEED, goal_tol=GOAL_TOL,
                               heading_tol=HEADING_TOL, lookahead_m=LOOKAHEAD_M,
                               replan_hz=REPLAN_HZ), f, indent=2)
            _set(record_drive=os.path.basename(self.dir), record_steps=0)
            print(f"[recorder] recording → {self.dir}")
        except Exception as e:
            print(f"[recorder] start failed: {e}"); self.dir = None

    def log(self, scan, cost_grid, raw_path, smoothed_path, meta, frame=None, frame_dets=None):
        if not self.active:
            return
        try:
            meta = dict(meta); meta["step"] = self.step; meta["t"] = time.time()
            # Save the OAK-D RGB preview (with detection boxes) next to the npz.
            if frame is not None:
                jpg = render_image_with_detections(frame, frame_dets or [])
                name = f"step_{self.step:05d}.jpg"
                with open(os.path.join(self.dir, name), "wb") as f:
                    f.write(jpg)
                meta["image"] = name
            np.savez_compressed(
                os.path.join(self.dir, f"step_{self.step:05d}.npz"),
                scan_ranges=np.asarray(scan.ranges, np.float32),
                cost_grid=np.asarray(cost_grid, np.float32),
                raw_path=np.asarray(raw_path, np.int32).reshape(-1, 2),
                smoothed_path=np.asarray(smoothed_path, np.int32).reshape(-1, 2),
                meta=json.dumps(meta))
            with open(os.path.join(self.dir, "manifest.jsonl"), "a") as f:
                f.write(json.dumps(meta) + "\n")
            self.step += 1
            _set(record_steps=self.step)
        except Exception as e:
            print(f"[recorder] log failed: {e}")

    def stop(self, status):
        if not self.active:
            return
        try:
            with open(os.path.join(self.dir, "END.json"), "w") as f:
                json.dump(dict(status=status, steps=self.step,
                               end_time=time.strftime("%Y%m%d_%H%M%S")), f, indent=2)
        except Exception:
            pass
        print(f"[recorder] drive saved: {self.dir} ({self.step} steps)")
        _set(record_drive="", record_steps=0)
        self.dir, self.step = None, 0


# ── Navigation loop ───────────────────────────────────────────────────────────
def nav_loop():
    global ros_node
    last_replan = 0.0
    last_record = 0.0
    plan_fail_t = 0.0   # when A* first failed to find a path (0 = not failing)
    stuck_x = stuck_y = stuck_yaw = 0.0; stuck_t = 0.0   # no-progress (pinned) watchdog
    path: list  = []
    raw_path: list = []
    look_odom   = None   # lookahead target anchored in the odom/world frame
    look_local  = None
    goal_local  = None
    grid        = None
    gr = gc     = GRID_CELLS // 2
    recorder    = DriveRecorder()
    loop_start  = time.time(); last_motor_kick = 0.0; last_cam_kick = 0.0   # self-heal

    while True:
        with _lock:
            active      = _state['nav_active']
            desc        = _state['destination']
            direct_goal = _state['direct_goal']
            goal_odom   = _state['goal_odom']
            goal_mode   = _state['goal_mode']
            (ox, oy, oyaw) = (_state['odom_x'], _state['odom_y'], _state['odom_yaw'])
            scan        = _state['scan']
            scan_t      = _state['scan_t']
            img_t       = _state['img_t']
            run_active  = _state['run_active']
            docked      = _state['docked']
            status      = _state['status']
            dets_snap   = list(_state['detections'])
            frame_rgb   = _state['frame_rgb']
            recording   = _state['recording']
            hazards      = _state['hazards']
            last_haz_t   = _state['last_hazard_t']

        now = time.time()

        # LiDAR self-heal: the TB4 stops the RPLIDAR motor when it docks and
        # doesn't always restart it on undock, leaving /scan with a publisher
        # but no data (empty BEV, blind navigation). If scans go stale while
        # undocked, spin the motor back up (rate-limited, after a startup grace).
        scan_stale = (scan is None) or (now - scan_t > 4.0)
        if scan_stale and (docked is not True) and (now - loop_start > 6.0) and (now - last_motor_kick > 15.0):
            if ros_node and ros_node.restart_lidar_motor():
                _set(status='⟳ lidar stalled — restarting motor')
            last_motor_kick = now

        # Camera self-heal: the OAK-D Lite USB2 link occasionally freezes (stops
        # publishing). Restart it ONLY while idle (never mid-run — the relaunch
        # takes ~30s), rate-limited: a quiet between-runs safety net.
        # Skip while DOCKED: turtlebot4_node deliberately calls oakd/stop_camera
        # on the dock (power save), so stale frames there are expected, not a
        # freeze. Without this guard the relaunch races bringup for the OAK and
        # the loser retry-loops on X_LINK_DEVICE_ALREADY_IN_USE forever.
        cam_stale = (now - img_t > 5.0)
        if cam_stale and (docked is not True) and (not run_active) and (not active) and (now - loop_start > 8.0) and (now - last_cam_kick > 60.0):
            if ros_node and ros_node.restart_camera():
                _set(status='⟳ camera frozen — restarting OAK')
            last_cam_kick = now

        # Auto-clear the hazard banner ~2s after the last hazard msg (this
        # firmware's /hazard_detection is event-driven — silent when clear).
        if hazards and now - last_haz_t > 2.0:
            _set(hazards=[])

        # ── Recording lifecycle — UI-controlled, independent of navigation ──
        if recording and not recorder.active:
            recorder.start(goal_odom, (ox, oy, oyaw))
        elif not recording and recorder.active:
            recorder.stop(status)

        if active:
            # Vision: find target
            if desc and not direct_goal:
                gx_loc, gy_loc, found, info = find_target(desc)
                if found:
                    goal_odom = (ox + gx_loc*math.cos(oyaw) - gy_loc*math.sin(oyaw),
                                oy + gx_loc*math.sin(oyaw) + gy_loc*math.cos(oyaw))
                    _set(goal_odom=goal_odom, match_info=info,
                         status=f'found: {info}', destination='')

            # Replan
            if now >= last_replan + 1.0/REPLAN_HZ and goal_odom and scan:
                last_replan = now
                grid  = build_grid(scan)
                half  = GRID_CELLS // 2
                sr, sc = half, half
                # The grid is in the robot/base frame, so the odom-frame goal
                # delta MUST be rotated by -yaw into it — otherwise the A* goal
                # cell is only correct at yaw≈0 and lands in an obstacle once
                # the robot turns.
                goal_x = goal_odom[0] - ox
                goal_y = goal_odom[1] - oy
                goal_local = (goal_x * math.cos(oyaw) + goal_y * math.sin(oyaw),
                             -goal_x * math.sin(oyaw) + goal_y * math.cos(oyaw))
                gr    = int(half + goal_local[0] / GRID_RES)
                gc    = int(half + goal_local[1] / GRID_RES)
                # Keep the A* goal REACHABLE: if it's off-grid (target farther than
                # the ~3 m grid half-width — e.g. a person across the room) or on an
                # obstacle, project it back along its bearing to the nearest in-grid
                # FREE cell, so the robot drives TOWARD a far target instead of
                # sitting still with an empty path. (This was why follow_human stalled
                # when the person was >3 m away.)
                if not (0 <= gr < GRID_CELLS and 0 <= gc < GRID_CELLS) or math.isinf(grid[gr, gc]):
                    lim = half - 2
                    dr, dc = gr - half, gc - half
                    m = max(abs(dr), abs(dc), 1)
                    s0 = min(1.0, lim / m)
                    dr, dc = dr * s0, dc * s0
                    for s in (1.0, 0.85, 0.7, 0.55, 0.4, 0.28, 0.18):
                        ggr, ggc = int(half + dr * s), int(half + dc * s)
                        if 0 <= ggr < GRID_CELLS and 0 <= ggc < GRID_CELLS and not math.isinf(grid[ggr, ggc]):
                            gr, gc = ggr, ggc; break
                # Other detected objects (balls the lidar can't see) become no-go
                # disks so the push routes around them -- the target (at the goal)
                # is excluded so we still drive into it.
                if goal_mode == 'push':
                    _add_detection_obstacles(grid, _get('detections'), goal_local)
                # PUSH mode: the target IS an object (an obstacle in the grid), so
                # plan on a copy with the cost cleared in a disk around the goal —
                # A* then paths INTO the object instead of routing around it.
                plan_grid = grid
                if goal_mode == 'push' and 0 <= gr < GRID_CELLS and 0 <= gc < GRID_CELLS:
                    plan_grid = grid.copy()
                    rr = int(PUSH_CLEAR_R / GRID_RES)
                    Y, X = np.ogrid[:GRID_CELLS, :GRID_CELLS]
                    plan_grid[(Y-gr)**2 + (X-gc)**2 <= rr*rr] = 0.0
                raw_path = astar(plan_grid, (sr,sc), (gr,gc)) or []   # intermediate
                path     = smooth_path(plan_grid, raw_path)
                # Lookahead ~LOOKAHEAD_M along the (smoothed) path by arc length,
                # anchored in odom so it doesn't slide as the robot turns.
                look_odom, look_local = None, None
                if len(path) >= 2:
                    pts = [((r-half)*GRID_RES, (c-half)*GRID_RES) for r, c in path]
                    look_x, look_y = pts[-1]            # default: the goal vertex
                    acc, prev = 0.0, pts[0]
                    for px, py in pts[1:]:
                        seg = math.hypot(px-prev[0], py-prev[1])
                        if acc + seg >= LOOKAHEAD_M:
                            t = (LOOKAHEAD_M - acc) / seg if seg > 1e-9 else 0.0
                            look_x = prev[0] + t*(px-prev[0])
                            look_y = prev[1] + t*(py-prev[1])
                            break
                        acc += seg; prev = (px, py)
                    look_local = (look_x, look_y)
                    look_odom = (ox + look_x*math.cos(oyaw) - look_y*math.sin(oyaw),
                                 oy + look_x*math.sin(oyaw) + look_y*math.cos(oyaw))
                _set(path=path, goal_local=goal_local, look_local=look_local)

            # Control — explicit branches surface *why* it isn't driving.
            dist_goal = math.hypot(goal_odom[0]-ox, goal_odom[1]-oy) if goal_odom else 1e9
            if not scan:
                _set(status='waiting for lidar')
            elif not goal_odom:
                _set(status='no goal set')
            elif dist_goal < (PUSH_GOAL_TOL if goal_mode == 'push' else GOAL_TOL):
                # Arrival is checked BEFORE the path branches, so losing the A*
                # path on final approach (goal cell inflated as we close in)
                # still counts as arrived instead of stalling at "planning path".
                # Push uses a tighter tolerance so it drives onto the target.
                if ros_node: ros_node.send_twist(0.0, 0.0)
                done = 'pushed' if goal_mode == 'push' else 'arrived'
                _set(nav_active=False, goal_mode='move', status=f'{done} (δ={dist_goal:.2f}m)', path=[])
            elif not path or len(path) < 2 or look_odom is None:
                if plan_fail_t == 0.0:
                    plan_fail_t = now
                if now - plan_fail_t > PLAN_TIMEOUT:
                    if ros_node: ros_node.send_twist(0.0, 0.0)
                    _set(nav_active=False, goal_mode='move',
                         status='blocked: no path', path=[])
                else:
                    _set(status=f'planning path… (len={len(path)})')
            elif not ros_node:
                _set(status='ros_node not ready')
            else:
                plan_fail_t = 0.0
                # Re-project the odom-anchored lookahead into the CURRENT robot
                # frame each tick. Blend turn+drive: speed scales with
                # cos(bearing) — full ahead when aligned, 0 at ±90° (pure
                # rotation when the target is behind). Faster than turn-in-place,
                # and stable because the target is odom-anchored (no oscillation).
                lx = look_odom[0] - ox
                ly = look_odom[1] - oy
                target_x =  lx*math.cos(oyaw) + ly*math.sin(oyaw)
                target_y = -lx*math.sin(oyaw) + ly*math.cos(oyaw)
                bearing  = math.atan2(target_y, target_x) \
                           if math.hypot(target_x, target_y) > 1e-3 else 0.0
                w = max(-TURN_SPEED, min(TURN_SPEED, 2.0 * bearing))
                v = DRIVE_SPEED * max(0.0, math.cos(bearing))
                ros_node.send_twist(v, w)
                # no-progress watchdog: commanded to move but the pose isn't
                # changing -> pinned/stuck (base didn't flag a stall). Stop.
                if math.hypot(ox - stuck_x, oy - stuck_y) > 0.015 or \
                   abs((oyaw - stuck_yaw + math.pi) % (2*math.pi) - math.pi) > 0.04:
                    stuck_x, stuck_y, stuck_yaw, stuck_t = ox, oy, oyaw, now
                if now - stuck_t > STALL_TIMEOUT:
                    ros_node.send_twist(0.0, 0.0)
                    _set(nav_active=False, goal_mode='move', path=[],
                         status=('⚠ stall — stopped' if goal_mode == 'push' else 'blocked: stuck'))
                else:
                    verb = 'pushing' if goal_mode == 'push' else 'navigating'
                    _set(status=f'{verb} → ({goal_odom[0]:.2f}, {goal_odom[1]:.2f})')
        else:
            # Not navigating — clear path state; keep idle/arrived status.
            plan_fail_t = 0.0
            stuck_x, stuck_y, stuck_yaw, stuck_t = ox, oy, oyaw, now
            if path: _set(path=[], goal_local=None)
            path, raw_path, look_odom, look_local, goal_local = [], [], None, None, None
            _set(look_local=None)
            # A running ToyScript skill (push/follow/scan) servo-drives the base
            # itself with nav_active False, so it owns the status line — don't
            # stamp 'idle' over what it is actually reporting.
            if not run_active and not status.startswith(('idle', 'arrived', '⚠')):
                _set(status='idle')

        # ── Dataset logging — every ~1/REPLAN_HZ while recording (with or
        # without an active nav goal); captures scan + cost grid + planner state.
        if recorder.active and scan is not None and now >= last_record + 1.0/REPLAN_HZ:
            last_record = now
            # tag each step with the running ToyScript program + current command
            run_prog = _get('run_program') if _get('run_active') else None
            _ch = _get('cmd_history')
            cur_cmd = _ch[-1]['cmd'] if (_ch and _ch[-1].get('running')) else None
            rec_grid = grid if (active and grid is not None) else build_grid(scan)
            if look_local is not None:
                lb = math.atan2(look_local[1], look_local[0])
                cmd_w = max(-TURN_SPEED, min(TURN_SPEED, 2.0 * lb))
                cmd_v = DRIVE_SPEED * max(0.0, math.cos(lb))
            else:
                lb = cmd_v = cmd_w = 0.0
            recorder.log(scan, rec_grid, raw_path, path, dict(
                pose=[ox, oy, oyaw], nav_active=active,
                run_program=run_prog, current_cmd=cur_cmd,
                goal_odom=list(goal_odom) if goal_odom else None,
                goal_local=list(goal_local) if goal_local else None,
                goal_cell=[gr, gc] if active else None,
                look_odom=list(look_odom) if look_odom else None,
                look_local=list(look_local) if look_local else None,
                cmd=dict(v=cmd_v, w=cmd_w, bearing=lb),
                cmd_sent=_get('cmd_sent'),
                raw_path_len=len(raw_path), smoothed_path_len=len(path),
                scan_params=dict(angle_min=float(scan.angle_min),
                                 angle_increment=float(scan.angle_increment),
                                 range_min=float(scan.range_min),
                                 range_max=float(scan.range_max),
                                 n=len(scan.ranges)),
                detections=dets_snap),
                frame=frame_rgb, frame_dets=dets_snap)

        time.sleep(0.05)


# ── ToyScript runtime ───────────────────────────────────────────────────────────
# A tiny DSL (toyscript.py) for high-level tasks. Programs live in programs/*.toy;
# user text is matched to one and run in a thread that drives THIS navigator.
PROGRAMS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'programs')


class NavRobot:
    """ToyScript primitive bindings to the live navigator (one per run)."""
    def __init__(self): self._abort = False

    def abort(self): self._abort = True
    def _check(self):
        if self._abort: raise toyscript.StopProgram()

    def _wait(self, timeout=90):
        """Block until the nav goal completes (nav_active clears) or times out."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            self._check()
            if not _get('nav_active'):
                return self._classify(_get('status'))
            time.sleep(0.15)
        _set(nav_active=False)
        if ros_node: ros_node.stop()
        return 'timeout'

    @staticmethod
    def _classify(status):
        s = (status or '').lower()
        if s.startswith('arrived'): return 'arrived'
        if s.startswith('pushed'):  return 'pushed'
        if 'stall' in s or 'pinned' in s: return 'stalled'
        if 'blocked' in s or 'stuck' in s or 'no path' in s: return 'blocked'
        return 'done'

    def go_to(self, x, y):
        _set(goal_odom=(x, y), goal_mode='move', direct_goal=True, nav_active=True,
             path=[], goal_local=None, destination='', match_info='toyscript',
             status=f'navigating → ({x:.2f}, {y:.2f})')
        return self._wait()

    def push_to(self, x, y):
        _set(goal_odom=(x, y), goal_mode='push', direct_goal=True, nav_active=True,
             path=[], goal_local=None, destination='', match_info='toyscript',
             status=f'pushing → ({x:.2f}, {y:.2f})')
        return self._wait()

    def push_away(self, bx, by, step=0.40):
        """Push the ball toward the wall by driving (push mode) straight AT it.
        The goal is the ball itself, NOT a point beyond it, so the robot stops
        at the ball instead of sailing past to an empty point when the first
        fix is off-axis; the program then re-finds and pushes again from close
        range, where the bearing error is small. (`step` kept for the call
        signature.)"""
        return self.push_to(bx, by)

    def find(self, name):
        """Nearest current detection whose label matches `name`, as an odom point."""
        q = name.lower().replace('_', ' ').strip()
        ox, oy, oyaw = _get('odom_x'), _get('odom_y'), _get('odom_yaw')
        best = None; tc = _get('target_color')
        for d in _get('detections'):
            lbl = (d.get('label') or '').lower()
            if q in lbl or lbl in q or (q.split() and q.split()[0] in lbl):
                if tc and d.get('color') != tc: continue      # colour-targeted command
                if best is None or d.get('dist', 1e9) < best.get('dist', 1e9):
                    best = d
        if not best: return None
        xl, yl = best['x_loc'], best['y_loc']
        return {'x': round(ox + xl*math.cos(oyaw) - yl*math.sin(oyaw), 3),
                'y': round(oy + xl*math.sin(oyaw) + yl*math.cos(oyaw), 3)}

    def _scan_hit(self, name):
        h = self.find(name)
        if h is None and "ball" in name.lower():
            h = self.find("apple")
        return h

    def _center_ball(self, name, tol=0.12):
        """Rotate in place until `name` (or apple) is ~centred ahead, then
        settle and return its odom {x,y}. SCAN_FOR stops the instant the ball
        first appears — at the frame edge, where the fix is least reliable — so
        we centre it before locking."""
        if not ros_node: return None
        for _ in range(60):                              # up to ~6s of centring
            self._check()
            h = self._scan_hit(name)
            if h is None:
                ros_node.send_twist(0.0, 0.0); return None
            ox, oy, oyaw = _get('odom_x'), _get('odom_y'), _get('odom_yaw')
            bearing = math.atan2(h['y'] - oy, h['x'] - ox) - oyaw
            bearing = (bearing + math.pi) % (2 * math.pi) - math.pi
            if abs(bearing) < tol:
                break
            ros_node.send_twist(0.0, (SCAN_TURN_SPEED * 0.6) * (1.0 if bearing > 0 else -1.0))
            time.sleep(0.1)
        ros_node.send_twist(0.0, 0.0)
        time.sleep(0.5)                                  # settle, then re-measure centred
        return self._scan_hit(name)

    def push_through(self, dist=0.7):
        """Drive straight forward up to `dist`, stopping on no-progress (= wall
        reached / can't move). Finishes a ball that has dropped below the camera
        right in front of the robot, instead of rotating away to re-find it."""
        if not ros_node: return 'no-robot'
        sx, sy = _get('odom_x'), _get('odom_y')
        _set(status='pushing → straight through')
        t0 = last = time.time(); px, py = sx, sy
        while math.hypot(_get('odom_x') - sx, _get('odom_y') - sy) < dist and time.time() - t0 < 12:
            self._check()
            ros_node.send_twist(PUSH_THROUGH_SPEED, 0.0); time.sleep(0.05)
            cx, cy = _get('odom_x'), _get('odom_y')
            if math.hypot(cx - px, cy - py) > 0.003: last = time.time()
            elif time.time() - last > 2.0:
                ros_node.send_twist(0.0, 0.0); return 'pinned'   # wall / can't move
            px, py = cx, cy
        ros_node.send_twist(0.0, 0.0)
        return 'done'

    def _live_det(self, name):
        """Nearest current detection matching `name` (or apple, for a ball), in
        the robot frame: {x_loc, y_loc, dist, lidar_dist, ...}. None if unseen."""
        q = name.lower().replace('_', ' ').strip()
        best = None; tc = _get('target_color')
        for d in _get('detections'):
            lbl = (d.get('label') or '').lower()
            if q in lbl or lbl in q or ('ball' in q and 'apple' in lbl):
                if tc and d.get('color') != tc: continue      # colour-targeted command
                if best is None or d.get('dist', 1e9) < best.get('dist', 1e9):
                    best = d
        return best

    def _wall_goal(self, det):
        """Odom point at the WALL behind the ball: along the ball's bearing, at
        the lidar range there (the ball sits below the lidar plane, so that
        range is the wall), pulled back by the robot radius. Stable — doesn't
        jitter with the ball detection."""
        wall = det.get('lidar_dist')
        if wall is None or wall <= 0:
            wall = det.get('dist', 0.0) + 1.0          # fallback: 1m past the ball
        wall = max(0.3, wall - ROBOT_R)
        ox, oy, oyaw = _get('odom_x'), _get('odom_y'), _get('odom_yaw')
        a = oyaw + math.atan2(det['y_loc'], det['x_loc'])
        return (ox + wall * math.cos(a), oy + wall * math.sin(a))

    def _reposition_behind(self, name, goal):
        """Drive FORWARD (go_to, never reverse) to a standoff behind the ball on
        the far side from the goal, so the ball ends up between robot and goal
        again. The planner curves the robot around."""
        h = self.find(name)
        if h is None: return 'lost'
        dx, dy = h['x'] - goal[0], h['y'] - goal[1]    # goal -> ball
        d = math.hypot(dx, dy) or 1e-6
        sx = h['x'] + dx / d * PUSH_WALL_STANDOFF
        sy = h['y'] + dy / d * PUSH_WALL_STANDOFF
        _set(status='repositioning → behind the ball')
        r = self.go_to(sx, sy)
        return 'ok' if r in ('arrived', 'done') else 'lost'

    def _jitter_find(self, name, sweeps=4):
        """Re-acquire a FLICKERING detection by wagging slowly left/right — the
        ball pops in and out near the frame edges, so a small slow sweep catches
        a coordinate a stationary look misses. Returns the live det, or None."""
        det = self._live_det(name)
        if det is not None or not ros_node:
            return det
        w = SCAN_TURN_SPEED * 0.5                      # slow (~9 deg/s)
        for i in range(sweeps):
            direction = 1 if i % 2 == 0 else -1
            amp = 0.25 if i == 0 else 0.5              # half-step first, then full alternating
            start = _get('odom_yaw'); turned = 0.0
            while turned < amp:
                self._check()
                ros_node.send_twist(0.0, direction * w); time.sleep(0.08)
                cur = _get('odom_yaw'); turned += abs((cur - start + math.pi) % (2*math.pi) - math.pi); start = cur
                det = self._live_det(name)
                if det is not None:
                    ros_node.send_twist(0.0, 0.0); time.sleep(0.2)
                    return self._live_det(name) or det
        ros_node.send_twist(0.0, 0.0)
        return None

    def _scan_points(self, maxr=4.0):
        """Lidar returns as (x_fwd, y_left) points in the robot frame (base_link)."""
        scan = _get('scan')
        if scan is None: return []
        pts = []; ang = scan.angle_min
        for r in scan.ranges:
            if math.isfinite(r) and scan.range_min <= r <= min(scan.range_max, maxr):
                pts.append((-r * math.sin(ang), r * math.cos(ang)))
            ang += scan.angle_increment
        return pts

    def explore_open(self, max_time=900, reach=2.0):
        """'Open explore': repeatedly head to the most OPEN spot nearby -- the point
        on a ~reach-m ring around the robot with the FEWEST lidar returns near it --
        driving there obstacle-aware (move mode). Never pushes. Until Stop/max_time."""
        if not ros_node: return 'no-robot'
        t0 = time.time(); RING = 24; NEAR = 1.3
        while time.time() - t0 < max_time:
            self._check()
            pts = self._scan_points()
            best = None; bestcnt = 10 ** 9
            for k in range(RING):
                a = -math.pi + 2 * math.pi * k / RING
                cx, cy = reach * math.cos(a), reach * math.sin(a)
                if any(abs(math.atan2(py, px) - a) < 0.25 and math.hypot(px, py) < reach * 0.7
                       for px, py in pts):
                    continue                                            # that way is blocked
                cnt = sum(1 for px, py in pts if (px - cx) ** 2 + (py - cy) ** 2 < NEAR * NEAR)
                if cnt < bestcnt:
                    bestcnt = cnt; best = (cx, cy)
            if best is None:
                _set(status='explore: boxed in, waiting'); time.sleep(0.6); continue
            ox, oy, oyaw = _get('odom_x'), _get('odom_y'), _get('odom_yaw')
            gx = ox + best[0] * math.cos(oyaw) - best[1] * math.sin(oyaw)
            gy = oy + best[0] * math.sin(oyaw) + best[1] * math.cos(oyaw)
            _set(goal_odom=(gx, gy), goal_mode='move', nav_active=True, direct_goal=True,
                 destination='', match_info='explore', path=[], goal_local=None,
                 status='exploring -> open space (%d pts near)' % bestcnt)
            tg = time.time()
            while time.time() - tg < 8:
                self._check()
                if math.hypot(_get('odom_x') - gx, _get('odom_y') - gy) < 0.5:
                    break
                time.sleep(0.2)
        _set(nav_active=False, goal_odom=None); ros_node.stop()
        return 'done'

    def _lidar_ball(self, exp_bearing, exp_range, win=0.45, margin=0.4):
        """Locate the close ball in the LIDAR when the camera loses it: the ball
        is a convex blob sticking out closer than the floor/wall behind it.
        Search a bearing window around exp_bearing for the closest valid return
        near/under exp_range. Returns (bearing, range) robot-frame, or None.
        Only meaningful up close, where a ~40cm ball rises into the scan plane."""
        scan = _get('scan')
        if scan is None: return None
        ranges = scan.ranges; amin = scan.angle_min; ainc = scan.angle_increment
        n = len(ranges)
        if n == 0 or ainc == 0: return None
        i0 = max(0, int((exp_bearing - win - amin) / ainc))
        i1 = min(n - 1, int((exp_bearing + win - amin) / ainc))
        best = None
        for i in range(i0, i1 + 1):
            r = ranges[i]
            if not (math.isfinite(r) and 0.12 < r < exp_range + margin):
                continue
            if best is None or r < best[0]:
                best = (r, amin + i * ainc)
        if best is None: return None
        return (best[1], best[0])                       # (bearing, range)

    def push_to_wall(self, name='ball', max_time=70):
        """Push `name` to the wall behind it. GOAL = wall point from LIDAR along
        the ball's bearing (stable); STEERING = keep the live ball centred
        (camera). Done = stalled near the wall-goal. If the ball escapes
        sideways, drive FORWARD to a standoff behind it and resume."""
        if not ros_node: return 'no-robot'
        t0 = time.time()
        self._center_ball(name)
        det = self._jitter_find(name)            # wag slowly L/R to catch a flickering detection
        if det is None: return 'lost'
        if det.get('dist', 9) < 0.6:             # ball already at our feet — push straight to the wall
            return self.push_through(0.8)
        goal = self._wall_goal(det)
        _set(goal_mode='push', goal_odom=goal, status='pushing → wall (lidar goal)')
        _set(push_from=None)
        last_bearing = math.atan2(det['y_loc'], det['x_loc']); last_range = det.get('dist', 1.0)
        last_prog = time.time(); prog_x, prog_y = _get('odom_x'), _get('odom_y'); lost = 0
        while time.time() - t0 < max_time:
            self._check()
            ox, oy = _get('odom_x'), _get('odom_y')
            dgoal = math.hypot(goal[0] - ox, goal[1] - oy)
            det = self._live_det(name)
            if det is not None:
                lost = 0
                bearing = math.atan2(det['y_loc'], det['x_loc'])
                last_bearing = bearing; last_range = det.get('dist', last_range)
                if abs(bearing) > PUSH_WALL_ESCAPE and dgoal > PUSH_GOAL_TOL + 0.2:
                    ros_node.send_twist(0.0, 0.0)         # ball escaped sideways
                    if self._reposition_behind(name, goal) != 'ok': return 'lost'
                    last_prog = time.time(); prog_x, prog_y = _get('odom_x'), _get('odom_y'); continue
                v = PUSH_WALL_V * max(0.3, 1.0 - abs(bearing))
                w = max(-0.8, min(0.8, PUSH_WALL_STEER * bearing))   # +: steer TOWARD the ball
                ros_node.send_twist(v, w)
            else:
                # camera lost the ball — up close, fall back to the LIDAR blob (fusion)
                lb = self._lidar_ball(last_bearing, last_range) if last_range < 1.1 else None
                if lb is not None:
                    lost = 0; bearing = lb[0]; last_bearing = bearing; last_range = lb[1]
                    v = PUSH_WALL_V * max(0.3, 1.0 - abs(bearing))
                    w = max(-0.8, min(0.8, PUSH_WALL_STEER * bearing))
                    ros_node.send_twist(v, w)
                    _set(status='pushing → wall (lidar-tracked ball)')
                else:
                    lost += 1
                    ros_node.send_twist(PUSH_WALL_V, 0.0)     # keep pushing straight
                    if lost > 25 and dgoal > 0.5:
                        ros_node.send_twist(0.0, 0.0)
                        if self._reposition_behind(name, goal) != 'ok': return 'lost'
                        last_prog = time.time(); prog_x, prog_y = _get('odom_x'), _get('odom_y'); lost = 0; continue
            if dgoal < 0.30:
                ros_node.send_twist(0.0, 0.0)
                return self.push_through(0.4)             # at the wall: finish straight until stall
            # progress from the last MARKER (see _push_segment): a per-tick
            # threshold false-stalls whenever the ball is off-centre and the
            # shove drops to PUSH_WALL_V*0.3 (<4 mm per tick).
            cx, cy = _get('odom_x'), _get('odom_y')
            if math.hypot(cx - prog_x, cy - prog_y) > PROG_M:
                last_prog = time.time(); prog_x, prog_y = cx, cy
            elif time.time() - last_prog > STALL_TIMEOUT:
                ros_node.send_twist(0.0, 0.0)
                return 'pinned' if dgoal < 0.55 else 'stuck'
            time.sleep(0.08)
        ros_node.send_twist(0.0, 0.0)
        return 'timeout'

    # ── Push the ball to a HUMAN-SET goal ────────────────────────────────────
    # find ball → mark it → drive to the point BEHIND it on the goal↔ball line →
    # square up → shove → repeat. Each round re-measures the ball, so a shove that
    # sends the ball slightly off-line is corrected by the next one instead of
    # compounding — that is what makes it converge on the goal.

    def _ball_odom(self, det):
        """Robot-frame detection -> odom point."""
        ox, oy, oyaw = _get('odom_x'), _get('odom_y'), _get('odom_yaw')
        xl, yl = det['x_loc'], det['y_loc']
        return (ox + xl * math.cos(oyaw) - yl * math.sin(oyaw),
                oy + xl * math.sin(oyaw) + yl * math.cos(oyaw))

    def _locate_ball(self, name, hint=None, sweep=True):
        """Best-effort ball fix in odom, trying progressively harder: live
        detection → TURN TO `hint` (the odom point we last saw it at — cheap, and
        after a reposition the robot is often simply facing away) → slow waggle
        (the ball flickers in and out at the frame edge) → step-and-stare sweep
        (it really did leave the field of view; ~30s, so it is the last resort).
        Returns (x, y) or None."""
        det = self._live_det(name)
        if det is None and hint is not None:
            self._square_up(hint, max_time=5.0)
            det = self._live_det(name)
        if det is None:
            det = self._jitter_find(name)
        if det is not None:
            return self._ball_odom(det)
        if not sweep:
            return None
        hit = self.scan_for(name, 360.0)          # rotates; returns an odom {x, y}
        return (hit['x'], hit['y']) if hit else None

    def _square_up(self, target, tol=PUSH_GOAL_ALIGN, max_time=6.0):
        """Rotate in place until `target` is straight ahead, so the shove starts
        square-on instead of glancing off the side of the ball."""
        if not ros_node: return
        t0 = time.time()
        while time.time() - t0 < max_time:
            self._check()
            ox, oy, oyaw = _get('odom_x'), _get('odom_y'), _get('odom_yaw')
            err = math.atan2(target[1] - oy, target[0] - ox) - oyaw
            err = (err + math.pi) % (2 * math.pi) - math.pi
            if abs(err) < tol:
                break
            ros_node.send_twist(0.0, max(-TURN_SPEED, min(TURN_SPEED, 1.6 * err)))
            time.sleep(0.06)
        ros_node.send_twist(0.0, 0.0)
        time.sleep(0.3)                            # let the base settle first

    def _dock_on_line(self, ball, goal, stop_dist=PUSH_DOCK_STOP, max_time=20.0):
        """Final approach: creep forward ALONG the push line, servoing the robot's
        lateral offset out, and stop just short of the ball.

        This is the step that makes the shove accurate. go_to only arrives within
        GOAL_TOL (0.25 m), and once the robot is touching the ball a lateral
        offset can no longer be corrected — the ball simply leaves at an angle.
        So fix it here, while there is still room: steer toward a carrot that sits
        ON the line 0.5 m ahead, which drives the offset to zero as we close in."""
        if not ros_node: return
        t0 = time.time()
        while time.time() - t0 < max_time:
            self._check()
            ox, oy, oyaw = _get('odom_x'), _get('odom_y'), _get('odom_yaw')
            (ux, uy), _ = push_line(ball, goal)
            d_ball = math.hypot(ball[0] - ox, ball[1] - oy)
            if d_ball <= stop_dist:
                break
            e = line_signed_offset((ox, oy), ball, goal)   # robot's offset: + = left
            # carrot = 0.5 m ahead along the line, pulled back onto it from here
            cx = ox + ux * PUSH_DOCK_CARROT + uy * e
            cy = oy + uy * PUSH_DOCK_CARROT - ux * e
            err = _wrap_angle(math.atan2(cy - oy, cx - ox) - oyaw)
            w = max(-TURN_SPEED, min(TURN_SPEED, 2.0 * err))
            v = max(0.10, min(PUSH_DOCK_V, 0.35 * d_ball)) * max(0.0, math.cos(err))
            ros_node.send_twist(v, w)
            _set(status='lining up on the ball (%.2fm off the line)' % abs(e))
            time.sleep(0.06)
        ros_node.send_twist(0.0, 0.0)
        time.sleep(0.2)

    def _push_segment(self, ball0, goal, name, max_time=PUSH_SEG_TIME):
        """One shove: drive forward keeping the ball centred (contact push) until
        the ball reaches `goal`. `ball0` anchors the push line, so a ball that
        veers sideways is detected instead of dragging the line along with it.
        Returns 'at goal' / 'stalled' / 'blocked' / 'retry' (re-line-up)."""
        if not ros_node: return 'blocked'
        t0 = time.time(); last_prog = t0
        prog_x, prog_y = _get('odom_x'), _get('odom_y')
        lost = 0; last_bearing, last_range = 0.0, 1.0
        while time.time() - t0 < max_time:
            self._check()
            kinds = [h['type'] for h in _get('hazards')]
            if 'cliff' in kinds or 'wheel_drop' in kinds:
                ros_node.send_twist(0.0, 0.0)
                return 'blocked'
            ox, oy, oyaw = _get('odom_x'), _get('odom_y'), _get('odom_yaw')
            det = self._live_det(name)
            if det is not None:
                lost = 0
                ball = self._ball_odom(det)
                lx, ly = _odom_to_local(ball, ox, oy, oyaw)
                bearing = math.atan2(ly, lx)
                last_bearing, last_range = bearing, det.get('dist', 1.0)
                dbg = math.hypot(ball[0] - goal[0], ball[1] - goal[1])
                _set(ball_mark=ball)
                if dbg <= PUSH_GOAL_DONE:
                    ros_node.send_twist(0.0, 0.0)
                    _set(status='ball at goal ✓ (%.2fm)' % dbg)
                    return 'at goal'
                if line_offset(ball, ball0, goal) > PUSH_LINE_TOL:
                    ros_node.send_twist(0.0, 0.0)
                    _set(status='ball veered off the line — re-lining-up')
                    return 'retry'
                if math.hypot(ball[0] - ox, ball[1] - oy) < ROBOT_R + 0.06:
                    ros_node.send_twist(0.0, 0.0)          # the ball is under us
                    _set(status='ball under the robot — re-lining-up')
                    return 'retry'
                # A ball rolls along the robot→ball line, so "keep the ball
                # centred" is NOT enough: a robot that is 4 cm off the push line
                # keeps the ball centred while shoving it 4 cm off the line every
                # 30 cm, and the drift locks in. Two corrections, both derived
                # from that geometry:
                #   (a) aim at a point offset from the ball to the SAME side the
                #       ball is off the line — the robot has to get to the far
                #       side of the ball to push it back (aiming at the near side
                #       pushes it further out, which is the runaway this replaces);
                #   (b) a little yaw toward the goal so the robot itself returns
                #       to the line instead of trailing behind at an angle.
                # Speed still keys off the REAL ball bearing, so the robot never
                # turns faster than it can keep contact.
                (lux, luy), _ = push_line(ball0, goal)
                off = max(-0.25, min(0.25, PUSH_GOAL_LAT_K *
                                     line_signed_offset(ball, ball0, goal)))
                virt = (ball[0] - luy * off, ball[1] + lux * off)
                vlx, vly = _odom_to_local(virt, ox, oy, oyaw)
                aim = math.atan2(vly, vlx)
                head = _wrap_angle(math.atan2(goal[1] - oy, goal[0] - ox) - oyaw)
                v = PUSH_GOAL_V * max(0.3, 1.0 - abs(bearing))
                w = max(-0.8, min(0.8, PUSH_GOAL_STEER * aim + PUSH_GOAL_LINE_K * head))
                ros_node.send_twist(v, w)
                _set(status='pushing ball → goal (%.2fm to go)' % dbg)
            else:
                # camera lost it — up close the ball still shows in the LIDAR
                lb = self._lidar_ball(last_bearing, last_range) if last_range < 1.1 else None
                if lb is not None:
                    lost = 0
                    last_bearing, last_range = lb[0], lb[1]
                    head = _wrap_angle(math.atan2(goal[1] - oy, goal[0] - ox) - oyaw)
                    v = PUSH_GOAL_V * max(0.3, 1.0 - abs(lb[0]))
                    w = max(-0.8, min(0.8, PUSH_GOAL_STEER * lb[0] + PUSH_GOAL_LINE_K * head))
                    ros_node.send_twist(v, w)
                    _set(status='pushing ball → goal (lidar-tracked)')
                else:
                    lost += 1
                    ros_node.send_twist(PUSH_GOAL_V, 0.0)  # keep shoving straight
                    if lost > 25:                          # ~2s blind: re-find it
                        ros_node.send_twist(0.0, 0.0)
                        return 'retry'
            # Progress is measured from the last progress MARKER, not the previous
            # tick: with the ball off-centre the shove legitimately crawls at
            # PUSH_GOAL_V*0.3, i.e. <4 mm per 80 ms tick — a per-tick threshold
            # reads that as a stall and quits on a ball that is still moving.
            cx, cy = _get('odom_x'), _get('odom_y')
            if math.hypot(cx - prog_x, cy - prog_y) > PROG_M:
                last_prog = time.time(); prog_x, prog_y = cx, cy
            elif time.time() - last_prog > STALL_TIMEOUT:
                ros_node.send_twist(0.0, 0.0)              # ball pinned on something
                _set(status='⚠ ball jammed — stopped')
                return 'stalled'
            time.sleep(0.08)
        ros_node.send_twist(0.0, 0.0)
        return 'retry'

    def push_to_goal(self, gx=None, gy=None, name='ball', max_time=240.0, max_rounds=14):
        """Push `name` to a goal a HUMAN set — the (gx, gy) arguments, or the
        point marked in the UI (state 'ball_goal') when called with none.

        Each round: find + mark the ball → if the robot is not behind it on the
        goal↔ball line, let the planner drive it there → dock the last metre
        ALONG the line → shove → re-measure → repeat. Re-aiming from a fresh fix
        every round is what makes it converge: a crooked shove is corrected by the
        next one instead of compounding.
        Returns 'at goal' / 'stalled' / 'lost' / 'blocked' / 'no-goal' / 'timeout'."""
        if not ros_node: return 'no-robot'
        if gx is not None and gy is not None:
            goal = (float(gx), float(gy))
        else:
            g = _get('ball_goal')
            goal = (float(g[0]), float(g[1])) if g else None
        if goal is None:
            _set(status='no ball goal set — tick “Ball goal” and click the BEV')
            return 'no-goal'
        _set(ball_goal=goal, status='pushing ball → goal (%.2f, %.2f)' % goal)
        t0 = time.time(); rounds = 0; repositioned = False; blocks = 0
        last_ball = None                # where we saw it last: aim here first
        while time.time() - t0 < max_time and rounds < max_rounds:
            self._check(); rounds += 1
            ball = self._locate_ball(name, hint=last_ball)
            if ball is None:
                ros_node.send_twist(0.0, 0.0)
                _set(status='lost the ball — giving up')
                return 'lost'
            _set(ball_mark=ball)
            last_ball = ball
            dbg = math.hypot(ball[0] - goal[0], ball[1] - goal[1])
            if dbg <= PUSH_GOAL_DONE:
                ros_node.send_twist(0.0, 0.0)
                _set(status='ball at goal ✓ (%.2fm)' % dbg)
                return 'at goal'
            stance = push_stance(ball, goal)
            _set(push_from=stance)
            along, lat, _ = stance_error((_get('odom_x'), _get('odom_y')), ball, goal)
            if along > -PUSH_ALIGN_BACK:
                # Abreast of / ahead of / on top of the ball: NEVER shove from
                # here (it would go the wrong way). Back off well behind it.
                _set(status='not behind the ball — backing off to line up')
                r = self.go_to(*push_stance(ball, goal, PUSH_APPROACH))
                repositioned = False; blocks += 1
                if r not in ('arrived', 'done') and blocks >= 4:
                    _set(status='cannot get behind the ball: %s' % r)
                    return 'blocked'
                continue
            # The dock can only straighten out from a band of distances behind the
            # ball; outside it (or off the line) let the planner reposition first.
            in_band = PUSH_DOCK_ROOM <= -along <= PUSH_APPROACH + GOAL_TOL
            if (lat > PUSH_ALIGN_LAT or not in_band) and not repositioned:
                # move mode, so A* routes around furniture instead of ploughing
                # through it — then dock the last metre.
                _set(status='driving behind the ball (%.2fm to goal)' % dbg)
                r = self.go_to(*push_stance(ball, goal, PUSH_APPROACH))
                blocks = blocks + 1 if r not in ('arrived', 'done') else 0
                if blocks >= 3:
                    _set(status='cannot get behind the ball: %s' % r)
                    return 'blocked'
                repositioned = True
                continue
            self._dock_on_line(ball, goal)           # creep onto the line, then shove
            _set(goal_mode='push')                   # a BUMP here is the ball, not a wall
            try:
                r = self._push_segment(ball, goal, name)
            finally:
                _set(goal_mode='move')
            repositioned = False                     # a shove happened: allow re-lining-up
            if r in ('at goal', 'stalled', 'blocked'):
                return r
        ros_node.send_twist(0.0, 0.0)
        _set(status='push-to-goal timed out')
        return 'timeout'

    def follow(self, name=None, standoff=None, fast=None, max_time=900):
        """Follow the CLOSEST `name`, AVOIDING OBSTACLES on the way — a person to
        help them carry things, a dog, a ball, anything nameable.

        What to follow, how close to stand, whether to acquire it with a
        step-and-stare sweep first, and whether to use the lidar predictor are
        resolved from the spoken phrase by _follow_params() and passed in through
        state — which is why ONE programs/follow.toy covers every target instead
        of four near-identical programs. Explicit arguments (a hand-written .toy,
        or a test) take precedence over state; `fast` asks for the predictor and
        is the third DSL argument, FOLLOW(name, standoff, fast).

        Unlike push mode this uses the move-mode A* planner — it routes around
        furniture and stops short of things, never pushing. It continuously sets
        a goal a `standoff` short of the target and lets the nav loop drive
        there; holds when already within standoff (never crowds it); if the
        target drops out of view it briefly holds, then TURNS toward where it was
        last seen and sweeps until re-acquired. Runs until Stop or max_time.

        Returns 'done' / 'not-found' / 'no-robot'."""
        if not ros_node: return 'no-robot'
        name     = name or _get('follow_target') or 'person'
        standoff = standoff or _get('follow_standoff') or FOLLOW_STANDOFF
        if fast is None: fast = bool(_get('follow_fast'))
        close = _is_close_target(name)
        # The predictor's lidar model (_lidar_ball) hunts a close convex blob
        # sticking out nearer than the wall behind it — a sensor model for a small
        # object on the floor. It is not applied to a person or a dog even if a
        # hand-written .toy asks for it; the camera holds those in frame anyway.
        fast = bool(fast) and close
        if bool(_get('follow_scan_first')) or fast:
            # Small, low target: FOLLOW's own re-acquire is a CONTINUOUS spin,
            # which the OAK detects poorly through (motion blur at low fps), so it
            # sails past a ball. scan_for steps and stares instead. Give up rather
            # than orbit an empty room for max_time — that is what follow_ball.toy
            # used to encode as `PRINT "No ball found in the room."`.
            _set(status=f'looking for the {name} before following')
            if self.scan_for(name, 360.0) is None:
                ros_node.stop()
                return 'not-found'
        if fast:
            return self.follow_ball_fast(name, standoff, max_time)
        t0 = time.time(); lost = 0; last_side = 1.0        # +1 = last seen on the left
        _set(status=f'following a {name} (obstacle-aware)')
        while time.time() - t0 < max_time:
            self._check()
            det = self._live_det(name)
            if det is not None:
                lost = 0
                ox, oy, oyaw = _get('odom_x'), _get('odom_y'), _get('odom_yaw')
                xl, yl = det['x_loc'], det['y_loc']
                last_side = 1.0 if yl >= 0 else -1.0                     # remember which side they were on
                px = ox + xl*math.cos(oyaw) - yl*math.sin(oyaw)          # person, odom
                py = oy + xl*math.sin(oyaw) + yl*math.cos(oyaw)
                d = math.hypot(px - ox, py - oy)
                # Hysteresis MUST exceed GOAL_TOL: the nav loop reports a goal
                # closer than GOAL_TOL as "arrived" without moving, so a smaller
                # band leaves a STATIC target (sofa/chair/bottle) re-issuing an
                # already-arrived goal forever while the robot never drives.
                if d > standoff + GOAL_TOL + 0.05:
                    gx = px - (px - ox)/d * standoff                     # goal: standoff short of them
                    gy = py - (py - oy)/d * standoff
                    _set(goal_odom=(gx, gy), goal_mode='move', nav_active=True,
                         direct_goal=True, destination='', match_info='follow',
                         path=[], goal_local=None,
                         status=f'following -> {name} (avoiding obstacles)')
                else:
                    _set(nav_active=False, goal_odom=None); ros_node.stop()   # within standoff, hold
            else:
                lost += 1
                if lost <= 6:                                            # brief grace (~1.5s): occlusion / frame edge
                    pass                                                 #   nav keeps driving to last-seen spot
                else:                                                    # turn toward last-seen side, sweep until re-acquired
                    _set(nav_active=False, goal_odom=None, path=[], goal_local=None,
                         status=f'lost the {name} — turning to find them')
                    ros_node.send_twist(0.0, SCAN_TURN_SPEED * last_side)
            time.sleep(0.25)
        _set(nav_active=False, goal_odom=None); ros_node.stop()
        return 'done'

    def follow_ball_fast(self, name="ball", standoff=0.7, max_time=900):
        """AGGRESSIVE ball follow: keep a velocity belief of the ball and bridge
        the camera's gaps with the LIDAR. The camera IDs/locks it (reliable,
        ~2fps); the lidar tracks its blob between camera frames (~8fps), gated to
        the predicted spot; when neither confirms, EXTRAPOLATE along its velocity;
        after ~1.5s unconfirmed, decay to a scan. Drives obstacle-aware (move
        mode) to a standoff short of the belief -- never rams it (not push).

        Not a DSL primitive of its own any more: `follow` delegates here when
        _follow_params() resolves `fast` for a small target, so a .toy asks for it
        with FOLLOW(name, standoff, 1) rather than a separate verb. That keeps the
        predictor's validity condition (small, low, floor-level object) checked in
        one place instead of at every call site."""
        if not ros_node: return 'no-robot'
        t0 = time.time()
        bx = by = None; vx = vy = 0.0; t_conf = 0.0; hist = []
        while time.time() - t0 < max_time:
            self._check()
            now = time.time()
            ox, oy, oyaw = _get('odom_x'), _get('odom_y'), _get('odom_yaw')
            det = self._live_det(name)
            conf = False
            if det is not None:                                     # camera lock
                bx = ox + det['x_loc']*math.cos(oyaw) - det['y_loc']*math.sin(oyaw)
                by = oy + det['x_loc']*math.sin(oyaw) + det['y_loc']*math.cos(oyaw)
                conf = True
            elif bx is not None:                                    # camera lost -> lidar near prediction
                dt = now - t_conf
                pbx, pby = bx + vx*dt, by + vy*dt
                bearing = (math.atan2(pby-oy, pbx-ox) - oyaw + math.pi) % (2*math.pi) - math.pi
                rng = math.hypot(pbx-ox, pby-oy)
                lb = self._lidar_ball(bearing, rng, win=0.30, margin=0.5) if rng < 2.2 else None
                if lb is not None:
                    bx = ox + lb[1]*math.cos(oyaw + lb[0])
                    by = oy + lb[1]*math.sin(oyaw + lb[0])
                    conf = True
            if conf:
                hist.append((now, bx, by)); hist = hist[-6:]
                if len(hist) >= 2 and (hist[-1][0] - hist[0][0]) > 0.1:
                    dtv = hist[-1][0] - hist[0][0]
                    vx = (hist[-1][1] - hist[0][1]) / dtv
                    vy = (hist[-1][2] - hist[0][2]) / dtv
                t_conf = now
            if bx is None or (now - t_conf > 1.5):                  # decay rail -> re-scan
                _set(nav_active=False); ros_node.stop()
                found = self.scan_for(name, 220)
                if found is not None:
                    bx, by = found['x'], found['y']; vx = vy = 0.0; hist = []; t_conf = time.time()
                continue
            dt = now - t_conf                                       # drive to predicted-now, standoff short
            pnx, pny = bx + vx*dt, by + vy*dt
            d = math.hypot(pnx-ox, pny-oy)
            if d > standoff + 0.10:
                gx = pnx - (pnx-ox)/d*standoff
                gy = pny - (pny-oy)/d*standoff
                _set(goal_odom=(gx, gy), goal_mode='move', nav_active=True, direct_goal=True,
                     destination='', match_info='follow-fast', path=[], goal_local=None,
                     status='following ball (predicted + lidar)')
            else:
                _set(nav_active=False, goal_odom=None); ros_node.stop()
            time.sleep(0.12)
        _set(nav_active=False, goal_odom=None); ros_node.stop()
        return 'done'

    def scan_for(self, name, max_deg=360.0):
        """STEP-AND-STARE sweep: rotate in ~25 deg steps, STOPPING at each to
        look at several still frames. The OAK detects poorly while rotating
        (motion blur at low fps), so a continuous spin sails past the ball; a
        stop-and-look sweep reliably finds it. Centre + return on first hit."""
        if not ros_node: return None
        STEP = math.radians(25)
        w = (-1.0 if max_deg >= 0 else 1.0) * SCAN_TURN_SPEED
        target = math.radians(abs(max_deg))
        swept = 0.0; t0 = time.time()
        while swept < target and time.time() - t0 < 130:
            for _ in range(6):                       # stare: poll still frames (no motion blur)
                self._check()
                hit = self._scan_hit(name)
                if hit is not None:
                    centered = self._center_ball(name)
                    return centered if centered is not None else hit
                time.sleep(0.12)
            prev = _get("odom_yaw"); stepped = 0.0; ts = time.time()   # step ~25 deg
            while stepped < STEP and time.time() - ts < 5:
                self._check()
                ros_node.send_twist(0.0, w); time.sleep(0.05)
                cur = _get("odom_yaw")
                stepped += abs((cur - prev + math.pi) % (2*math.pi) - math.pi); prev = cur
            ros_node.send_twist(0.0, 0.0)
            swept += stepped
        return None

    def wait(self, secs):
        """Dwell in place for `secs`s so the camera/NN settles for a clean look."""
        t0 = time.time()
        while time.time() - t0 < secs:
            self._check()
            time.sleep(0.05)
        return 'done'


_PLAN_PHRASE = {
    'SCAN_FOR':     'locate the target (scan & centre)',
    'FIND':         'locate the target',
    'WAIT':         'pause & settle',
    'PUSH_AWAY':    'approach & push the ball',
    'PUSH_TO':      'approach the target',
    'PUSH_THROUGH': 'push it to the wall',
    'PUSH_TO_WALL': 'push it to the wall',
    'PUSH_TO_GOAL': 'line up behind the ball & push it to your goal',
    'FOLLOW':       'follow the target',
    'GO_TO':        'drive to the spot',
    'POINT':        'turn to face it',
}

def _plan_steps(source):
    """Human-readable plan for a program, for the UI 'reasoning' panel. Uses a
    `# plan:` header if present (`phrase@VERB | phrase@VERB`), else derives one
    from the action verbs used, in first-appearance order. Each step carries the
    ToyScript verb that drives it so the UI can light up the live step."""
    for line in source.splitlines():
        if line.lower().startswith('# plan:'):
            steps = []
            for item in line.split(':', 1)[1].split('|'):
                item = item.strip()
                if not item:
                    continue
                phrase, _, verb = item.partition('@')
                steps.append({'phrase': phrase.strip(), 'verb': verb.strip().upper()})
            if steps:
                return steps
    steps, seen = [], set()
    for line in source.splitlines():
        if line.strip().startswith('#'):
            continue
        for mt in re.finditer(r'\b([A-Z_]+)\s*\(', line):
            v = mt.group(1)
            if v in _PLAN_PHRASE and v not in seen:
                seen.add(v); steps.append({'phrase': _PLAN_PHRASE[v], 'verb': v})
    return steps


def load_programs():
    progs = {}
    for f in sorted(glob.glob(os.path.join(PROGRAMS_DIR, '*.toy'))):
        with open(f) as fh:
            src = fh.read()
        m = {'name': os.path.basename(f)[:-4], 'description': '', 'triggers': [], 'source': src}
        for line in src.splitlines():
            for key in ('name', 'description', 'triggers'):
                if line.lower().startswith(f'# {key}:'):
                    val = line.split(':', 1)[1].strip()
                    m[key] = [t.strip() for t in val.split(',')] if key == 'triggers' else val
        m['plan'] = _plan_steps(src)
        progs[m['name']] = m
    return progs


def match_program(text, progs):
    """Match a command to a program. A trigger phrase contained verbatim in
    the text wins outright (longest first) — token overlap can't shadow a
    program's own trigger (e.g. "push to wall" used to lose to
    push_ball_to_wall's larger word pool). Falls back to token-overlap
    scoring of description+triggers.
    (Deliberately simple/dependency-free; swap in an embedding model here.)"""
    norm = ' '.join(re.findall(r'[a-z]+', text.lower()))
    phrase_hit, phrase_len = None, 0
    for m in progs.values():
        for trig in m['triggers']:
            t = ' '.join(re.findall(r'[a-z]+', trig.lower()))
            if t and t in norm and len(t) > phrase_len:
                phrase_hit, phrase_len = m, len(t)
    if phrase_hit:
        return phrase_hit, 1.0
    q = set(re.findall(r'[a-z]+', text.lower()))
    best, bs = None, 0.0
    for m in progs.values():
        corpus = (m['description'] + ' ' + ' '.join(m['triggers'])).lower()
        words = set(re.findall(r'[a-z]+', corpus))
        if not words: continue
        score = len(q & words) / math.sqrt(len(q) + 1)
        if score > bs: bs, best = score, m
    return best, bs


_run_lock = threading.Lock()

def _run_program(meta):
    robot = NavRobot()
    lines = []
    def log(msg):
        lines.append(str(msg)); _set(run_log=list(lines)[-50:])
        print(f"[toyscript:{meta['name']}] {msg}")
    def _fmt_res(r):
        if r is None: return 'NONE'
        if isinstance(r, dict): return '{x=%.2f, y=%.2f}' % (r.get('x', 0.0), r.get('y', 0.0))
        return str(r)
    def on_call(label, result, done):
        hist = list(_get('cmd_history') or [])
        if not done:
            hist.append({'cmd': label, 'result': '', 'running': True})
        elif hist and hist[-1].get('running'):
            hist[-1] = {'cmd': label, 'result': _fmt_res(result), 'running': False}
        else:
            hist.append({'cmd': label, 'result': _fmt_res(result), 'running': False})
        _set(cmd_history=hist[-5:])
    _set(run_active=True, run_program=meta['name'], run_log=[], run_error=None, cmd_history=[])
    globals()['_active_robot'] = robot
    try:
        toyscript.Interpreter(robot, log=log, on_call=on_call).run(meta['source'])
    except toyscript.StopProgram:
        log('[stopped]')
    except Exception as e:
        _set(run_error=f'{type(e).__name__}: {e}'); log(f'[error] {e}')
    finally:
        if ros_node: ros_node.stop()
        # goal_mode is a shared flag the push skills flip; a skill that exits
        # early must not leave the next drive in push mode (different hazard
        # handling), so reset it here along with the other per-run state.
        _set(run_active=False, nav_active=False, target_color=None,
             follow_target=None, follow_standoff=None, follow_scan_first=False,
             follow_fast=False, goal_mode='move', push_from=None, ball_mark=None)


# ── Flask routes ──────────────────────────────────────────────────────────────
@app.route('/')
def index():
    return render_template("index.html")

@app.route('/state')
def get_state():
    with _lock:
        _hnow = time.time()
        resp = jsonify({
            'cam_ok':       (_hnow - _state['img_t'])  < 3.0,
            'lidar_ok':     (_hnow - _state['scan_t']) < 3.0,
            'base_ok':      (_hnow - _state['odom_t']) < 2.0,
            'destination':  _state['destination'],
            'status':       _state['status'],
            'match_info':   _state['match_info'],
            'nav_active':   _state['nav_active'],
            'goal_mode':    _state['goal_mode'],
            'detections':   _state['detections'],
            'use_lidar_dist': _state['use_lidar_dist'],
            'odom_x':       round(_state['odom_x'],   3),
            'odom_y':       round(_state['odom_y'],   3),
            'odom_yaw':     round(math.degrees(_state['odom_yaw']), 1),
            'goal_odom':    _state['goal_odom'],
            'ball_goal':    _state['ball_goal'],
            'battery_voltage': _state['battery_voltage'],
            'battery_percentage': _state['battery_percentage'],
            'docked':       _state['docked'],
            'dock_busy':    _state['dock_busy'],
            'recording':    _state['recording'],
            'record_drive': _state['record_drive'],
            'record_steps': _state['record_steps'],
            'hazards':      _state['hazards'],
            'run_active':   _state['run_active'],
            'run_program':  _state['run_program'],
            'run_log':      _state['run_log'],
            'cmd_history':  _state['cmd_history'],
            'run_error':    _state['run_error'],
            'last_command': _state['last_command'],
            'match_score':  _state['match_score'],
            'plan':         _state['plan'],
            'target_color': _state['target_color'],
            'follow_target': _state['follow_target'],
            # Glass-box: what the phrase was understood to ask for, so you can see
            # why the robot is standing 0.7 m off a ball but 1.0 m off you.
            'follow': {
                'target':     _state['follow_target'],
                'standoff':   _state['follow_standoff'],
                'scan_first': _state['follow_scan_first'],
                'fast':       _state['follow_fast'],
            },
        })
    resp.headers['Cache-Control'] = 'no-store'   # never serve a stale /state from browser cache
    return resp

@app.route('/programs')
def list_programs():
    progs = load_programs()
    return jsonify([{'name': m['name'], 'description': m['description']} for m in progs.values()])

@app.route('/run', methods=['POST'])
def run_program():
    """Run a program by name, or match free text to one. {program:...} or {text:...}."""
    if _get('run_active'):
        return jsonify({'ok': False, 'error': 'a program is already running'}), 409
    if _get('docked'):
        return jsonify({'ok': False, 'error': 'robot is docked — undock first (wheels are disabled)'}), 409
    data = request.get_json(force=True)
    progs = load_programs()
    text = data.get('text', '')
    # Resolve the follow parameters from the phrase ONCE. This route used to call
    # _parse_follow_target twice (once to route, once to set state), which is how
    # the routing and the skill could end up disagreeing.
    fp = _follow_params(text)
    if data.get('program'):
        meta = progs.get(data['program'])
        if not meta: return jsonify({'ok': False, 'error': 'no such program'}), 404
        matched, score = meta, 1.0
    elif fp['target'] and progs.get('follow'):
        # "follow the <COCO object>" — the phrase already named the target, so go
        # straight to the one follow program. The token-overlap matcher gets no
        # say here. Safe now that follow.toy is the ONLY follow program: this
        # special case used to fire ahead of three other programs' own triggers.
        matched, score = progs['follow'], 1.0
    else:
        matched, score = match_program(text, progs)
        if not matched or score <= 0:
            return jsonify({'ok': False, 'error': 'no matching program',
                            'available': list(progs)}), 404
    _set(target_color=_parse_color(text),
         follow_target=fp['target'],
         follow_standoff=fp['standoff'],
         follow_scan_first=fp['scan_first'],
         follow_fast=fp['fast'],
         last_command=(text or matched['name']),
         match_score=round(score, 2),
         plan=matched.get('plan') or [])
    threading.Thread(target=_run_program, args=(matched,), daemon=True).start()
    return jsonify({'ok': True, 'program': matched['name'], 'target_color': _get('target_color'),
                    'follow': fp,
                    'description': matched['description'], 'score': round(score, 3)})

@app.route('/run_stop', methods=['POST'])
def run_stop():
    r = globals().get('_active_robot')
    if r: r.abort()
    _set(nav_active=False)
    if ros_node: ros_node.stop()
    return jsonify({'ok': True})

@app.route('/goto_xy', methods=['POST'])
def goto_xy():
    data = request.get_json(force=True)
    try:
        gx = float(data['x'])
        gy = float(data['y'])
    except (KeyError, ValueError):
        return jsonify({'ok': False, 'error': 'need x and y'}), 400
    mode = 'push' if str(data.get('mode', 'move')).lower() == 'push' else 'move'
    verb = 'pushing →' if mode == 'push' else 'navigating →'
    _set(goal_odom=(gx, gy), goal_mode=mode, direct_goal=True, nav_active=True,
         destination='', match_info=f'direct coordinate ({mode})',
         status=f'{verb} ({gx:.2f}, {gy:.2f})',
         path=[], goal_local=None)
    return jsonify({'ok': True, 'goal': [gx, gy], 'mode': mode})

@app.route('/set_ball_goal', methods=['POST'])
def set_ball_goal():
    """Mark where the BALL should be pushed (odom x, y) — the goal PUSH_TO_GOAL
    drives the ball to. Stores the point and draws it on the BEV WITHOUT moving
    the robot, so you can mark the goal and then run the task. Body {x, y}, or
    {"clear": true} to unmark."""
    data = request.get_json(force=True, silent=True) or {}
    if data.get('clear'):
        _set(ball_goal=None, push_from=None)
        return jsonify({'ok': True, 'ball_goal': None})
    try:
        gx = float(data['x'])
        gy = float(data['y'])
    except (KeyError, TypeError, ValueError):
        return jsonify({'ok': False, 'error': 'need x and y'}), 400
    _set(ball_goal=(gx, gy), push_from=None)
    return jsonify({'ok': True, 'ball_goal': [gx, gy]})

@app.route('/navigate', methods=['POST'])
def navigate():
    data = request.get_json(force=True)
    desc = (data.get('description') or '').strip()
    if desc:
        _set(destination=desc, nav_active=True, status='finding', goal_mode='move',
             goal_odom=None, direct_goal=False, path=[], match_info='')
    return jsonify({'ok': True})

@app.route('/stop', methods=['POST'])
def stop_nav():
    _set(nav_active=False, status='idle', path=[], goal_local=None,
         goal_odom=None, direct_goal=False, goal_mode='move')
    if ros_node: ros_node.stop()
    return jsonify({'ok': True})

def _run_dock_action(kind):
    """Background worker: cancel nav and fire the Dock/Undock action."""
    try:
        if kind == 'dock':
            ros_node.dock()
        else:
            ros_node.undock()
    finally:
        _set(dock_busy=False)

@app.route('/dock', methods=['POST'])
def dock():
    """Send the robot to its charging dock (cancels any active nav)."""
    if not ros_node:
        return jsonify({'ok': False, 'error': 'ros node not ready'}), 503
    if _get('dock_busy'):
        return jsonify({'ok': False, 'error': 'dock action already running'}), 409
    _set(nav_active=False, status='docking', path=[], goal_local=None,
         goal_odom=None, direct_goal=False, dock_busy=True)
    threading.Thread(target=_run_dock_action, args=('dock',), daemon=True).start()
    return jsonify({'ok': True})

@app.route('/undock', methods=['POST'])
def undock():
    """Drive the robot off its charging dock."""
    if not ros_node:
        return jsonify({'ok': False, 'error': 'ros node not ready'}), 503
    if _get('dock_busy'):
        return jsonify({'ok': False, 'error': 'dock action already running'}), 409
    _set(status='undocking', dock_busy=True)
    threading.Thread(target=_run_dock_action, args=('undock',), daemon=True).start()
    return jsonify({'ok': True})

@app.route('/record', methods=['POST'])
def record_toggle():
    """Arm/disarm dataset recording (nav_loop picks up the flag). Body:
    {"on": true|false}; the nav_loop starts/stops a drive accordingly."""
    data = request.get_json(force=True, silent=True) or {}
    on = bool(data.get('on'))
    _set(recording=on)
    return jsonify({'ok': True, 'recording': on})

@app.route('/lidar_dist', methods=['POST'])
def set_lidar_dist():
    """Toggle distance source: min(OAK stereo, lidar) when on; raw OAK when off."""
    data = request.get_json(force=True, silent=True) or {}
    on = bool(data.get('on'))
    _set(use_lidar_dist=on)
    return jsonify({'ok': True, 'use_lidar_dist': on})

@app.route('/camera.jpg')
def camera_jpg():
    frame = _get('frame_rgb')
    dets = _get('detections')
    jpg = render_image_with_detections(frame, dets)
    return Response(jpg, mimetype='image/jpeg',
                    headers={'Cache-Control': 'no-store'})

@app.route('/lidar.png')
def lidar_png():
    with _lock:
        cached = _state['lidar_png']
        scan, path, gl = _state['scan'], _state['path'], _state['goal_local']
        dets = list(_state['detections'])
        ll = _state['look_local']
        gm = _state['goal_mode']
    png = cached or render_lidar_png(scan, path, gl, dets, ll, gm)
    return Response(png, mimetype='image/png',
                    headers={'Cache-Control': 'no-store'})


# ── Entry point ───────────────────────────────────────────────────────────────
# ── Startup chime ─────────────────────────────────────────────────────────────
STARTUP_READY_TIMEOUT = 120     # s to wait for cam+lidar+base before the warn tone
# "Twinkle Twinkle Little Star" (first two phrases): C C G G A A G, F F E E D D C
_NOTE = {'C':523, 'D':587, 'E':659, 'F':698, 'G':784, 'A':880}   # C5 octave
_Q, _H = 260, 520                                                # quarter / half note (ms)
READY_CHIME = [(_NOTE[c], d) for c, d in [
    ('C',_Q),('C',_Q),('G',_Q),('G',_Q),('A',_Q),('A',_Q),('G',_H),
    ('F',_Q),('F',_Q),('E',_Q),('E',_Q),('D',_Q),('D',_Q),('C',_H)]]
WARN_TONE   = [(392,220),(330,220),(262,420)]              # descending = "something's missing"

def _play_notes(node, notes):
    """Beep a [(freq_hz, ms), ...] tune through the Create 3 speaker (/cmd_audio)."""
    if not getattr(node, 'audio_pub', None):
        return
    msg = AudioNoteVector(); msg.append = False
    for f, ms in notes:
        n = AudioNote(); n.frequency = int(f)
        n.max_runtime = _AudioDur(sec=int(ms)//1000, nanosec=(int(ms) % 1000)*1_000_000)
        msg.notes.append(n)
    try:
        node.audio_pub.publish(msg)
    except Exception:
        pass

def _ready_announcer(node):
    """Chime once cam+lidar+base are all live so you know the robot is ready to
    drive; if they're not all up within the timeout, play a descending warn tone
    and set the status to what's missing. (No sound at all ⇒ the base/bridge is
    down — the 'sees but cannot move' case; reboot the Create 3 base.)"""
    t0 = time.time()
    while time.time() - t0 < STARTUP_READY_TIMEOUT:
        now = time.time()
        if (now - _get('img_t') < 3.0) and (now - _get('scan_t') < 3.0) and (now - _get('odom_t') < 2.0):
            time.sleep(1.0)                     # let the streams settle a beat
            _play_notes(node, READY_CHIME)
            _set(status='READY — cam/lidar/base all live')
            print('[TB4 Nav] READY — chimed'); return
        time.sleep(1.0)
    now = time.time()
    miss = [n for n, ok in (('cam', now - _get('img_t') < 3.0),
                            ('lidar', now - _get('scan_t') < 3.0),
                            ('base', now - _get('odom_t') < 2.0)) if not ok]
    _play_notes(node, WARN_TONE)
    _set(status='startup INCOMPLETE — missing: ' + (','.join(miss) or '?'))
    print('[TB4 Nav] startup incomplete — missing:', miss)


def main():
    global ros_node

    print(f'[TB4 Nav] YOLO       : on-camera (OAK-D VPU) → {NN_TOPIC}')
    print(f'[TB4 Nav] Battery    : /battery_state')
    print(f'[TB4 Nav] Web UI     : http://0.0.0.0:{FLASK_PORT}')
    print(f'[TB4 Nav] Refresh    : {IMG_REFRESH_HZ} fps')

    rclpy.init()
    ros_node = NavNode()
    executor = MultiThreadedExecutor()
    executor.add_node(ros_node)

    threading.Thread(target=executor.spin, daemon=True).start()
    threading.Thread(target=nav_loop,      daemon=True).start()
    threading.Thread(target=_ready_announcer, args=(ros_node,), daemon=True).start()

    try:
        app.run(host='0.0.0.0', port=FLASK_PORT,
                debug=False, threaded=True, use_reloader=False)
    finally:
        if ros_node: ros_node.stop()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
