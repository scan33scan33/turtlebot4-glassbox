#!/usr/bin/env python3
"""Offline tests for PUSH_TO_GOAL — "push the ball to a human-set goal".

The Pi has ROS; this box doesn't, so the ROS message packages are stubbed and
the module is imported for real. The tests then drive the SHIPPING code
(NavRobot.push_to_goal / push_stance / stance_error / line_offset and the Flask
routes) against a closed-loop fake base that integrates cmd_vel into a pose,
rolls the ball when the robot shoves it, and completes go_to goals the way the
nav_loop planner does.

Run:  python3 tests/test_push_to_goal.py           (needs numpy, opencv, flask)
"""
import math
import os
import sys
import threading
import time
import types
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)


# ── ROS stubs ─────────────────────────────────────────────────────────────────
# Just enough surface for `import tb4_claude_nav` to succeed: every name the
# module touches at import time or in the code paths under test.
def _stub_modules():
    def mod(name, **attrs):
        m = types.ModuleType(name)
        for k, v in attrs.items():
            setattr(m, k, v)
        sys.modules[name] = m
        return m

    class _Any:
        """Attribute bag that also works as a base class / constructor."""
        def __init__(self, *a, **k):
            pass

        def __getattr__(self, item):
            return _Any()

        def __call__(self, *a, **k):
            return _Any()

    class Node:
        def __init__(self, *a, **k):
            pass

        def create_subscription(self, *a, **k):
            return None

        def create_publisher(self, *a, **k):
            return _Any()

        def create_client(self, *a, **k):
            return _Any()

        def get_logger(self):
            return _Any()

        def get_clock(self):
            return _Any()

    mod('rclpy', init=lambda *a, **k: None, shutdown=lambda *a, **k: None,
        node=mod('rclpy.node', Node=Node),
        action=mod('rclpy.action', ActionClient=_Any),
        executors=mod('rclpy.executors', MultiThreadedExecutor=_Any),
        qos=mod('rclpy.qos', QoSProfile=_Any, ReliabilityPolicy=_Any,
                DurabilityPolicy=_Any, HistoryPolicy=_Any))
    mod('sensor_msgs', msg=mod('sensor_msgs.msg', LaserScan=_Any, Image=_Any,
                               BatteryState=_Any))
    mod('nav_msgs', msg=mod('nav_msgs.msg', Odometry=_Any))
    mod('geometry_msgs', msg=mod('geometry_msgs.msg', TwistStamped=_Any))
    mod('vision_msgs', msg=mod('vision_msgs.msg', Detection3DArray=_Any))
    mod('irobot_create_msgs',
        msg=mod('irobot_create_msgs.msg', DockStatus=_Any, HazardDetectionVector=_Any,
                AudioNoteVector=_Any, AudioNote=_Any),
        action=mod('irobot_create_msgs.action', Dock=_Any, Undock=_Any))
    mod('builtin_interfaces', msg=mod('builtin_interfaces.msg', Duration=_Any))
    mod('std_srvs', srv=mod('std_srvs.srv', Empty=_Any))


_stub_modules()
import tb4_claude_nav as tb4                                    # noqa: E402


def clear_scan(obstacle=None):
    """360° RPLIDAR scan; an optional obstacle is specified in base_link x/y."""
    inc = 2 * math.pi / 360
    ranges = [4.0] * 360
    if obstacle is not None:
        x, y = obstacle
        # base_x=-r*sin(lidar_angle), base_y=r*cos(lidar_angle)
        angle = math.atan2(-x, y)
        i = round((angle + math.pi) / inc) % 360
        ranges[i] = math.hypot(x, y)
    return types.SimpleNamespace(ranges=ranges, angle_min=-math.pi,
                                 angle_increment=inc, range_min=0.12,
                                 range_max=8.0)


# ── Fake base + world ─────────────────────────────────────────────────────────
class FakeBase:
    """NavNode stand-in that also runs the world: integrates the commanded twist
    into an odom pose, rolls the ball on contact, publishes the ball as a camera
    detection when it is in the field of view, and drives/serves go_to goals the
    way nav_loop does."""

    FOV_HALF = 0.58          # rad — OAK-D 416 preview is ~65 deg across
    CONTACT = 0.30           # m — robot centre to ball centre while pushing

    def __init__(self, ball, pose=(0.0, 0.0, 0.0), rate=100.0, scan=None):
        self.ball = [float(ball[0]), float(ball[1])]
        self.rate = rate
        self.scan = scan
        self._v = self._w = 0.0
        self._stop = threading.Event()
        self.twists = []
        self.max_ball_goal_dist = 0.0
        self.ball_track = []
        tb4._set(odom_x=pose[0], odom_y=pose[1], odom_yaw=pose[2],
                 odom_t=time.time(), scan=scan, scan_t=time.time() if scan else 0.0,
                 docked=False, detections=[],
                 hazards=[], nav_active=False, goal_odom=None, goal_mode='move',
                 goal_local=None, look_local=None, path=[], status='idle',
                 ball_mark=None, push_from=None,
                 target_color=None, follow_target=None, run_active=False)
        self.goal0 = None
        self._publish_ball()
        self._th = threading.Thread(target=self._run, daemon=True)

    # ── NavNode surface the skills use ──
    def send_twist(self, linear, angular):
        self._v, self._w = float(linear), float(angular)
        self.twists.append((self._v, self._w))
        tb4._set(cmd_sent=dict(v=self._v, w=self._w, t=time.time()))

    def stop(self):
        self.send_twist(0.0, 0.0)

    def get_logger(self):
        return self

    def info(self, *a, **k):
        pass

    def warning(self, *a, **k):
        pass

    def restart_lidar_motor(self):
        return False

    def restart_camera(self):
        return False

    # ── lifecycle ──
    def __enter__(self):
        tb4.ros_node = self          # the skills drive through this global
        self._th.start()
        return self

    def __exit__(self, *exc):
        self._stop.set()
        self._th.join(timeout=3)
        tb4.ros_node = None
        return False

    # ── world ──
    def _publish_ball(self):
        """Publish the ball as a spatial detection, but only when the camera can
        actually see it (in front, inside the FOV, in range) — otherwise the
        skill must fall back to jitter/scan, exactly as on the robot."""
        x, y, yaw = tb4._get('odom_x'), tb4._get('odom_y'), tb4._get('odom_yaw')
        lx, ly = tb4._odom_to_local(self.ball, x, y, yaw)
        d = math.hypot(lx, ly)
        bearing = math.atan2(ly, lx)
        visible = d < 4.0 and lx > 0.0 and abs(bearing) < self.FOV_HALF
        if not visible:
            tb4._set(detections=[], det_t=time.time())
            return
        tb4._set(detections=[dict(label='sports ball', conf=0.92, x_loc=lx, y_loc=ly,
                                  dist=d, cam_dist=d, lidar_dist=None, p25_dist=d,
                                  x_px=208.0, y_px=250.0, w_px=24.0, h_px=24.0,
                                  color=None)], det_t=time.time())

    def _serve_nav(self):
        """nav_loop stand-in: drive toward goal_odom, report arrival like it does."""
        if not tb4._get('nav_active'):
            return
        g = tb4._get('goal_odom')
        x, y, yaw = tb4._get('odom_x'), tb4._get('odom_y'), tb4._get('odom_yaw')
        if not g:
            self._v = self._w = 0.0
            tb4._set(nav_active=False, status='no goal set')
            return
        lx, ly = tb4._odom_to_local(g, x, y, yaw)
        d = math.hypot(lx, ly)
        if d < tb4.GOAL_TOL:
            self._v = self._w = 0.0
            tb4._set(nav_active=False, goal_mode='move',
                     status='arrived (δ=%.2fm)' % d, path=[])
            return
        bearing = math.atan2(ly, lx)
        self._w = max(-tb4.TURN_SPEED, min(tb4.TURN_SPEED, 2.0 * bearing))
        self._v = 0.6 * max(0.0, math.cos(bearing))   # quicker than real, same logic

    def _run(self):
        """Integrate by ELAPSED wall time, not the nominal tick: under GIL
        contention time.sleep overshoots, and integrating a fixed dt would make
        the robot crawl in wall-clock terms — which the skill's no-progress
        watchdog then (correctly, for that world) reads as a stall."""
        tick = 1.0 / self.rate
        last = time.time()
        while not self._stop.is_set():
            time.sleep(tick)
            now = time.time()
            # A busy CI host can delay this thread >50 ms. Discarding most of
            # that elapsed time makes the fake ball crawl and spuriously trips
            # the real controller's stall/segment timers; bound only big pauses.
            dt = min(0.20, max(0.0, now - last))
            last = now
            x, y, yaw = tb4._get('odom_x'), tb4._get('odom_y'), tb4._get('odom_yaw')
            v, w = self._v, self._w
            yaw += w * dt
            nx = x + v * math.cos(yaw) * dt
            ny = y + v * math.sin(yaw) * dt
            bx, by = self.ball
            d = math.hypot(nx - bx, ny - by)
            if d < self.CONTACT and v > 0.0 and d < math.hypot(x - bx, y - by):
                # Resolve the overlap by moving the BALL away from the robot's
                # proposed pose. Clamping the robot to the OLD ball position
                # (as this fake used to do) pins it exactly at contact: next
                # tick its movement is undone and the ball never advances.
                ux, uy = ((bx - nx) / d, (by - ny) / d) if d > 1e-6 else \
                         (math.cos(yaw), math.sin(yaw))
                overlap = self.CONTACT - d
                self.ball = [bx + ux * overlap, by + uy * overlap]
                self.ball_track.append(tuple(self.ball))
            tb4._set(odom_x=nx, odom_y=ny, odom_yaw=yaw, odom_t=time.time())
            if self.scan is not None:
                tb4._set(scan=self.scan, scan_t=time.time())
            self._publish_ball()
            self._serve_nav()

    def ball_dist_to(self, goal):
        return math.hypot(self.ball[0] - goal[0], self.ball[1] - goal[1])


# ── Tests ─────────────────────────────────────────────────────────────────────
class TestPushGeometry(unittest.TestCase):
    """The pure 'line between the goal and the robot with the ball in between'."""

    def test_stance_is_behind_the_ball_on_the_goal_line(self):
        ball, goal = (1.0, 0.0), (3.0, 0.0)
        sx, sy = tb4.push_stance(ball, goal)
        self.assertAlmostEqual(sx, 1.0 - tb4.PUSH_STANDOFF)
        self.assertAlmostEqual(sy, 0.0)

    def test_stance_works_on_a_diagonal(self):
        ball, goal = (0.0, 0.0), (1.0, 1.0)
        sx, sy = tb4.push_stance(ball, goal)
        k = tb4.PUSH_STANDOFF / math.sqrt(2.0)
        self.assertAlmostEqual(sx, -k, places=6)
        self.assertAlmostEqual(sy, -k, places=6)
        # collinear: robot / ball / goal on one line, ball in the middle
        (ux, uy), _ = tb4.push_line(ball, goal)
        self.assertAlmostEqual((ball[0] - sx) * uy - (ball[1] - sy) * ux, 0.0, places=9)

    def test_stance_error_flags_the_wrong_side_and_off_line(self):
        ball, goal = (1.0, 0.0), (3.0, 0.0)
        # directly behind, on the line -> fine
        along, lat, bad = tb4.stance_error((0.0, 0.0), ball, goal)
        self.assertAlmostEqual(along, -1.0)
        self.assertAlmostEqual(lat, 0.0)
        self.assertFalse(bad)
        # in FRONT of the ball (would shove it away from the goal) -> reposition
        along, lat, bad = tb4.stance_error((2.0, 0.0), ball, goal)
        self.assertGreater(along, 0.0)
        self.assertTrue(bad)
        # behind but off the line -> reposition
        along, lat, bad = tb4.stance_error((0.0, 0.9), ball, goal)
        self.assertAlmostEqual(lat, 0.9)
        self.assertTrue(bad)

    def test_line_offset_uses_the_anchored_line(self):
        ball0, goal = (1.0, 0.0), (3.0, 0.0)
        self.assertAlmostEqual(tb4.line_offset((2.0, 0.4), ball0, goal), 0.4, places=6)
        # a ball that slid sideways shows up, even though ball->goal would rotate
        self.assertAlmostEqual(tb4.line_offset((2.0, -0.25), ball0, goal), 0.25, places=6)

    def test_arrived_go_to_can_satisfy_the_line_up_check(self):
        """PUSH_STANDOFF must exceed GOAL_TOL + PUSH_ALIGN_BACK, else a go_to that
        reports 'arrived' still fails stance_error and the loop re-lines-up
        forever."""
        self.assertGreater(tb4.PUSH_STANDOFF, tb4.GOAL_TOL + tb4.PUSH_ALIGN_BACK)
        self.assertGreater(tb4.PUSH_ALIGN_LAT, tb4.GOAL_TOL)

    def test_signed_offset_side(self):
        ball0, goal = (1.0, 0.0), (3.0, 0.0)
        self.assertGreater(tb4.line_signed_offset((2.0, 0.3), ball0, goal), 0.0)   # left
        self.assertLess(tb4.line_signed_offset((2.0, -0.3), ball0, goal), 0.0)     # right
        self.assertAlmostEqual(
            abs(tb4.line_signed_offset((2.0, 0.3), ball0, goal)),
            tb4.line_offset((2.0, 0.3), ball0, goal))

    def test_odom_local_round_trip(self):
        for yaw in (0.0, 0.7, -2.3, math.pi):
            lx, ly = 1.4, -0.6
            ox, oy = 3.0, -2.0
            px = ox + lx * math.cos(yaw) - ly * math.sin(yaw)
            py = oy + lx * math.sin(yaw) + ly * math.cos(yaw)
            bx, by = tb4._odom_to_local((px, py), ox, oy, yaw)
            self.assertAlmostEqual(bx, lx, places=9)
            self.assertAlmostEqual(by, ly, places=9)

    def test_rear_clearance_uses_the_rotated_lidar_frame(self):
        self.assertTrue(tb4._rear_path_clear(clear_scan(), 0.7))
        # In lidar coordinates +pi/2 points BEHIND the robot, not to its left.
        self.assertFalse(tb4._rear_path_clear(clear_scan((-0.5, 0.0)), 0.7))
        self.assertFalse(tb4._rear_path_clear(clear_scan((-0.5, 0.25)), 0.7))
        self.assertTrue(tb4._rear_path_clear(clear_scan((-0.5, 0.6)), 0.7))

    def test_reverse_needs_a_valid_360_degree_rear_scan(self):
        self.assertFalse(tb4._rear_path_clear(None, 0.7))
        partial = clear_scan()
        partial.ranges = partial.ranges[:180]
        self.assertFalse(tb4._rear_path_clear(partial, 0.7))
        invalid = clear_scan()
        invalid.ranges[270] = float('nan')
        self.assertFalse(tb4._rear_path_clear(invalid, 0.7))
        no_returns = clear_scan()
        no_returns.ranges = [float('inf')] * 360
        self.assertFalse(tb4._rear_path_clear(no_returns, 0.7))
        too_near = clear_scan()
        too_near.ranges[270] = 0.0
        self.assertFalse(tb4._rear_path_clear(too_near, 0.7))


class TestPushToGoal(unittest.TestCase):
    """Closed-loop: the real NavRobot.push_to_goal driving the fake base."""

    def _robot(self):
        return tb4.NavRobot()

    def test_no_goal_marked_returns_no_goal(self):
        tb4._set(ball_goal=None)
        with FakeBase((1.0, 0.0)):
            r = self._robot()
            self.assertEqual(r.push_to_goal(), 'no-goal')
            self.assertIn('no ball goal', tb4._get('status'))

    def test_pushes_a_straight_ball_to_the_goal(self):
        goal = (1.9, 0.0)
        with FakeBase((1.0, 0.0)) as base:
            res = self._robot().push_to_goal(goal[0], goal[1], max_time=60)
            self.assertEqual(res, 'at goal')
            self.assertLessEqual(base.ball_dist_to(goal), tb4.PUSH_GOAL_DONE)
            self.assertEqual(tb4._get('ball_goal'), goal)

    def test_backs_up_when_close_behind_then_pushes_forward(self):
        goal = (1.55, 0.0)
        # 0.34 m behind: too little room to dock straight on the push line.
        # Keep the goal close so this test probes repositioning, not the
        # unrelated 25-second shove limit when the fake base is CPU-starved.
        with FakeBase((1.0, 0.0), pose=(0.66, 0.0, 0.0), scan=clear_scan()) as base:
            res = self._robot().push_to_goal(*goal, max_time=60)
            self.assertEqual(res, 'at goal')
            self.assertTrue(any(v < 0 for v, _ in base.twists))
            self.assertTrue(any(v > 0 for v, _ in base.twists))
            self.assertLessEqual(base.ball_dist_to(goal), tb4.PUSH_GOAL_DONE)
            self.assertGreaterEqual(min(b[0] for b in base.ball_track), 0.95)

    def test_reverse_is_only_for_close_aligned_repositioning(self):
        ball, goal = (1.0, 0.0), (1.9, 0.0)
        for pose in ((1.4, 0.0, math.pi),   # ahead of the ball
                     (1.0, 0.4, 0.0),      # abreast
                     (0.65, 0.4, 0.0),     # off the push line
                     (0.65, 0.0, 0.5),     # heading across the line
                     (-0.6, 0.0, 0.0)):   # already far behind
            with self.subTest(pose=pose):
                with FakeBase(ball, pose=pose, scan=clear_scan()) as base:
                    self.assertEqual(self._robot()._back_off_for_push(ball, goal), 'skip')
                    self.assertFalse(any(v < 0 for v, _ in base.twists))

    def test_blocked_or_stale_rear_scan_never_starts_reverse(self):
        ball, goal = (1.0, 0.0), (1.9, 0.0)
        for scan in (clear_scan((-0.5, 0.0)), None):
            with self.subTest(scan=scan):
                with FakeBase(ball, pose=(0.66, 0.0, 0.0), scan=scan) as base:
                    self.assertEqual(self._robot()._back_off_for_push(ball, goal), 'skip')
                    self.assertFalse(any(v < 0 for v, _ in base.twists))
        with FakeBase(ball, pose=(0.66, 0.0, 0.0)) as base:
            tb4._set(scan=clear_scan(), scan_t=time.time() - 3.0)
            self.assertEqual(self._robot()._back_off_for_push(ball, goal), 'skip')
            self.assertFalse(any(v < 0 for v, _ in base.twists))

    def test_cliff_during_reverse_stops_and_blocks_the_task(self):
        class CliffBase(FakeBase):
            def send_twist(self, linear, angular):
                super().send_twist(linear, angular)
                if linear < 0:
                    tb4._set(hazards=[{'type': 'cliff'}])

        with CliffBase((1.0, 0.0), pose=(0.66, 0.0, 0.0), scan=clear_scan()) as base:
            res = self._robot().push_to_goal(1.9, 0.0, max_time=20)
            self.assertEqual(res, 'blocked')
            self.assertTrue(any(v < 0 for v, _ in base.twists))
            self.assertEqual(base.twists[-1], (0.0, 0.0))
            self.assertAlmostEqual(base.ball[0], 1.0)

    def test_lost_lidar_during_reverse_stops(self):
        class StaleBase(FakeBase):
            def send_twist(self, linear, angular):
                super().send_twist(linear, angular)
                if linear < 0:
                    self.scan = None
                    tb4._set(scan_t=time.time() - 3.0)

        with StaleBase((1.0, 0.0), pose=(0.66, 0.0, 0.0), scan=clear_scan()) as base:
            self.assertEqual(self._robot()._back_off_for_push((1.0, 0.0), (1.9, 0.0)),
                             'blocked')
            self.assertTrue(any(v < 0 for v, _ in base.twists))
            self.assertEqual(base.twists[-1], (0.0, 0.0))

    def test_new_obstacle_during_reverse_stops(self):
        class ObstacleBase(FakeBase):
            def send_twist(self, linear, angular):
                super().send_twist(linear, angular)
                if linear < 0:
                    self.scan = clear_scan((-0.4, 0.0))
                    tb4._set(scan=self.scan, scan_t=time.time())

        with ObstacleBase((1.0, 0.0), pose=(0.66, 0.0, 0.0), scan=clear_scan()) as base:
            self.assertEqual(self._robot()._back_off_for_push((1.0, 0.0), (1.9, 0.0)),
                             'blocked')
            self.assertTrue(any(v < 0 for v, _ in base.twists))
            self.assertEqual(base.twists[-1], (0.0, 0.0))

    def test_abort_during_reverse_stops_the_base(self):
        robot = self._robot()

        class AbortBase(FakeBase):
            def send_twist(self, linear, angular):
                super().send_twist(linear, angular)
                if linear < 0:
                    robot.abort()

        with AbortBase((1.0, 0.0), pose=(0.66, 0.0, 0.0), scan=clear_scan()) as base:
            with self.assertRaises(tb4.toyscript.StopProgram):
                robot._back_off_for_push((1.0, 0.0), (1.9, 0.0))
            self.assertTrue(any(v < 0 for v, _ in base.twists))
            self.assertEqual(base.twists[-1], (0.0, 0.0))

    def test_reverse_stall_stops_if_base_cannot_move_backward(self):
        class StuckBase(FakeBase):
            def send_twist(self, linear, angular):
                super().send_twist(linear, angular)
                if linear < 0:
                    self._v = 0.0            # Create 3 ignored the reverse command

        with StuckBase((1.0, 0.0), pose=(0.66, 0.0, 0.0), scan=clear_scan()) as base:
            self.assertEqual(self._robot()._back_off_for_push((1.0, 0.0), (1.9, 0.0)),
                             'blocked')
            self.assertIn('stalled', tb4._get('status'))
            self.assertEqual(base.twists[-1], (0.0, 0.0))

    def test_lines_up_first_when_the_robot_is_off_the_line(self):
        """Robot starts 0.8 m to the side: it must drive to the stance point
        BEHIND the ball (go_to) before shoving, and still deliver the ball."""
        goal = (1.9, 0.0)
        with FakeBase((1.0, 0.0), pose=(0.0, 0.8, 0.0)) as base:
            res = self._robot().push_to_goal(goal[0], goal[1], max_time=90)
            self.assertEqual(res, 'at goal')
            self.assertLessEqual(base.ball_dist_to(goal), tb4.PUSH_GOAL_DONE)

    def test_never_shoves_the_ball_away_from_the_goal(self):
        """Robot starts abreast of the ball (level with it, not behind). It must
        back off and line up rather than push the ball the wrong way."""
        goal = (2.5, 0.0)
        with FakeBase((1.0, 0.0), pose=(1.0, -1.0, 0.0), scan=clear_scan()) as base:
            start = base.ball_dist_to(goal)
            res = self._robot().push_to_goal(goal[0], goal[1], max_time=90)
            end = base.ball_dist_to(goal)
            self.assertEqual(res, 'at goal')
            self.assertLess(end, start)
            self.assertFalse(any(v < 0 for v, _ in base.twists))
            # the ball never got meaningfully farther from the goal than it started
            worst = max([start] + [math.hypot(b[0] - goal[0], b[1] - goal[1])
                                   for b in base.ball_track])
            self.assertLessEqual(worst, start + 0.10)

    def test_shove_holds_the_ball_on_the_line(self):
        """Regression for the push controller's sign. 'Keep the ball centred' alone
        is not enough: a ball rolls along the robot→ball line, so a robot that is
        a few cm off the push line keeps the ball centred while shoving it further
        off the line, and the drift locks in. Start 0.18 m off the line (well
        inside GOAL_TOL, i.e. a normal go_to arrival) and the ball must stay on
        the line — flipping the lateral servo's sign makes this diverge."""
        goal, ball0 = (1.9, 0.0), (1.0, 0.0)
        with FakeBase(ball0, pose=(0.35, 0.18, 0.0)) as base:
            res = tb4.NavRobot().push_to_goal(goal[0], goal[1], max_time=60)
            self.assertEqual(res, 'at goal')
            worst = max(tb4.line_offset(b, ball0, goal) for b in base.ball_track)
            self.assertLess(worst, 0.15)

    def test_marks_the_ball_and_stance_for_the_bev(self):
        goal = (1.6, 0.0)
        with FakeBase((1.0, 0.0)) as base:
            self._robot().push_to_goal(goal[0], goal[1], max_time=60)
            mark = tb4._get('ball_mark')
            self.assertIsNotNone(mark)
            self.assertLessEqual(math.hypot(mark[0] - base.ball[0],
                                            mark[1] - base.ball[1]), 0.05)

    def test_lost_ball_reports_lost(self):
        """No ball anywhere: _locate_ball exhausts jitter + sweep and says lost."""
        class Blind(FakeBase):
            def _publish_ball(self):
                tb4._set(detections=[])

        with Blind((5.0, 5.0)) as base:
            robot = self._robot()
            robot.scan_for = lambda name, max_deg=360.0: None   # sweep finds nothing
            self.assertEqual(robot.push_to_goal(2.0, 0.0, max_time=30), 'lost')


class TestHttpSurface(unittest.TestCase):
    def setUp(self):
        tb4.app.testing = True
        self.client = tb4.app.test_client()
        # The navigator's state is module-global, so a drive test that ran
        # earlier in this process leaves goal_odom/nav_active set. These tests
        # assert on a *fresh* navigator ("marking a goal must not start a
        # drive"), so reset the nav fields here instead of inheriting them —
        # otherwise the result depends on test execution order.
        tb4._set(ball_goal=None, push_from=None, goal_odom=None,
                 goal_local=None, nav_active=False, path=[])

    def test_set_and_clear_ball_goal(self):
        r = self.client.post('/set_ball_goal', json={'x': 1.25, 'y': -0.5})
        self.assertEqual(r.status_code, 200)
        self.assertEqual(r.get_json()['ball_goal'], [1.25, -0.5])
        self.assertEqual(tb4._get('ball_goal'), (1.25, -0.5))
        # surfaced in /state so the UI can show it
        self.assertEqual(self.client.get('/state').get_json()['ball_goal'], [1.25, -0.5])
        # marking must NOT start the robot driving
        self.assertFalse(tb4._get('nav_active'))
        self.assertIsNone(tb4._get('goal_odom'))
        r = self.client.post('/set_ball_goal', json={'clear': True})
        self.assertEqual(r.status_code, 200)
        self.assertIsNone(tb4._get('ball_goal'))

    def test_index_renders_the_ball_goal_controls(self):
        """The UI is one big Jinja template — a typo breaks the whole page."""
        html = self.client.get('/').get_data(as_text=True)
        for needle in ('ball_goal_mode', 'clearBallGoal', 'ballgoal_disp',
                       '/set_ball_goal', 'Ball goal'):
            self.assertIn(needle, html)

    def test_set_ball_goal_needs_coordinates(self):
        r = self.client.post('/set_ball_goal', json={'x': 'abc'})
        self.assertEqual(r.status_code, 400)

    def test_marked_goal_is_used_by_push_to_goal(self):
        with FakeBase((1.0, 0.0)) as base:
            self.client.post('/set_ball_goal', json={'x': 1.9, 'y': 0.0})
            res = tb4.NavRobot().push_to_goal(max_time=60)   # no args -> the mark
            self.assertEqual(res, 'at goal')
            self.assertLessEqual(base.ball_dist_to((1.9, 0.0)), tb4.PUSH_GOAL_DONE)


class TestProgramWiring(unittest.TestCase):
    def test_program_loads_with_triggers_and_plan(self):
        progs = tb4.load_programs()
        self.assertIn('push_ball_to_goal', progs)
        m = progs['push_ball_to_goal']
        self.assertIn('push the ball to the goal', m['triggers'])
        self.assertEqual([s['verb'] for s in m['plan']], ['SCAN_FOR', 'PUSH_TO_GOAL'])

    def test_phrase_matches_the_new_program(self):
        progs = tb4.load_programs()
        for text in ('push the ball to the goal', 'push ball to goal',
                     'push the ball there'):
            matched, score = tb4.match_program(text, progs)
            self.assertEqual(matched['name'], 'push_ball_to_goal', text)
            self.assertEqual(score, 1.0)

    def test_old_push_programs_still_win_their_own_phrases(self):
        progs = tb4.load_programs()
        for text, want in (('push the ball to the wall', 'push_ball_to_wall'),
                           ('push to wall', 'push_to_wall'),
                           # follow_human was folded into the generic `follow`
                           # program; its phrases must now land there.
                           ('follow me', 'follow')):
            self.assertEqual(tb4.match_program(text, progs)[0]['name'], want, text)

    def test_every_follow_phrase_reaches_the_one_program(self):
        """follow_human / follow_dog / follow_ball / follow_ball_aggressive were
        four .toy files that differed only in a target, a standoff and a
        controller choice -- all of which follow from WHAT you named. They are now
        one program, so every phrase that used to reach any of them must still
        reach `follow` (and nothing may resolve to a deleted name)."""
        progs = tb4.load_programs()
        for gone in ('follow_human', 'follow_dog', 'follow_ball',
                     'follow_ball_aggressive'):
            self.assertNotIn(gone, progs)
        for text in ('follow me', 'help me carry', 'carry mode',
                     'follow the human', 'follow person', 'come with me',
                     'follow the dog', 'chase the dog', 'track the dog',
                     'follow the puppy', 'come with dog',
                     'follow the ball', 'chase the ball', 'trail the ball',
                     'track the ball', 'predict the ball',
                     'follow the ball aggressively', 'chase the ball fast'):
            matched, _ = tb4.match_program(text, progs)
            self.assertEqual(matched['name'], 'follow', text)

    def test_follow_program_hardcodes_no_target(self):
        """The single follow.toy must ask the skill to follow with NO arguments,
        so target/standoff/fast come from the phrase via state rather than from
        DSL text. A literal baked into the .toy is how the duplication crept back
        in last time."""
        import toyscript
        progs = tb4.load_programs()

        class Rec(toyscript.MockRobot):
            def __init__(self):
                super().__init__(log=lambda *a: None)
                self.calls = []

            def follow(self, *a, **k):
                self.calls.append((a, k))
                return 'done'

        rec = Rec()
        toyscript.Interpreter(rec, log=lambda *a: None).run(progs['follow']['source'])
        self.assertEqual(rec.calls, [((), {})],
                         "follow.toy should call FOLLOW() bare")

    def test_dsl_follow_takes_zero_to_three_arguments(self):
        """FOLLOW() is the normal form (everything comes from the phrase), but a
        hand-written .toy or a test must still be able to pin a target, a
        standoff and the predictor. The third argument replaces the deleted
        FOLLOW_FAST verb, so dispatch has to accept all four arities."""
        import toyscript

        class Rec(toyscript.MockRobot):
            def __init__(self):
                super().__init__(log=lambda *a: None)
                self.calls = []

            def follow(self, *a, **k):
                self.calls.append(a)
                return 'done'

        for src, want in (('SET r = FOLLOW()',                ()),
                          ('SET r = FOLLOW("dog")',           ('dog',)),
                          ('SET r = FOLLOW("dog", 1.5)',      ('dog', 1.5)),
                          ('SET r = FOLLOW("ball", 0.7, 1)',  ('ball', 0.7, True)),
                          ('SET r = FOLLOW("ball", 0.7, 0)',  ('ball', 0.7, False))):
            rec = Rec()
            toyscript.Interpreter(rec, log=lambda *a: None).run(src)
            self.assertEqual(rec.calls, [want], src)
        # FOLLOW_FAST is gone: it must be an unknown primitive, not a silent no-op
        with self.assertRaises(NameError):
            toyscript.Interpreter(Rec(), log=lambda *a: None).run(
                'SET r = FOLLOW_FAST("ball", 0.7)')

    def test_track_the_dog_still_follows_a_dog(self):
        """Regression guard for the merge that started this: 'track' was NOT in
        _FOLLOW_VERBS, so "track the dog" parsed to nothing and only worked
        because follow_dog.toy declared it as a literal trigger. Now that the verb
        list and the programs are one source of truth, 'track' parses -- and a dog
        must get the big-target treatment (follow at once, 1 m standoff, no lidar
        predictor), exactly as follow_dog used to."""
        self.assertEqual(tb4._parse_follow_target('track the dog'), 'dog')
        p = tb4._follow_params('track the dog')
        self.assertEqual(p['target'], 'dog')
        self.assertEqual(p['standoff'], tb4.FOLLOW_STANDOFF)
        self.assertFalse(p['scan_first'])
        self.assertFalse(p['fast'])

    def test_small_targets_get_the_close_standoff_and_a_scan_first(self):
        """A ~40 cm object is lost at 1 m in the 416-px preview, and FOLLOW's
        re-acquire is a CONTINUOUS spin the OAK detects poorly through -- so small
        targets stand off closer and are acquired step-and-stare. Big targets
        start following at once, because FOLLOW's own sweep re-finds them."""
        for text, want in (('follow the ball', 'sports ball'),
                           ('chase the ball', 'sports ball'),
                           ('follow the bottle', 'bottle'),
                           ('follow the cup', 'cup')):
            p = tb4._follow_params(text)
            self.assertEqual(p['target'], want, text)
            self.assertEqual(p['standoff'], tb4.FOLLOW_STANDOFF_CLOSE, text)
            self.assertTrue(p['scan_first'], text)
            self.assertFalse(p['fast'], text)
        for text, want in (('follow me', None),
                           ('follow the dog', 'dog'),
                           ('follow the person', 'person'),
                           ('follow the cat', 'cat')):
            p = tb4._follow_params(text)
            self.assertEqual(p['target'], want, text)
            self.assertEqual(p['standoff'], tb4.FOLLOW_STANDOFF, text)
            self.assertFalse(p['scan_first'], text)
            self.assertFalse(p['fast'], text)

    def test_the_lidar_predictor_stays_reachable_for_balls_only(self):
        """follow_ball_aggressive.toy was the only caller of FOLLOW_FAST, and
        /run's special case shadowed it for every phrase except "track/predict the
        ball" -- so the skill was nearly unreachable. It must stay reachable for
        ball phrases that ask to follow hard, and be REFUSED for big targets: its
        lidar model hunts a close blob on the floor, wrong for a person or dog."""
        for text in ('track the ball', 'predict the ball',
                     'follow the ball aggressively', 'aggressive ball follow',
                     'chase the ball fast', 'follow the ball fast'):
            p = tb4._follow_params(text)
            self.assertTrue(p['fast'], text)
            self.assertEqual(p['standoff'], tb4.FOLLOW_STANDOFF_CLOSE, text)
        for text in ('follow the dog fast', 'follow me quickly',
                     'track the person aggressively', 'chase the cat hard'):
            self.assertFalse(tb4._follow_params(text)['fast'], text)

    def test_a_fast_word_alone_does_not_invent_a_target(self):
        """'fast'/'track'/'predict' with no small object named must not
        manufacture a ball to follow -- that would turn an unrelated phrase into a
        floor-hunting predictor run."""
        for text in ('go fast', 'that was quick', 'predict the weather',
                     'track my package'):
            p = tb4._follow_params(text)
            self.assertIsNone(p['target'], text)
            self.assertFalse(p['fast'], text)
    def test_dsl_dispatches_push_to_goal(self):
        import toyscript

        class Recorder(toyscript.MockRobot):
            def __init__(self):
                super().__init__(log=lambda *a: None)
                self.calls = []

            def push_to_goal(self, *a, **k):
                self.calls.append((a, k))
                return 'at goal'

        rec = Recorder()
        toyscript.Interpreter(rec, log=lambda *a: None).run('SET r = PUSH_TO_GOAL()')
        self.assertEqual(rec.calls, [((), {})])
        rec.calls.clear()
        toyscript.Interpreter(rec, log=lambda *a: None).run('SET r = PUSH_TO_GOAL(1.5, -0.25)')
        self.assertEqual(rec.calls, [((1.5, -0.25), {})])


class TestBevRendering(unittest.TestCase):
    def test_ball_goal_layer_renders(self):
        png_plain = tb4.render_lidar_png(None, [], None, [], None, 'move')
        png_marked = tb4.render_lidar_png(None, [], None, [], None, 'move',
                                          ball_goal_local=(1.5, 0.4),
                                          ball_local=(0.8, 0.1),
                                          push_from_local=(0.25, -0.1))
        self.assertTrue(png_plain.startswith(b'\x89PNG'))
        self.assertTrue(png_marked.startswith(b'\x89PNG'))
        self.assertNotEqual(png_plain, png_marked)


if __name__ == '__main__':
    unittest.main(verbosity=2)
