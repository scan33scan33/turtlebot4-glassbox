#!/usr/bin/env python3
"""Offline regressions for the default push_ball_to_wall program and PUSH_AWAY.

Shares the closed-loop fake Create 3 / forward OAK camera with test_push_to_goal.
Run: python3 -m unittest tests.test_push_to_wall -v
"""
import os
import sys
import time
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from tests.test_push_to_goal import FakeBase, clear_scan, tb4  # ROS stubs + simulation
import toyscript


class TestWallPush(unittest.TestCase):
    def test_close_ball_moves_with_live_camera_without_astar_or_lidar(self):
        """Regression: old PUSH_AWAY sent A* to the ball's old coordinate;
        camera SCAN_FOR succeeded but without /scan the bot waited, not pushed.
        """
        with FakeBase((0.80, 0.0), scan=None) as base:
            result = tb4.NavRobot().push_away(0.80, 0.0, step=0.25)
            self.assertEqual(result, 'pushed')
            self.assertGreater(base.ball[0], 1.0)
            self.assertFalse(tb4._get('nav_active'))
            self.assertIsNone(tb4._get('goal_odom'))
            self.assertTrue(any(v > 0 for v, _ in base.twists))
            self.assertEqual(base.twists[-1], (0.0, 0.0))

    def test_live_steering_centres_an_off_axis_ball(self):
        with FakeBase((0.80, 0.16), scan=None) as base:
            result = tb4.NavRobot().push_away(0.80, 0.16, step=0.16)
            self.assertEqual(result, 'pushed')
            self.assertGreater(base.ball[0], 0.85)
            self.assertTrue(any(abs(w) > 0.0 for _, w in base.twists))
            self.assertEqual(base.twists[-1], (0.0, 0.0))

    def test_far_ball_needs_a_recent_lidar_scan_to_route_around_furniture(self):
        with FakeBase((1.8, 0.0), scan=None) as base:
            result = tb4.NavRobot().push_away(1.8, 0.0)
            self.assertEqual(result, 'blocked')
            self.assertIn('lidar unavailable', tb4._get('status'))
            self.assertFalse(any(v > 0 for v, _ in base.twists))

    def test_far_ball_approaches_short_then_uses_live_shove(self):
        with FakeBase((1.7, 0.0), scan=clear_scan()) as base:
            result = tb4.NavRobot().push_away(1.7, 0.0, step=0.16)
            self.assertEqual(result, 'pushed')
            self.assertGreater(base.ball[0], 1.75)
            self.assertTrue(any(v > 0 for v, _ in base.twists))
            self.assertEqual(base.twists[-1], (0.0, 0.0))

    def test_failed_far_approach_never_falls_through_to_a_blind_shove(self):
        class BlockedPlanner(tb4.NavRobot):
            def go_to(self, *args, **kwargs):
                return 'blocked: no path'

        with FakeBase((1.7, 0.0), scan=clear_scan()) as base:
            self.assertEqual(BlockedPlanner().push_away(1.7, 0.0), 'blocked')
            self.assertIn('cannot approach ball', tb4._get('status'))
            self.assertFalse(any(v > 0 for v, _ in base.twists))

    def test_stale_nn_detection_does_not_drive_on_an_old_ball_position(self):
        class FrozenCamera(FakeBase):
            def _publish_ball(self):
                super()._publish_ball()
                tb4._set(det_t=time.time() - 3.0)  # NN stopped, detections persist

        with FrozenCamera((0.85, 0.0)) as base:
            robot = tb4.NavRobot()
            robot._jitter_find = lambda *args, **kw: None
            self.assertEqual(robot.push_away(0.85, 0.0), 'lost')
            self.assertIn('stale location', tb4._get('status'))
            self.assertFalse(any(v > 0 for v, _ in base.twists))

    def test_ball_lost_far_from_bumper_stops_instead_of_blind_drive(self):
        class VanishingBall(FakeBase):
            def send_twist(self, linear, angular):
                super().send_twist(linear, angular)
                if linear > 0:
                    self.visible = False

            def _publish_ball(self):
                if getattr(self, 'visible', True):
                    super()._publish_ball()
                else:
                    tb4._set(detections=[], det_t=time.time())

        with VanishingBall((0.98, 0.0)) as base:
            self.assertEqual(tb4.NavRobot().push_away(0.98, 0.0), 'lost')
            self.assertIn('lost away from bumper', tb4._get('status'))
            self.assertEqual(base.twists[-1], (0.0, 0.0))
            self.assertLess(base.ball[0], 1.0)

    def test_ball_disappearing_near_bumper_only_gets_a_short_finish(self):
        class LowCamera(FakeBase):
            def _publish_ball(self):
                super()._publish_ball()
                dets = tb4._get('detections')
                if dets and dets[0]['dist'] < 0.56:
                    if not hasattr(self, 'lost_x'):
                        self.lost_x = tb4._get('odom_x')
                    tb4._set(detections=[], det_t=time.time())

        with LowCamera((0.80, 0.0)) as base:
            self.assertEqual(tb4.NavRobot().push_away(0.80, 0.0, step=0.45),
                             'at-feet')
            self.assertEqual(base.twists[-1], (0.0, 0.0))
            self.assertLess(tb4._get('odom_x') - base.lost_x,
                            tb4.PUSH_AWAY_BLIND + 0.03)

    def test_not_moving_cannot_be_mistaken_for_ball_pinned_against_wall(self):
        class DisabledWheels(FakeBase):
            def send_twist(self, linear, angular):
                super().send_twist(linear, angular)
                if linear > 0:
                    self._v = 0.0

        with DisabledWheels((0.60, 0.0)) as base:
            self.assertEqual(tb4.NavRobot().push_away(0.60, 0.0), 'blocked')
            self.assertIn('stalled before contact', tb4._get('status'))
            self.assertEqual(base.twists[-1], (0.0, 0.0))

    def test_cliff_mid_push_stops_even_without_active_nav_goal(self):
        class Cliff(FakeBase):
            def send_twist(self, linear, angular):
                super().send_twist(linear, angular)
                if linear > 0:
                    tb4._set(hazards=[{'type': 'cliff'}])

        with Cliff((0.75, 0.0)) as base:
            self.assertEqual(tb4.NavRobot().push_away(0.75, 0.0), 'blocked')
            self.assertTrue(any(v > 0 for v, _ in base.twists))
            self.assertEqual(base.twists[-1], (0.0, 0.0))

    def test_abort_during_shove_always_stops_motors(self):
        robot = tb4.NavRobot()

        class Abort(FakeBase):
            def send_twist(self, linear, angular):
                super().send_twist(linear, angular)
                if linear > 0:
                    robot.abort()

        with Abort((0.75, 0.0)) as base:
            with self.assertRaises(toyscript.StopProgram):
                robot.push_away(0.75, 0.0)
            self.assertTrue(any(v > 0 for v, _ in base.twists))
            self.assertEqual(base.twists[-1], (0.0, 0.0))
            self.assertEqual(tb4._get('goal_mode'), 'move')

    def test_straight_finish_does_not_report_pinned_on_disabled_wheels(self):
        class DisabledWheels(FakeBase):
            def send_twist(self, linear, angular):
                super().send_twist(linear, angular)
                self._v = 0.0

        with DisabledWheels((0.38, 0.0)) as base:
            self.assertEqual(tb4.NavRobot().push_through(0.30), 'blocked')
            self.assertEqual(base.twists[-1], (0.0, 0.0))

    def test_straight_finish_stops_on_cliff(self):
        class Cliff(FakeBase):
            def send_twist(self, linear, angular):
                super().send_twist(linear, angular)
                if linear > 0:
                    tb4._set(hazards=[{'type': 'cliff'}])

        with Cliff((0.38, 0.0)) as base:
            self.assertEqual(tb4.NavRobot().push_through(0.30), 'blocked')
            self.assertTrue(any(v > 0 for v, _ in base.twists))
            self.assertEqual(base.twists[-1], (0.0, 0.0))


class TestWallProgram(unittest.TestCase):
    def setUp(self):
        self.source = tb4.load_programs()['push_ball_to_wall']['source']

    def test_end_to_end_wall_push_pins_ball_with_real_skill(self):
        class Wall(FakeBase):
            def send_twist(self, linear, angular):
                super().send_twist(linear, angular)
                if linear > 0 and self.ball[0] >= 1.35:
                    self._v = 0.0                # ball meets the wall

        with Wall((0.85, 0.0), scan=None) as base:
            logs = []
            result = toyscript.Interpreter(tb4.NavRobot(), log=logs.append).run(self.source)
            self.assertEqual(result['r'], 'pinned')
            self.assertGreater(base.ball[0], 1.35)
            self.assertTrue(any(v > 0 for v, _ in base.twists))
            self.assertTrue(any('Ball pinned against the wall.' in line for line in logs))

    def test_nn_health_is_separate_from_raw_camera_health(self):
        tb4.app.testing = True
        client = tb4.app.test_client()
        tb4._set(det_t=time.time(), img_t=time.time(),
                 cmd_sent={'v': 0.16, 'w': 0.0, 't': time.time()})
        self.assertTrue(client.get('/state').get_json()['nn_ok'])
        self.assertAlmostEqual(client.get('/state').get_json()['cmd_sent']['v'], 0.16)
        self.assertLess(client.get('/state').get_json()['cmd_age_s'], 1.0)
        tb4._set(det_t=time.time() - 5.0)
        state = client.get('/state').get_json()
        self.assertFalse(state['nn_ok'])
        self.assertTrue(state['cam_ok'])
        html = client.get('/').get_data(as_text=True)
        self.assertIn('h_nn', html)
        self.assertIn("s.nn_ok", html)
        self.assertIn('cmd_disp', html)

    def test_failed_push_does_not_blindly_push_through_or_claim_done(self):
        calls = []

        class BlockedRobot(toyscript.MockRobot):
            def scan_for(self, *args):
                return {'x': 0.8, 'y': 0.0}

            def find(self, *args):
                return None   # camera flickered after initial sighting

            def wait(self, *args):
                return 'done'

            def push_away(self, *args):
                calls.append('push_away')
                return 'blocked'

            def push_through(self, *args):
                calls.append('push_through')
                return 'done'

        logs = []
        result = toyscript.Interpreter(BlockedRobot(log=lambda *a: None),
                                       log=logs.append).run(self.source)
        self.assertEqual(result['r'], 'blocked')
        self.assertEqual(calls, ['push_away'])
        self.assertIn('blocked', logs)
        self.assertFalse(any('Done.' in line for line in logs))

    def test_finish_blind_only_after_confirmed_ball_at_feet(self):
        calls = []

        class CloseBall(toyscript.MockRobot):
            def scan_for(self, *args):
                return {'x': 0.5, 'y': 0.0}

            def find(self, *args):
                return None

            def wait(self, *args):
                return 'done'

            def push_away(self, *args):
                calls.append('push_away')
                return 'at-feet'

            def push_through(self, *args):
                calls.append('push_through')
                return 'pinned'

        result = toyscript.Interpreter(CloseBall(log=lambda *a: None),
                                       log=lambda *a: None).run(self.source)
        self.assertEqual(result['r'], 'pinned')
        self.assertEqual(calls, ['push_away', 'push_through'])


if __name__ == '__main__':
    unittest.main(verbosity=2)
