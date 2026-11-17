"""The gate must stop on stale or malformed commands and bound both axes."""
import math
import unittest

from amr_bringup.command_gate import CommandGate


class CommandGateTest(unittest.TestCase):
    def test_starts_stopped(self):
        self.assertEqual(CommandGate().output(10.0), (0.0, 0.0))

    def test_limits_both_directions(self):
        gate = CommandGate()
        gate.receive(2.0, -3.0, 1.0)
        forward = gate.output(1.1)
        self.assertLessEqual(abs(forward[0] - forward[1]*0.11), 0.065*math.pi + 1e-12)
        self.assertAlmostEqual(forward[1]/forward[0], -4.0)
        gate.receive(-2.0, 3.0, 1.2)
        self.assertEqual(gate.output(1.3), tuple(-v for v in forward))

    def test_wheel_rpm_configuration_and_custom_geometry(self):
        gate = CommandGate(wheel_radius=0.05, wheel_separation=0.4, max_wheel_rpm=30)
        gate.receive(0.15, 0.6, 1.0)
        v, w = gate.output(1.1)
        self.assertAlmostEqual(v + w*0.2, math.pi*0.05)
        self.assertAlmostEqual(w/v, 4.0)

    def test_bumpers_require_fresh_clear_feedback_and_new_command(self):
        gate = CommandGate(require_bumpers=True)
        gate.receive(0.1, 0.0, 1.0)
        self.assertEqual(gate.output(1.01), (0.0, 0.0))
        for name in ('left', 'center', 'right'):
            gate.receive_bumper(name, False, 1.02)
        self.assertEqual(gate.output(1.03), (0.0, 0.0))
        gate.receive(0.1, 0.0, 1.04)
        self.assertEqual(gate.output(1.05), (0.1, 0.0))
        gate.receive_bumper('center', True, 1.06)
        self.assertEqual(gate.output(1.07), (0.0, 0.0))
        gate.receive(0.1, 0.0, 1.08)
        gate.receive_bumper('center', False, 1.09)
        self.assertEqual(gate.output(1.10), (0.0, 0.0))
        gate.receive(0.1, 0.0, 1.11)
        self.assertEqual(gate.output(1.12), (0.1, 0.0))
        # The command is fresh but left/right feedback has expired.
        gate.receive(0.1, 0.0, 1.50)
        self.assertEqual(gate.output(1.52), (0.0, 0.0))
        for name in ('left', 'center', 'right'):
            gate.receive_bumper(name, False, 1.53)
        self.assertEqual(gate.output(1.54), (0.0, 0.0))

    def test_expires_at_boundary_and_recovers(self):
        gate = CommandGate()
        gate.receive(0.1, 0.2, 1.0)
        self.assertEqual(gate.output(1.499), (0.1, 0.2))
        self.assertEqual(gate.output(1.5), (0.0, 0.0))
        gate.receive(-0.1, -0.2, 2.0)
        self.assertEqual(gate.output(2.1), (-0.1, -0.2))

    def test_invalid_commands_clear_previous_motion(self):
        for bad in (math.nan, math.inf, -math.inf):
            for linear, angular in ((bad, 0.1), (0.1, bad)):
                gate = CommandGate()
                gate.receive(0.1, 0.2, 1.0)
                gate.receive(linear, angular, 1.1)
                self.assertEqual(gate.output(1.2), (0.0, 0.0))

    def test_clock_reversal_stops(self):
        gate = CommandGate()
        gate.receive(0.1, 0.2, 3.0)
        self.assertEqual(gate.output(2.0), (0.0, 0.0))

    def test_configuration_must_be_positive_and_finite(self):
        for name in ('linear_limit', 'angular_limit', 'timeout', 'wheel_radius',
                     'wheel_separation', 'max_wheel_rpm', 'bumper_timeout'):
            for value in (0.0, -1.0, math.inf, math.nan):
                with self.assertRaises(ValueError):
                    CommandGate(**{name: value})


if __name__ == '__main__':
    unittest.main()
