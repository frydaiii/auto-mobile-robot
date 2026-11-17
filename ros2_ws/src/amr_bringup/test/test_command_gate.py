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
        self.assertEqual(gate.output(1.1), (0.15, -0.6))
        gate.receive(-2.0, 3.0, 1.2)
        self.assertEqual(gate.output(1.3), (-0.15, 0.6))

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
        for name in ('linear_limit', 'angular_limit', 'timeout'):
            for value in (0.0, -1.0, math.inf, math.nan):
                with self.assertRaises(ValueError):
                    CommandGate(**{name: value})


if __name__ == '__main__':
    unittest.main()
