"""Conversions from simulated measurements to hardware-style feedback."""
import math


class EncoderTicks:
    """Count quadrature edges per wheel revolution from continuous joint angles."""

    names = ('left_wheel_joint', 'right_wheel_joint')

    def __init__(self, counts_per_revolution):
        cpr = counts_per_revolution
        if not math.isfinite(cpr) or cpr <= 0 or int(cpr) != cpr:
            raise ValueError('Encoder counts per wheel revolution must be a positive integer')
        self.cpr = cpr
        self._origin = None
        self._stamp = None

    def update(self, names, positions, stamp):
        if len(names) != len(positions) or any(names.count(n) != 1 for n in self.names):
            return None
        angles = tuple(positions[names.index(n)] for n in self.names)
        if not all(math.isfinite(v) for v in (*angles, stamp)):
            return None
        if self._stamp is not None and stamp == self._stamp:
            return None
        if self._origin is None or stamp < self._stamp:
            self._origin = angles
        self._stamp = stamp
        return tuple(round((a - start)*self.cpr/(2*math.pi))
                     for a, start in zip(angles, self._origin))


def nearest_range(ranges, minimum, maximum):
    """Range convention: -inf too close, +inf no return; ignore invalid NaNs."""
    valid = [r for r in ranges if not math.isnan(r) and r <= maximum]
    nearest = min(valid, default=math.inf)
    return -math.inf if nearest < minimum else nearest
