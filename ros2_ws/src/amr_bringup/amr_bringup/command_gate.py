"""ROS-independent command limits and receipt-time watchdog."""
import math


class CommandGate:
    def __init__(self, linear_limit=0.15, angular_limit=0.6, timeout=0.5,
                 wheel_radius=0.0325, wheel_separation=0.220, max_wheel_rpm=60.0,
                 require_bumpers=False, bumper_timeout=0.5):
        for value in (linear_limit, angular_limit, timeout, wheel_radius,
                      wheel_separation, max_wheel_rpm, bumper_timeout):
            if not math.isfinite(value) or value <= 0:
                raise ValueError('Limits and timeout must be finite and positive')
        self.linear_limit = linear_limit
        self.angular_limit = angular_limit
        self.timeout = timeout
        self.wheel_separation = wheel_separation
        self.wheel_speed_limit = wheel_radius * max_wheel_rpm * 2*math.pi / 60
        self.require_bumpers = require_bumpers
        self.bumper_timeout = bumper_timeout
        self._bumpers = {name: None for name in ('left', 'center', 'right')}
        self._received = None
        self._command = (0.0, 0.0)

    def receive(self, linear, angular, now):
        self._received = now
        if (not all(math.isfinite(v) for v in (linear, angular, now))
                or not self._bumpers_clear(now)):
            self._received = None
            self._command = (0.0, 0.0)
            return
        self._command = (
            max(-self.linear_limit, min(self.linear_limit, linear)),
            max(-self.angular_limit, min(self.angular_limit, angular)),
        )
        v, w = self._command
        fastest = max(abs(v - w*self.wheel_separation/2),
                      abs(v + w*self.wheel_separation/2))
        if fastest > self.wheel_speed_limit:
            scale = self.wheel_speed_limit / fastest
            self._command = (v*scale, w*scale)

    def receive_bumper(self, name, pressed, now):
        self._bumpers[name] = (pressed, now)
        if pressed:
            self._received = None

    def _bumpers_clear(self, now):
        if not self.require_bumpers:
            return True
        return all(sample is not None and not sample[0]
                   and 0 <= now - sample[1] < self.bumper_timeout
                   for sample in self._bumpers.values())

    def output(self, now):
        if not self._bumpers_clear(now):
            self._received = None
            return (0.0, 0.0)
        if self._received is None or not 0 <= now - self._received < self.timeout:
            return (0.0, 0.0)
        return self._command
