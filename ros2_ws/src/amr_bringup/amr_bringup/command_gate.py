"""ROS-independent command limits and receipt-time watchdog."""
import math


class CommandGate:
    def __init__(self, linear_limit=0.15, angular_limit=0.6, timeout=0.5):
        for value in (linear_limit, angular_limit, timeout):
            if not math.isfinite(value) or value <= 0:
                raise ValueError('Limits and timeout must be finite and positive')
        self.linear_limit = linear_limit
        self.angular_limit = angular_limit
        self.timeout = timeout
        self._received = None
        self._command = (0.0, 0.0)

    def receive(self, linear, angular, now):
        self._received = now
        if not all(math.isfinite(v) for v in (linear, angular, now)):
            self._received = None
            self._command = (0.0, 0.0)
            return
        self._command = (
            max(-self.linear_limit, min(self.linear_limit, linear)),
            max(-self.angular_limit, min(self.angular_limit, angular)),
        )

    def output(self, now):
        if self._received is None or not 0 <= now - self._received < self.timeout:
            return (0.0, 0.0)
        return self._command
