# motors.py
# Differential drive using two Motor instances (L298N + raw Pin/PWM).
#
# Soft start / soft braking:
# -------------------------
# High-level commands set *targets* (left_target, right_target) in [-1..1].
# Each control cycle, the *applied* speeds move toward those targets by at most
#   max_step = RAMP_RATE_PER_S * dt
# This is linear ramping (constant slew rate), which limits jerk and protects
# the yellow TT gearbox from sudden torque steps.


from motor import Motor


class Motors:
    def __init__(
        self,
        left_in1,
        left_in2,
        left_en,
        right_in1,
        right_in2,
        right_en,
        pwm_freq=1000,
        ramp_rate_per_s=1.2,
    ):
        """
        All *_in1, *_in2, *_en arguments are GPIO numbers (ints).

        ramp_rate_per_s: maximum change in normalized speed per second for each side.
                         Example: 1.2 means it takes ~0.83 s to go from 0 to 1.0.
        """
        self.left = Motor(left_in1, left_in2, left_en, pwm_freq=pwm_freq)
        self.right = Motor(right_in1, right_in2, right_en, pwm_freq=pwm_freq)

        self._lt = 0.0
        self._rt = 0.0
        self._la = 0.0
        self._ra = 0.0
        self._ramp = float(ramp_rate_per_s)

    def set_ramp_rate(self, ramp_rate_per_s):
        self._ramp = float(ramp_rate_per_s)

    def _ramp_toward(self, current, target, dt):
        """Move current toward target, limited by linear slew rate."""
        if dt <= 0:
            return current
        max_step = self._ramp * dt
        delta = target - current
        if delta > max_step:
            return current + max_step
        if delta < -max_step:
            return current - max_step
        return target

    def set_wheel_targets(self, left_speed, right_speed):
        """
        Set commanded wheel targets in [-1..1] (soft ramp applies in update()).
        """
        self._lt = max(-1.0, min(1.0, float(left_speed)))
        self._rt = max(-1.0, min(1.0, float(right_speed)))

    def forward(self, speed):
        s = max(0.0, min(1.0, float(speed)))
        self.set_wheel_targets(s, s)

    def backward(self, speed):
        s = max(0.0, min(1.0, float(speed)))
        self.set_wheel_targets(-s, -s)

    def turn_left(self, speed):
        """
        Gentle left turn: reduce left wheel, keep right wheel higher (both forward).
        Not tank-drive (no full reverse on inner wheel by default).
        """
        s = max(0.0, min(1.0, float(speed)))
        inner = 0.35  # inner track factor (tune for your chassis)
        self.set_wheel_targets(s * inner, s)

    def turn_right(self, speed):
        s = max(0.0, min(1.0, float(speed)))
        inner = 0.35
        self.set_wheel_targets(s, s * inner)

    def stop(self):
        """Soft stop: ramp targets to zero; update() will bring motors down smoothly."""
        self.set_wheel_targets(0.0, 0.0)

    def update(self, dt):
        """
        Apply soft start/braking and write PWM to hardware.

        Call this at a fixed rate from your main loop (e.g. every 10–20 ms).
        """
        self._la = self._ramp_toward(self._la, self._lt, dt)
        self._ra = self._ramp_toward(self._ra, self._rt, dt)
        self.left.drive(self._la)
        self.right.drive(self._ra)

    def applied_speeds(self):
        """Return (left_applied, right_applied) normalized speeds after ramping."""
        return (self._la, self._ra)
