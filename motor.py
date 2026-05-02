# motor.py
# Single L298N channel: two direction pins + one PWM enable.
#
# Speed command is normalized to [-1.0, 1.0]:
#   0   -> motor off (PWM duty = 0)
#   >0  -> forward (IN1/IN2 pattern one way)
#   <0  -> reverse (IN1/IN2 pattern swapped)
#
# Deadzone compensation (Yellow TT motors):
# -------------------------------------------
# These motors often need a minimum PWM before they overcome static friction.
# This implementation uses a *duty offset* of 350 (on a 0..1023 scale):
#
#   Let m = abs(speed) in (0, 1].
#   duty = DEADZONE_OFFSET + m * (PWM_MAX - DEADZONE_OFFSET)
#
# So at the smallest non-zero |speed|, duty is still ~350, which helps the
# motor start moving. As |speed| -> 1, duty -> 1023 (full).
# At speed == 0, duty is forced to 0 (no creep from offset).


from machine import Pin, PWM


class Motor:
    PWM_MAX = 1023
    # Minimum non-zero PWM duty for yellow TT motors (per project spec)
    DEADZONE_OFFSET = 350

    def __init__(self, pin_in1, pin_in2, pin_en, pwm_freq=1000):
        """
        pin_in1, pin_in2, pin_en: GPIO numbers (integers).
        pwm_freq: PWM frequency in Hz for the enable pin.
        """
        self._in1 = Pin(int(pin_in1), Pin.OUT, value=0)
        self._in2 = Pin(int(pin_in2), Pin.OUT, value=0)
        self._en = PWM(Pin(int(pin_en)), freq=int(pwm_freq))
        self._en.duty(0)

    def _speed_to_duty(self, speed):
        """
        Map normalized speed [-1..1] to PWM duty [0..1023] with deadzone scaling.

        For magnitude m = abs(speed):
          m == 0     -> duty 0 (motor off)
          0 < m <= 1 -> duty scales linearly from DEADZONE_OFFSET .. PWM_MAX
        """
        m = abs(speed)
        if m <= 0.0:
            return 0
        # Clamp magnitude
        if m > 1.0:
            m = 1.0
        span = self.PWM_MAX - self.DEADZONE_OFFSET
        duty = self.DEADZONE_OFFSET + int(m * span + 0.5)
        if duty > self.PWM_MAX:
            duty = self.PWM_MAX
        return duty

    def drive(self, speed):
        """
        Apply direction + PWM for one update cycle.

        speed: float in [-1.0, 1.0]
        """
        if speed > 1.0:
            speed = 1.0
        elif speed < -1.0:
            speed = -1.0

        duty = self._speed_to_duty(speed)

        if duty <= 0:
            self._in1.off()
            self._in2.off()
            self._en.duty(0)
            return

        if speed > 0.0:
            # Forward pattern (swap if your wiring is inverted)
            self._in1.on()
            self._in2.off()
        else:
            self._in1.off()
            self._in2.on()

        self._en.duty(duty)

    def stop_hard(self):
        """Coast/brake off immediately (both lows + PWM 0)."""
        self._in1.off()
        self._in2.off()
        self._en.duty(0)
