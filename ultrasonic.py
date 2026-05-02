# ultrasonic.py
# HC-SR04 driver using machine.time_pulse_us with a timeout to avoid long blocking.
#
# The echo pulse width (microseconds) relates to range by:
#   distance_cm = (pulse_us * speed_of_sound_cm_per_us) / 2
# with speed_of_sound ≈ 0.0343 cm/us at ~20°C.
#
# time_pulse_us(pin, pulse_level, timeout_us) waits up to timeout_us for the pulse;
# if it times out, MicroPython raises OSError on some ports or returns negative;
# we normalize failures to None.


from machine import Pin, time_pulse_us
import time


class Ultrasonic:
    # Approximate speed of sound in cm/us (used for distance conversion)
    CM_PER_US = 0.0343

    def __init__(self, pin_trig, pin_echo, max_distance_cm=400):
        """
        pin_trig, pin_echo: GPIO numbers (ints).

        max_distance_cm: used only to compute a safe echo timeout in microseconds.
        """
        self._trig = Pin(int(pin_trig), Pin.OUT, value=0)
        self._echo = Pin(int(pin_echo), Pin.IN)
        self._max_cm = float(max_distance_cm)
        # Round-trip path for max distance -> timeout for echo HIGH pulse
        self._timeout_us = int((self._max_cm * 2.0) / self.CM_PER_US) + 1000

    def read_cm(self):
        """
        Return distance in centimeters (float), or None if no valid echo (timeout).

        Uses a short trigger pulse; does not busy-wait beyond time_pulse_us timeout.
        """
        # Ensure clean low before trigger
        self._trig.off()
        time.sleep_us(5)

        # 10 us trigger pulse (HC-SR04 spec)
        self._trig.on()
        time.sleep_us(10)
        self._trig.off()

        try:
            pulse = time_pulse_us(self._echo, 1, self._timeout_us)
        except OSError:
            return None

        # Some firmware variants return negative on timeout
        if pulse <= 0:
            return None

        # Convert to one-way distance in cm
        dist = (pulse * self.CM_PER_US) / 2.0
        if dist <= 0 or dist > self._max_cm:
            return None
        return dist
