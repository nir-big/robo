# pid.py
# Generic PID controller for MicroPython (ESP32).
#
# Discrete-time approximation (backward Euler on integral):
#   error = setpoint - measurement
#   P = Kp * error
#   integral += error * dt
#   I = Ki * integral
#   D = Kd * (error - last_error) / dt
#   output = P + I + D
#
# Output is clamped to [output_min, output_max]. A simple anti-windup clamps
# the *integral state* if the sum tries to push past the output limits.


class PID:
    def __init__(self, kp, ki, kd, setpoint=0.0, output_limits=(None, None)):
        """
        kp, ki, kd: PID gains (floats).
        setpoint: desired value for the measurement.
        output_limits: (min, max) tuple; None means no limit on that side.
        """
        self.kp = float(kp)
        self.ki = float(ki)
        self.kd = float(kd)
        self.setpoint = float(setpoint)
        self.output_limits = output_limits

        self._integral = 0.0
        self._last_error = 0.0
        self._first = True

    def reset(self):
        """Clear integral and derivative state."""
        self._integral = 0.0
        self._last_error = 0.0
        self._first = True

    def set_setpoint(self, value):
        self.setpoint = float(value)

    def _clamp_output(self, u):
        lo, hi = self.output_limits
        if lo is not None and u < lo:
            return lo
        if hi is not None and u > hi:
            return hi
        return u

    def update(self, measurement, dt):
        """
        One PID step.

        measurement: current process variable
        dt: time step in seconds (must be > 0 for meaningful D term)

        Returns: control output (clamped to output_limits)
        """
        if dt <= 0:
            # Degenerate dt: proportional-only fallback
            e = self.setpoint - float(measurement)
            return self._clamp_output(self.kp * e)

        e = self.setpoint - float(measurement)

        # Integral accumulation (pre anti-windup)
        self._integral += e * dt

        p_term = self.kp * e
        i_term = self.ki * self._integral

        if self._first:
            d_term = 0.0
            self._first = False
        else:
            d_term = self.kd * (e - self._last_error) / dt

        self._last_error = e

        u = p_term + i_term + d_term
        u_sat = self._clamp_output(u)

        # Anti-windup: if output saturates, shrink integral so I term matches saturation
        if self.ki != 0.0 and u != u_sat:
            # Desired integral contribution that would make u == u_sat (holding P,D fixed)
            i_desired = u_sat - p_term - d_term
            self._integral = i_desired / self.ki

        return u_sat
