# main.py
# Main control loop: triple HC-SR04 + MPU6050 yaw + dual motor ramping.
#
# Design goals (per specification):
# 1) Straight line: heading PID with setpoint 0 deg (yaw). Differential correction:
#       left  = base - heading_pid
#       right = base + heading_pid
#    If robot nose drifts right (yaw > 0), error = -yaw < 0; with Kp>0 the PID
#    output goes negative, so left increases and right decreases (correction).
#
# 2) Front ultrasonic: PID tries to regulate forward command so the robot holds
#    a target standoff distance. Additional piecewise rules:
#       - If distance < 15 cm: commanded forward -> 0 (soft stop via Motors ramp)
#       - If distance < 30 cm: apply a smooth proximity factor (linear in distance)
#
# 3) Left/right ultrasonics: if both sides return valid readings, a second PID
#    acts on (left_cm - right_cm) with setpoint 0 to nudge the robot toward the
#    corridor center. The PID output is negated so that when the left side reads
#    farther than the right (robot hugging the right wall), we slow the left
#    wheel and speed the right wheel to steer back to center.
#
# Pins and tuning are passed via RobotConfig (no module-level GPIO constants).


import time
from machine import Pin, I2C

from pid import PID
from motors import Motors
from ultrasonic import Ultrasonic
from gyro import Gyro


class RobotConfig:
    """
    All hardware pins and tuning in one place.
    Instantiate and edit values for your wiring; passed into Robot().
    """

    def __init__(self):
        # L298N (GPIO numbers)
        self.left_in1 = 27
        self.left_in2 = 26
        self.left_en = 25
        self.right_in1 = 33
        self.right_in2 = 32
        self.right_en = 14

        # HC-SR04 (GPIO numbers): front, left, right
        self.trig_f = 4
        self.echo_f = 5
        self.trig_l = 18
        self.echo_l = 19
        self.trig_r = 22
        self.echo_r = 23

        # MPU6050 I2C
        self.i2c_id = 0
        self.pin_sda = 21
        self.pin_scl = 22

        # Motion / control timing
        self.loop_period_ms = 20
        self.motor_pwm_freq = 1000
        self.motor_ramp_rate = 1.5  # normalized speed units per second

        # Cruise forward command (0..1) when far from obstacles
        self.cruise_speed = 0.65

        # Front distance PID: measurement = front_cm, setpoint = desired standoff
        self.front_follow_cm = 45.0
        self.front_pid_kp = 0.04
        self.front_pid_ki = 0.002
        self.front_pid_kd = 0.01

        # Piecewise front safety bands (cm)
        self.front_soft_cm = 30.0
        self.front_stop_cm = 15.0

        # Heading (gyro yaw) PID: measurement = yaw_deg, setpoint = 0
        self.yaw_pid_kp = 0.04
        self.yaw_pid_ki = 0.0
        self.yaw_pid_kd = 0.02
        self.yaw_pid_out_limit = 0.35  # max differential normalized correction

        # Side centering PID: measurement = (left_cm - right_cm), setpoint = 0
        self.side_pid_kp = 0.03
        self.side_pid_ki = 0.0
        self.side_pid_kd = 0.01
        self.side_pid_out_limit = 0.25


class Robot:
    """
    High-level robot controller: owns sensors, PIDs, and Motors.
    """

    def __init__(self, cfg: RobotConfig):
        self.cfg = cfg

        self.motors = Motors(
            cfg.left_in1,
            cfg.left_in2,
            cfg.left_en,
            cfg.right_in1,
            cfg.right_in2,
            cfg.right_en,
            pwm_freq=cfg.motor_pwm_freq,
            ramp_rate_per_s=cfg.motor_ramp_rate,
        )

        self.us_f = Ultrasonic(cfg.trig_f, cfg.echo_f)
        self.us_l = Ultrasonic(cfg.trig_l, cfg.echo_l)
        self.us_r = Ultrasonic(cfg.trig_r, cfg.echo_r)

        self.i2c = I2C(
            cfg.i2c_id,
            scl=Pin(cfg.pin_scl),
            sda=Pin(cfg.pin_sda),
        )
        self.gyro = Gyro(self.i2c, calibrate_samples=400)
        self.gyro.reset_yaw(0.0)

        # Front PID: output is forward command factor roughly in [0, 1]
        self.pid_front = PID(
            cfg.front_pid_kp,
            cfg.front_pid_ki,
            cfg.front_pid_kd,
            setpoint=cfg.front_follow_cm,
            output_limits=(0.0, 1.0),
        )

        # Heading PID: output is left/right differential correction magnitude
        self.pid_yaw = PID(
            cfg.yaw_pid_kp,
            cfg.yaw_pid_ki,
            cfg.yaw_pid_kd,
            setpoint=0.0,
            output_limits=(
                -cfg.yaw_pid_out_limit,
                cfg.yaw_pid_out_limit,
            ),
        )

        # Side balance PID: centers robot when left/right distances differ
        self.pid_side = PID(
            cfg.side_pid_kp,
            cfg.side_pid_ki,
            cfg.side_pid_kd,
            setpoint=0.0,
            output_limits=(
                -cfg.side_pid_out_limit,
                cfg.side_pid_out_limit,
            ),
        )

        self._last_tick = time.ticks_ms()

    def _dt_s(self):
        """Delta time in seconds since last call."""
        now = time.ticks_ms()
        dt_ms = time.ticks_diff(now, self._last_tick)
        self._last_tick = now
        return max(dt_ms, 1) / 1000.0

    def _front_proximity_factor(self, front_cm):
        """
        Smooth slow-down between stop_cm and soft_cm.
        Returns multiplier in [0, 1] applied to forward command.
        """
        if front_cm is None:
            return 1.0
        if front_cm >= self.cfg.front_soft_cm:
            return 1.0
        if front_cm <= self.cfg.front_stop_cm:
            return 0.0
        # Linear ramp: 0 at stop_cm, 1 at soft_cm
        span = self.cfg.front_soft_cm - self.cfg.front_stop_cm
        return max(0.0, min(1.0, (front_cm - self.cfg.front_stop_cm) / span))

    def step(self):
        """
        One control iteration: read sensors, compute PIDs, update motor targets.
        """
        dt = self._dt_s()

        # Sensors
        d_f = self.us_f.read_cm()
        d_l = self.us_l.read_cm()
        d_r = self.us_r.read_cm()

        self.gyro.update(dt)
        yaw = self.gyro.yaw_deg()

        # Default cruise
        base = self.cfg.cruise_speed

        # Front: critical soft stop
        if d_f is not None and d_f < self.cfg.front_stop_cm:
            base = 0.0
            self.pid_front.reset()
        else:
            # Front: distance-keeping PID when we have a valid reading
            if d_f is not None:
                base = self.pid_front.update(d_f, dt)
            # Proximity band: smooth extra reduction from 30cm down to 15cm
            base *= self._front_proximity_factor(d_f)

        # Clamp forward-only autonomous base to [0,1]
        if base < 0.0:
            base = 0.0
        if base > 1.0:
            base = 1.0

        # Heading correction (straight line: yaw -> 0)
        yaw_corr = self.pid_yaw.update(yaw, dt) if base > 0.0 else 0.0
        if base <= 0.0:
            self.pid_yaw.reset()

        # Side centering: only when both side readings are trustworthy
        side_corr = 0.0
        if d_l is not None and d_r is not None and base > 0.0:
            # More space on left than right => robot offset toward right wall:
            # steer left by slowing left / speeding right => apply positive correction
            # to left/right split below; negated PID achieves that sign mapping.
            side_meas = d_l - d_r
            side_corr = -self.pid_side.update(side_meas, dt)
        else:
            self.pid_side.reset()

        # Differential mixing:
        # left  = base - yaw_corr - side_corr
        # right = base + yaw_corr + side_corr
        # (See module docstring for sign rationale.)
        left = base - yaw_corr - side_corr
        right = base + yaw_corr + side_corr

        # Final clamp (allow slight backward if you extend logic; here forward-only)
        left = max(0.0, min(1.0, left))
        right = max(0.0, min(1.0, right))

        self.motors.set_wheel_targets(left, right)
        self.motors.update(dt)

    def run_forever(self):
        """Blocking main loop with periodic timing."""
        period = self.cfg.loop_period_ms
        while True:
            t0 = time.ticks_ms()
            self.step()
            elapsed = time.ticks_diff(time.ticks_ms(), t0)
            if elapsed < period:
                time.sleep_ms(period - elapsed)

    def stop(self):
        """Request soft stop."""
        self.motors.stop()


def main():
    cfg = RobotConfig()
    bot = Robot(cfg)
    try:
        bot.run_forever()
    except KeyboardInterrupt:
        bot.stop()
        # Flush a few ramp cycles so PWM reaches 0 smoothly
        for _ in range(15):
            bot.motors.update(0.02)
            time.sleep_ms(20)


if __name__ == "__main__":
    main()
