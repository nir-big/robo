# gyro.py
# MPU6050 on I2C: integrate gyroscope Z rate to track yaw heading in degrees.
#
# Steps:
# 1) Wake MPU6050, select gyro full-scale (here ±250 dps -> 131 LSB per dps).
# 2) Calibrate stationary Z bias by averaging many raw samples.
# 3) Each update(dt): read gyro_z, subtract bias, convert to deg/s, integrate:
#       yaw_deg += gz_dps * dt
#
# Note: yaw drifts without magnetometer fusion; for short straight segments this
# is usually acceptable if Ki is modest in the heading PID.


from machine import I2C
import time


class Gyro:
    _ADDR = 0x68
    _REG_PWR_MGMT_1 = 0x6B
    _REG_GYRO_CONFIG = 0x1B
    _REG_GYRO_ZOUT_H = 0x47

    # ±250 dps sensitivity (datasheet): 131 LSB / (deg/s)
    _GYRO_LSB_PER_DPS = 131.0

    def __init__(self, i2c, addr=_ADDR, calibrate_samples=400):
        """
        i2c: machine.I2C instance (constructed by caller with chosen pins).
        addr: MPU6050 I2C address (0x68 default AD0 low).
        calibrate_samples: stationary averaging count for Z bias.
        """
        self._i2c = i2c
        self._addr = int(addr)

        # Wake device (clear sleep)
        self._write_reg(self._REG_PWR_MGMT_1, b"\x00")
        time.sleep_ms(50)
        # Gyro FS = ±250 dps
        self._write_reg(self._REG_GYRO_CONFIG, b"\x00")
        time.sleep_ms(20)

        self._bias_z = self._calibrate_gyro_z(calibrate_samples)
        self._yaw_deg = 0.0

    def _write_reg(self, reg, buf):
        self._i2c.writeto_mem(self._addr, reg, buf)

    def _read_reg(self, reg, n):
        return self._i2c.readfrom_mem(self._addr, reg, n)

    def _read_i16(self, reg_h):
        b = self._read_reg(reg_h, 2)
        v = (b[0] << 8) | b[1]
        if v & 0x8000:
            v -= 65536
        return v

    def _calibrate_gyro_z(self, samples):
        total = 0
        n = max(10, int(samples))
        for _ in range(n):
            total += self._read_i16(self._REG_GYRO_ZOUT_H)
            time.sleep_ms(2)
        return total / float(n)

    def read_gyro_z_dps(self):
        """Z-axis angular rate in degrees per second (bias removed)."""
        raw = self._read_i16(self._REG_GYRO_ZOUT_H)
        return (raw - self._bias_z) / self._GYRO_LSB_PER_DPS

    def update(self, dt):
        """Integrate gyro Z to update yaw estimate (degrees)."""
        if dt <= 0:
            return
        gz = self.read_gyro_z_dps()
        self._yaw_deg += gz * dt

    def reset_yaw(self, value_deg=0.0):
        """Set current yaw estimate (e.g. align to 0 at start of straight run)."""
        self._yaw_deg = float(value_deg)

    def yaw_deg(self):
        """Current yaw heading in degrees (integrated Z)."""
        return self._yaw_deg
