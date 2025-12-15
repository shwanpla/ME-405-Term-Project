"""
IMU_V1.py – BNO055 IMU driver (MicroPython, software I2C on B13/B14)
IMPROVED: Better timeout handling and recovery
"""

import pyb
import struct
from micropython import const
from pyb import Pin
from time import sleep_us, sleep_ms

# ── I2C Address ──────────────────────────────────────────────────────────
BNO055_ADDRESS_A = const(0x28)
BNO055_ADDRESS_B = const(0x29)

# ── Registers ────────────────────────────────────────────────────────────
REG_CHIP_ID        = const(0x00)
REG_PAGE_ID        = const(0x07)
REG_UNIT_SEL       = const(0x3B)
REG_OPR_MODE       = const(0x3D)
REG_PWR_MODE       = const(0x3E)
REG_SYS_TRIGGER    = const(0x3F)
REG_TEMP_SOURCE    = const(0x40)
REG_AXIS_MAP_CONFIG= const(0x41)
REG_AXIS_MAP_SIGN  = const(0x42)
REG_CALIB_STAT     = const(0x35)

# Euler (heading, roll, pitch) – little endian, 1 LSB = 1/16 degree
REG_EUL_HEADING_LSB= const(0x1A)
REG_EUL_ROLL_LSB   = const(0x1C)
REG_EUL_PITCH_LSB  = const(0x1E)

# Angular velocity (gyro) – little endian, 1 LSB = 1/16 deg/s
REG_GYR_DATA_X_LSB = const(0x14)
REG_GYR_DATA_Y_LSB = const(0x16)
REG_GYR_DATA_Z_LSB = const(0x18)

# Calibration profile
REG_CALIB_START    = const(0x55)
CALIB_BLOB_LEN     = const(22)

# ── Modes ────────────────────────────────────────────────────────────────
MODE_CONFIG        = const(0x00)
MODE_IMU           = const(0x08)
MODE_COMPASS       = const(0x09)
MODE_M4G           = const(0x0A)
MODE_NDOF_FMC_OFF  = const(0x0B)
MODE_NDOF          = const(0x0C)

PWR_NORMAL         = const(0x00)


class SoftI2C:
    """Software I2C implementation with improved timeout handling"""
    
    def __init__(self, scl_pin, sda_pin, delay_us=50):
        """Initialize with pin names and timing delay"""
        self.scl = Pin(scl_pin, Pin.OUT_OD)
        self.sda = Pin(sda_pin, Pin.OUT_OD)
        self.scl.high()
        self.sda.high()
        self.delay = delay_us  # Increased default for reliability
    
    def _start(self):
        """I2C START condition"""
        self.sda.high()
        self.scl.high()
        sleep_us(self.delay)
        self.sda.low()
        sleep_us(self.delay)
        self.scl.low()
        sleep_us(self.delay)
    
    def _stop(self):
        """I2C STOP condition"""
        self.scl.low()
        sleep_us(self.delay)
        self.sda.low()
        sleep_us(self.delay)
        self.scl.high()
        sleep_us(self.delay)
        self.sda.high()
        sleep_us(self.delay)
    
    def _repeated_start(self):
        """I2C REPEATED START condition"""
        self.scl.low()
        sleep_us(self.delay)
        self.sda.high()
        sleep_us(self.delay)
        self.scl.high()
        sleep_us(self.delay)
        self.sda.low()
        sleep_us(self.delay)
        self.scl.low()
        sleep_us(self.delay)
    
    def _write_bit(self, bit):
        """Write one bit"""
        if bit:
            self.sda.high()
        else:
            self.sda.low()
        sleep_us(self.delay)
        self.scl.high()
        sleep_us(self.delay * 2)
        self.scl.low()
        sleep_us(self.delay)
    
    def _read_bit(self):
        """Read one bit"""
        self.sda.high()
        sleep_us(self.delay)
        self.scl.high()
        sleep_us(self.delay)
        bit = self.sda.value()
        sleep_us(self.delay)
        self.scl.low()
        sleep_us(self.delay)
        return bit
    
    def _write_byte(self, byte):
        """Write one byte, return ACK bit"""
        for i in range(8):
            self._write_bit((byte >> (7 - i)) & 1)
        ack = not self._read_bit()
        return ack
    
    def _read_byte(self, ack):
        """Read one byte, send ACK/NACK"""
        byte = 0
        for i in range(8):
            byte = (byte << 1) | self._read_bit()
        self._write_bit(not ack)
        return byte
    
    def _bus_reset(self):
        """Reset the I2C bus - clock out up to 9 bits"""
        self.sda.high()
        for _ in range(9):
            self.scl.high()
            sleep_us(self.delay)
            self.scl.low()
            sleep_us(self.delay)
        self._stop()
    
    def mem_read(self, nbytes, addr, reg):
        """Read nbytes from register with bus recovery"""
        try:
            self._start()
            if not self._write_byte((addr << 1) | 0):
                self._stop()
                raise OSError("No ACK on address write")
            
            if not self._write_byte(reg):
                self._stop()
                raise OSError("No ACK on register write")
            
            self._repeated_start()
            if not self._write_byte((addr << 1) | 1):
                self._stop()
                raise OSError("No ACK on address read")
            
            data = []
            for i in range(nbytes):
                data.append(self._read_byte(i < nbytes - 1))
            
            self._stop()
            return bytes(data)
        except Exception as e:
            self._bus_reset()
            raise e
    
    def mem_write(self, data, addr, reg):
        """Write data to register with bus recovery"""
        try:
            self._start()
            if not self._write_byte((addr << 1) | 0):
                self._stop()
                raise OSError("No ACK on address write")
            
            if not self._write_byte(reg):
                self._stop()
                raise OSError("No ACK on register write")
            
            for byte in data:
                if not self._write_byte(byte):
                    self._stop()
                    raise OSError("No ACK on data write")
            
            self._stop()
        except Exception as e:
            self._bus_reset()
            raise e


def _le_i16(b):
    """bytes->int16 little-endian"""
    return struct.unpack('<h', b)[0]


class BNO055:
    """Driver for BNO055 using software I2C on arbitrary pins"""

    def __init__(self, scl_pin, sda_pin, addr=BNO055_ADDRESS_A, 
                 autodetect=True, retries=10, retry_delay_ms=20):
        """
        Initialize with SCL and SDA pin names (strings like 'B13', 'B14')
        
        Args:
            scl_pin: SCL pin name (e.g., 'B13')
            sda_pin: SDA pin name (e.g., 'B14')
            addr: I2C address (0x28 or 0x29)
            autodetect: Try both addresses if chip ID doesn't match
            retries: Number of retry attempts for I2C operations
            retry_delay_ms: Delay between retries (increased for stability)
        """
        self.i2c = SoftI2C(scl_pin, sda_pin, delay_us=150)  # Increased from 100
        self.addr = addr
        self._retries = retries
        self._retry_delay_ms = retry_delay_ms
        self._consecutive_errors = 0

        # Confirm device presence
        try:
            chip = self._mem_read(1, REG_CHIP_ID)[0]
        except OSError:
            chip = 0
        
        if chip != 0xA0 and autodetect:
            for candidate in (BNO055_ADDRESS_A, BNO055_ADDRESS_B):
                if candidate != self.addr:
                    self.addr = candidate
                    try:
                        chip = self._mem_read(1, REG_CHIP_ID)[0]
                        if chip == 0xA0:
                            break
                    except OSError:
                        pass
        
        if chip != 0xA0:
            pyb.delay(700)
            chip = self._mem_read(1, REG_CHIP_ID)[0]
            if chip != 0xA0:
                raise RuntimeError("BNO055 not detected or wrong chip ID: 0x%02X" % chip)

        # Ensure PAGE 0
        self._mem_write(bytes([0x00]), REG_PAGE_ID)

        # CONFIG mode
        self._set_mode_internal(MODE_CONFIG)

        # Normal power mode
        self._mem_write(bytes([PWR_NORMAL]), REG_PWR_MODE)
        pyb.delay(10)

        # Exit to NDOF mode
        self.set_mode(MODE_NDOF)

    # ── Public API ────────────────────────────────────────────────────────

    def set_mode(self, fusion_mode):
        """Change the operating mode"""
        self._set_mode_internal(MODE_CONFIG)
        self._set_mode_internal(fusion_mode)

    def get_calibration_status(self):
        """Return dict with calibration status"""
        val = self._mem_read(1, REG_CALIB_STAT)[0]
        sys  = (val >> 6) & 0x03
        gyro = (val >> 4) & 0x03
        accel= (val >> 2) & 0x03
        mag  = (val >> 0) & 0x03
        return {"sys": sys, "gyro": gyro, "accel": accel, "mag": mag}

    def read_calibration_blob(self):
        """Read calibration coefficients (22 bytes) as binary"""
        cur_mode = self._get_mode()
        try:
            self._set_mode_internal(MODE_CONFIG)
            self._mem_write(bytes([0x00]), REG_PAGE_ID)
            blob = self._mem_read(CALIB_BLOB_LEN, REG_CALIB_START)
            return bytes(blob)
        finally:
            self._set_mode_internal(cur_mode)

    def write_calibration_blob(self, blob):
        """Write calibration coefficients (exactly 22 bytes)"""
        if not isinstance(blob, (bytes, bytearray)) or len(blob) != CALIB_BLOB_LEN:
            raise ValueError("Calibration blob must be 22 bytes long")

        cur_mode = self._get_mode()
        try:
            self._set_mode_internal(MODE_CONFIG)
            self._mem_write(bytes([0x00]), REG_PAGE_ID)
            for i in range(CALIB_BLOB_LEN):
                self._mem_write(bytes([blob[i]]), REG_CALIB_START + i)
                pyb.delay(2)
        finally:
            self._set_mode_internal(cur_mode)
        pyb.delay(25)

    def read_euler(self):
        """Read Euler angles (heading, roll, pitch) in degrees"""
        raw = self._mem_read(6, REG_EUL_HEADING_LSB)
        h = _le_i16(raw[0:2]) / 16.0
        r = _le_i16(raw[2:4]) / 16.0
        p = _le_i16(raw[4:6]) / 16.0
        return (h, r, p)

    def get_heading(self):
        """Convenience: return heading (yaw) in degrees"""
        return self.read_euler()[0]

    def read_angular_velocity(self):
        """Read angular velocity (gx, gy, gz) in deg/s"""
        raw = self._mem_read(6, REG_GYR_DATA_X_LSB)
        gx = _le_i16(raw[0:2]) / 16.0
        gy = _le_i16(raw[2:4]) / 16.0
        gz = _le_i16(raw[4:6]) / 16.0
        return (gx, gy, gz)

    def get_yaw_rate(self):
        """Convenience: return gz (yaw rate) in deg/s"""
        return self.read_angular_velocity()[2]

    # ── Low-Level Helpers ─────────────────────────────────────────────────

    def _get_mode(self):
        return self._mem_read(1, REG_OPR_MODE)[0]

    def _set_mode_internal(self, mode):
        """Internal: write OPR_MODE with delays"""
        self._mem_write(bytes([mode]), REG_OPR_MODE)
        if mode == MODE_CONFIG:
            pyb.delay(25)
        else:
            pyb.delay(10)

    def _mem_write(self, data, reg):
        """Write bytes to register with improved retry logic"""
        if not isinstance(data, (bytes, bytearray)):
            data = bytes([data])
        
        last_err = None
        for attempt in range(self._retries):
            try:
                self.i2c.mem_write(data, self.addr, reg)
                self._consecutive_errors = 0  # Reset error counter on success
                return
            except OSError as e:
                last_err = e
                self._consecutive_errors += 1
                # Longer delay on repeated errors
                delay = self._retry_delay_ms * (attempt + 1)
                pyb.delay(min(delay, 100))  # Cap at 100ms
        
        raise last_err

    def _mem_read(self, nbytes, reg):
        """Read bytes from register with improved retry logic"""
        last_err = None
        for attempt in range(self._retries):
            try:
                data = self.i2c.mem_read(nbytes, self.addr, reg)
                self._consecutive_errors = 0  # Reset error counter on success
                return data
            except OSError as e:
                last_err = e
                self._consecutive_errors += 1
                # Longer delay on repeated errors
                delay = self._retry_delay_ms * (attempt + 1)
                pyb.delay(min(delay, 100))  # Cap at 100ms
        
        raise last_err