"""
Calibration manager for BNO055 IMU sensor with persistent storage.
Handles saving and loading calibration coefficients to/from calibration.txt file.

Hardware:
    - IMU Sensor: BNO055 9-DOF absolute orientation sensor
    - I2C Communication: I2C interface for sensor communication

Notes:
    - Calibration blob is 22 bytes containing all calibration coefficients
    - Stored as hexadecimal string in calibration.txt
    - Calibration must be performed when no valid calibration file exists
"""

import os
from defunct_IMU_driver import BNO055


class CalibrationManager:
    """
    Manages BNO055 IMU calibration persistence with file-based storage.
    Provides methods to save, load, and verify calibration data.
    """

    CALIB_FILE = "calibration.txt"
    CALIB_BLOB_LEN = 22

    def __init__(self, imu):
        """
        Initialize calibration manager with BNO055 IMU instance.

        :param imu: BNO055 IMU driver instance
        :type imu: BNO055
        """
        self.imu = imu
    
    def calib_blob_to_hex(self, blob):
        """
        Convert calibration blob (bytes) to hexadecimal string representation.

        :param blob: Raw calibration data bytes from IMU
        :type blob: bytes
        :return: Hexadecimal string representation of calibration data
        :rtype: str
        """
        return ''.join('{:02X}'.format(b) for b in blob)

    def hex_to_calib_blob(self, hex_str):
        """
        Convert hexadecimal string back to calibration blob (bytes).

        :param hex_str: Hexadecimal string from calibration file
        :type hex_str: str
        :return: Calibration data as bytes
        :rtype: bytes
        """
        # Remove whitespace and convert pairs of hex chars to bytes
        hex_str = hex_str.replace(' ', '').replace('\n', '')
        return bytes(int(hex_str[i:i+2], 16) for i in range(0, len(hex_str), 2))
    
    def save_calibration(self):
        """
        Read calibration coefficients from IMU and save to persistent storage.

        Reads the 22-byte calibration blob from the BNO055 sensor and writes
        it to calibration.txt as a hexadecimal string.

        :return: True if calibration saved successfully, False otherwise
        :rtype: bool
        """
        try:
            blob = self.imu.read_calibration_blob()
            hex_data = self.calib_blob_to_hex(blob)

            with open(self.CALIB_FILE, 'w') as f:
                f.write(hex_data)

            print(f"✓ Calibration saved to {self.CALIB_FILE}")
            print(f"  Data: {hex_data}")
            return True
        except Exception as e:
            print(f"✗ Failed to save calibration: {e}")
            return False
    
    def load_calibration(self):
        """
        Load calibration coefficients from file and write to IMU sensor.

        Reads calibration data from calibration.txt, validates the format and
        length, then writes the 22-byte calibration blob to the BNO055 sensor.

        :return: True if calibration loaded successfully, False otherwise
        :rtype: bool
        """
        try:
            # Check if file exists
            if self.CALIB_FILE not in os.listdir():
                print(f"⚠ {self.CALIB_FILE} not found - calibration required")
                return False

            # Read file
            with open(self.CALIB_FILE, 'r') as f:
                hex_data = f.read()

            # Convert and validate
            blob = self.hex_to_calib_blob(hex_data)
            if len(blob) != self.CALIB_BLOB_LEN:
                print(f"✗ Invalid calibration file (expected {self.CALIB_BLOB_LEN} bytes, got {len(blob)})")
                return False

            # Write to IMU
            self.imu.write_calibration_blob(blob)
            print(f"✓ Calibration loaded from {self.CALIB_FILE}")
            return True
        except Exception as e:
            print(f"✗ Failed to load calibration: {e}")
            return False
    
    def calibration_exists(self):
        """
        Check if calibration file exists in the current directory.

        :return: True if calibration.txt exists, False otherwise
        :rtype: bool
        """
        return self.CALIB_FILE in os.listdir()

    def delete_calibration(self):
        """
        Delete stored calibration file (for testing purposes).

        Removes calibration.txt file from the filesystem if it exists.
        Used to force recalibration during testing and development.

        :return: True if file deleted successfully, False if file not found
        :rtype: bool
        """
        try:
            if self.CALIB_FILE in os.listdir():
                os.remove(self.CALIB_FILE)
                print(f"✓ Deleted {self.CALIB_FILE}")
                return True
            return False
        except Exception as e:
            print(f"✗ Failed to delete calibration: {e}")
            return False