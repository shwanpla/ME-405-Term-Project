"""
Calibration Manager for BNO055 IMU.

Handles persistent storage of IMU calibration data by converting
binary calibration blobs to/from hex strings stored in a text file.

Typical usage:
    imu = BNO055(...)
    cm = CalibrationManager(imu)

    # Save current calibration to file
    cm.save_calibration()

    # Load previously saved calibration
    cm.load_calibration()

Notes:
    - Calibration data is stored as a 22-byte blob (BNO055 requirement)
    - File is saved in the local filesystem as "calibration.txt"
"""

import os
try:
    from .IMU_driver import BNO055
except ImportError:
    from IMU_driver import BNO055


class CalibrationManager:
    """
    Manages persistent calibration storage for the BNO055 IMU.

    Provides:
        - Conversion between binary calibration blobs and hex strings
        - Reading/writing calibration data to a file
        - Validation of calibration data length
        - Convenience helpers for existence and deletion
    """

    CALIB_FILE = "calibration.txt"
    CALIB_BLOB_LEN = 22  # Expected length of BNO055 calibration blob

    def __init__(self, imu):
        """
        Initialize the calibration manager.

        :param imu: Instance of a BNO055 IMU driver
        :type imu: BNO055
        """
        self.imu = imu

    # ----------------------------------------------------------------------
    # Conversion Helpers
    # ----------------------------------------------------------------------

    def calib_blob_to_hex(self, blob):
        """
        Convert a raw calibration blob (bytes) into a continuous hex string.

        :param blob: Calibration blob returned by IMU
        :type blob: bytes
        :return: Uppercase hex string representing the calibration data
        :rtype: str
        """
        return ''.join('{:02X}'.format(b) for b in blob)

    def hex_to_calib_blob(self, hex_str):
        """
        Convert a stored hex string back into a calibration blob (bytes).

        Strips whitespace before decoding.

        :param hex_str: Hexadecimal string representation of calibration data
        :type hex_str: str
        :return: Decoded calibration blob
        :rtype: bytes
        """
        hex_str = hex_str.replace(' ', '').replace('\n', '')
        return bytes(int(hex_str[i:i+2], 16) for i in range(0, len(hex_str), 2))

    # ----------------------------------------------------------------------
    # Persistence (Save / Load)
    # ----------------------------------------------------------------------

    def save_calibration(self):
        """
        Read calibration from the IMU and save it to the calibration file.

        :return: True on success, False on failure
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
        Load calibration data from the file and write it into the IMU.

        Validates that the stored calibration contains the required number
        of bytes before writing.

        :return: True if calibration successfully loaded, False otherwise
        :rtype: bool
        """
        try:
            if self.CALIB_FILE not in os.listdir():
                print(f"⚠ {self.CALIB_FILE} not found - calibration required")
                return False

            with open(self.CALIB_FILE, 'r') as f:
                hex_data = f.read()

            blob = self.hex_to_calib_blob(hex_data)

            if len(blob) != self.CALIB_BLOB_LEN:
                print(
                    f"✗ Invalid calibration file "
                    f"(expected {self.CALIB_BLOB_LEN} bytes, got {len(blob)})"
                )
                return False

            self.imu.write_calibration_blob(blob)
            print(f"✓ Calibration loaded from {self.CALIB_FILE}")
            return True

        except Exception as e:
            print(f"✗ Failed to load calibration: {e}")
            return False

    # ----------------------------------------------------------------------
    # Utility Helpers
    # ----------------------------------------------------------------------

    def calibration_exists(self):
        """
        Check whether a valid calibration file is present.

        :return: True if file exists, False otherwise
        :rtype: bool
        """
        return self.CALIB_FILE in os.listdir()

    def delete_calibration(self):
        """
        Delete the stored calibration file.

        :return: True if deleted, False if file not found or delete failed
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
