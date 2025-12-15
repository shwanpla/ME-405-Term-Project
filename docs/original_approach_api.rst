Original Approach API Reference
================================

This page documents the API for the original state-space observer implementation preserved in ``src/`` with the ``defunct_`` prefix. This code was not used in the final system but represents the initial sensor fusion approach combining IMU, encoders, and RK4 numerical integration.

.. note::
   This implementation was replaced by the displacement-based odometry approach documented in :doc:`tasks`. See :doc:`analysis` for the technical rationale behind this decision, including IMU communication issues and the pivot to encoder-only estimation.

IMU Driver
----------

The BNO055 IMU driver provides 9-axis absolute orientation sensing with software I2C implementation. This driver includes automatic calibration management, bus recovery, and retry logic for robust operation.

.. automodule:: defunct_IMU_driver
   :members:
   :undoc-members:
   :show-inheritance:

SoftI2C Class
"""""""""""""

.. py:class:: SoftI2C(scl_pin, sda_pin, delay_us=50)

   Software I2C implementation with improved timeout handling and bus recovery.

   :param scl_pin: SCL pin name (e.g., 'B13')
   :type scl_pin: str
   :param sda_pin: SDA pin name (e.g., 'B14')
   :type sda_pin: str
   :param delay_us: Bit timing delay in microseconds (default: 50)
   :type delay_us: int

.. py:method:: mem_read(nbytes, addr, reg)

   Read multiple bytes from a device register with automatic bus recovery.

   :param nbytes: Number of bytes to read
   :type nbytes: int
   :param addr: I2C device address (7-bit)
   :type addr: int
   :param reg: Register address to read from
   :type reg: int
   :return: Data read from register
   :rtype: bytes
   :raises OSError: If communication fails after bus recovery attempt

.. py:method:: mem_write(data, addr, reg)

   Write multiple bytes to a device register with automatic bus recovery.

   :param data: Data to write
   :type data: bytes
   :param addr: I2C device address (7-bit)
   :type addr: int
   :param reg: Register address to write to
   :type reg: int
   :raises OSError: If communication fails after bus recovery attempt

BNO055 Class
""""""""""""

.. py:class:: BNO055(scl_pin, sda_pin, addr=0x28, autodetect=True, retries=10, retry_delay_ms=20)

   Driver for BNO055 9-axis absolute orientation sensor using software I2C.

   :param scl_pin: SCL pin name (e.g., 'B13')
   :type scl_pin: str
   :param sda_pin: SDA pin name (e.g., 'B14')
   :type sda_pin: str
   :param addr: I2C address (0x28 or 0x29, default: 0x28)
   :type addr: int
   :param autodetect: Try both addresses if chip ID doesn't match (default: True)
   :type autodetect: bool
   :param retries: Number of retry attempts for I2C operations (default: 10)
   :type retries: int
   :param retry_delay_ms: Delay between retries in milliseconds (default: 20)
   :type retry_delay_ms: int
   :raises RuntimeError: If BNO055 not detected or wrong chip ID

.. py:method:: set_mode(fusion_mode)

   Change the BNO055 operating mode.

   :param fusion_mode: Operating mode constant (MODE_NDOF, MODE_IMU, etc.)
   :type fusion_mode: int

.. py:method:: get_calibration_status()

   Get calibration status for all sensor subsystems.

   :return: Calibration status with keys 'sys', 'gyro', 'accel', 'mag'. Each value ranges from 0 (uncalibrated) to 3 (fully calibrated).
   :rtype: dict

   **Example**::

      >>> status = imu.get_calibration_status()
      >>> print(status)
      {'sys': 3, 'gyro': 3, 'accel': 3, 'mag': 3}

.. py:method:: read_calibration_blob()

   Read calibration coefficients from BNO055 as binary blob.

   Temporarily switches to CONFIG mode, reads 22-byte calibration data, then restores previous operating mode.

   :return: 22-byte calibration blob
   :rtype: bytes

.. py:method:: write_calibration_blob(blob)

   Write calibration coefficients to BNO055 from binary blob.

   Temporarily switches to CONFIG mode, writes 22-byte calibration data, then restores previous operating mode.

   :param blob: Exactly 22 bytes of calibration data
   :type blob: bytes
   :raises ValueError: If blob is not exactly 22 bytes

.. py:method:: read_euler()

   Read Euler angles from BNO055.

   :return: (heading, roll, pitch) in degrees
   :rtype: tuple

   - **heading**: 0-360° (yaw, compass heading)
   - **roll**: -180 to +180° (rotation about X-axis)
   - **pitch**: -90 to +90° (rotation about Y-axis)

.. py:method:: get_heading()

   Get heading (yaw) angle in degrees.

   Convenience function that extracts only the heading from Euler angles.

   :return: Heading in degrees (0-360°, 0 = North in NDOF mode)
   :rtype: float

.. py:method:: read_angular_velocity()

   Read angular velocity (gyro rates) from BNO055.

   :return: (gx, gy, gz) angular velocities in degrees per second
   :rtype: tuple

   - **gx**: Roll rate (rotation about X-axis)
   - **gy**: Pitch rate (rotation about Y-axis)
   - **gz**: Yaw rate (rotation about Z-axis)

.. py:method:: get_yaw_rate()

   Get yaw rate (angular velocity about Z-axis) in degrees per second.

   Convenience function that extracts only the yaw rate from angular velocities.

   :return: Yaw rate in degrees per second (positive = counter-clockwise)
   :rtype: float

IMU Hardware Notes
""""""""""""""""""

- **Sensor**: BNO055 9-axis absolute orientation sensor
- **Interface**: Software I2C on pins B13 (SCL) and B14 (SDA)
- **Reset Pin**: B15 (active low)
- **I2C Address**: 0x28 (default) or 0x29 (alternate)
- **Operating Mode**: NDOF (9-DOF fusion with magnetometer)
- **Heading Range**: 0-360 degrees (0 = North in NDOF mode)
- **Angular Rates**: Degrees per second
- **Calibration Status**: 0 (uncalibrated) to 3 (fully calibrated)
- **Software I2C Delay**: 150 µs (tuned for reliability)

Calibration Manager
-------------------

.. automodule:: defunct_CalibrationManager
   :members:
   :undoc-members:
   :show-inheritance:

IMU Handler Task
----------------

.. automodule:: defunct_IMU_handler
   :members:
   :undoc-members:
   :show-inheritance:

Implements a 4-state startup sequence:

1. **STATE_INIT**: Check for existing calibration file
2. **STATE_LOAD**: Load calibration if available
3. **STATE_MANUAL_CAL**: User-guided manual calibration (if needed)
4. **STATE_READY**: Calibration complete, signal readiness via share

Also provides:

**Function**: ``imu_monitor_task_fun(shares)``

Continuous task that:
   - Reads Euler angles from IMU at periodic intervals
   - Computes continuous (unwrapped) heading from incremental yaw
   - Outputs heading and yaw rate via task shares for navigation/control

Shares:
   - ``imu_heading_share``: Current heading (degrees, unwrapped)
   - ``imu_yaw_rate_share``: Angular velocity (deg/s)
   - ``imu_ready_share``: Calibration complete flag

Navigation (Original)
---------------------

Position-based navigation logic using global X, Y coordinates from the observer and IMU heading for state transitions.

.. automodule:: defunct_navigation
   :members:
   :undoc-members:
   :show-inheritance:

Main (Original)
---------------

.. automodule:: defunct_main
