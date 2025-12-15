Original Approach API Reference
================================

This page documents the API for the original state-space observer implementation preserved in ``src/original-approach/``. This code was not used in the final system but represents the initial sensor fusion approach combining IMU, encoders, and RK4 numerical integration.

.. note::
   This implementation was replaced by the displacement-based odometry approach documented in :doc:`tasks`. See :doc:`analysis` for the technical rationale behind this decision, including IMU communication issues and the pivot to encoder-only estimation.

Observer Module
---------------

State-space dynamics and output equations implementing the 6-state kinematic observer with IMU fusion.

.. automodule:: original_approach.observer_fcn
   :members:
   :undoc-members:
   :show-inheritance:

RK4 Solver
----------

4th-order Runge-Kutta numerical integrator for propagating the state estimate forward in time.

.. automodule:: original_approach.RK4_solver
   :members:
   :undoc-members:
   :show-inheritance:

IMU Driver
----------

Bit-banged I²C implementation for communicating with the BNO055 IMU sensor, including bus recovery and retry logic.

**Class**: ``BNO055``

Provides:
   - Software I2C interface with bus recovery and retry logic
   - BNO055 sensor configuration and operating mode control
   - Euler angle (heading, roll, pitch) and gyroscope readout
   - Calibration blob read/write operations

Hardware:
   - Default I2C Address: 0x28 or 0x29 (configurable)
   - Typical pins: SCL on B13, SDA on B14

Key Methods:
   - ``read_euler()``: Get heading, roll, pitch as tuple
   - ``read_gyro()``: Get angular velocity (x, y, z)
   - ``read_calibration_blob()``: Get 22-byte calibration data
   - ``write_calibration_blob(data)``: Restore calibration
   - ``set_mode(mode)``: Configure operating mode

Calibration Manager
-------------------

Persistent storage and retrieval of BNO055 calibration blob to avoid repeated manual calibration across power cycles.

**Class**: ``CalibrationManager``

Provides:
   - Automatic calibration save/load to filesystem
   - Conversion between binary blob and hex string format
   - File-based persistence of calibration state

Typical usage:
   .. code-block:: python

      imu = BNO055(scl_pin, sda_pin)
      cm = CalibrationManager(imu)
      cm.load_calibration()  # Load from file if available
      cm.save_calibration()  # Save after calibration complete

Key Methods:
   - ``load_calibration()``: Restore calibration from ``calibration.txt``
   - ``save_calibration()``: Write calibration blob to file
   - ``is_calibrated()``: Check if device reports calibrated state

IMU Handler Task
----------------

Startup finite state machine managing IMU initialization, calibration loading, manual calibration, and readiness signaling.

**Function**: ``calibration_task_fun(shares)``

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

.. automodule:: original_approach.navigation_original
   :members:
   :undoc-members:
   :show-inheritance:

Main (Original)
---------------

Task coordination and cooperative scheduler setup for the original approach.

.. automodule:: original_approach.main_original
   :members:
   :undoc-members:
   :show-inheritance:
