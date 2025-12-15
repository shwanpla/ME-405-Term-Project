Original Approach API Reference
================================

This page documents the API for the original state-space observer implementation preserved in ``src/`` with the ``defunct_`` prefix. This code was not used in the final system but represents the initial sensor fusion approach combining IMU, encoders, and RK4 numerical integration.

.. note::
   This implementation was replaced by the displacement-based odometry approach documented in :doc:`tasks`. See :doc:`analysis` for the technical rationale behind this decision, including IMU communication issues and the pivot to encoder-only estimation.

IMU Driver
----------

.. automodule:: defunct_IMU_driver
   :members:
   :undoc-members:
   :show-inheritance:

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
