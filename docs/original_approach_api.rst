Original Approach API Reference
================================

This page documents the API for the original state-space observer implementation preserved in ``src/original-approach/``. This code was not used in the final system but represents the initial sensor fusion approach combining IMU, encoders, and RK4 numerical integration.

.. note::
   This implementation was replaced by the displacement-based odometry approach documented in :doc:`tasks`. See :doc:`analysis` for the technical rationale behind this decision, including IMU communication issues and the pivot to encoder-only estimation.

Observer Module
---------------

State-space dynamics and output equations implementing the 6-state kinematic observer with IMU fusion.

.. automodule:: original-approach.observer_fcn
   :members:
   :undoc-members:
   :show-inheritance:

RK4 Solver
----------

4th-order Runge-Kutta numerical integrator for propagating the state estimate forward in time.

.. automodule:: original-approach.RK4_solver
   :members:
   :undoc-members:
   :show-inheritance:

IMU Driver
----------

Bit-banged I²C implementation for communicating with the BNO055 IMU sensor, including bus recovery and retry logic.

.. automodule:: original-approach.IMU_driver
   :members:
   :undoc-members:
   :show-inheritance:

Calibration Manager
-------------------

Persistent storage and retrieval of BNO055 calibration blob to avoid repeated manual calibration across power cycles.

.. automodule:: original-approach.CalibrationManager
   :members:
   :undoc-members:
   :show-inheritance:

IMU Handler Task
----------------

Startup finite state machine managing IMU initialization, calibration loading, manual calibration, and readiness signaling.

.. automodule:: original-approach.IMU_handler
   :members:
   :undoc-members:
   :show-inheritance:

Navigation (Original)
---------------------

Position-based navigation logic using global X, Y coordinates from the observer and IMU heading for state transitions.

.. automodule:: original-approach.navigation_original
   :members:
   :undoc-members:
   :show-inheritance:

Main (Original)
---------------

Task coordination and cooperative scheduler setup for the original approach.

.. automodule:: original-approach.main_original
   :members:
   :undoc-members:
   :show-inheritance:
