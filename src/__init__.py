"""
Original state-space observer approach with IMU fusion.

This package contains the original navigation implementation that combined
encoder-derived wheel velocities with IMU-derived absolute heading using
RK4 numerical integration.

Note: This implementation was replaced by the displacement-based odometry
approach. See the Analysis documentation for details.
"""
# Import main classes for easier discovery by autodoc
try:
    from .defunct_IMU_driver import BNO055, SoftI2C
    from .defunct_CalibrationManager import CalibrationManager
    from .defunct_observer_fcn import observer
    from .defunct_RK4_solver import RK4
except ImportError:
    # Allow graceful failure if imports fail (e.g., during documentation build)
    pass

__all__ = [
    'BNO055',
    'SoftI2C',
    'defunct_CalibrationManager',
    'observer',
    'RK4',
    'calibration_task_fun',
    'imu_monitor_task_fun',
    'navigation_task_fun_original',
]