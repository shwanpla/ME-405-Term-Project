"""
Observer task for kinematic state estimation using RK4 integration.

Maintains and updates a 6-state vector:
    x = [X, Y, Theta, s, Omega_L, Omega_R]^T

Where:
    X, Y      : Global position in meters
    Theta     : Heading in radians
    s         : Arc length traveled (meters)
    Omega_L/R : Wheel linear velocities (m/s)

The observer:
    - Uses measured wheel speeds (from encoders) converted to m/s
    - Uses IMU heading (with offset and low-pass filtering) for position update
    - Integrates the kinematics with an RK4 solver at 100 ms horizon, 10 ms step
    - Publishes X, Y, and heading to shared variables for navigation
"""

from ulab import numpy as np
from math import pi, cos, sin
try:
    from .RK4_solver import RK4_solver
except ImportError:
    from RK4_solver import RK4_solver


def normalize_angle_deg(angle_deg):
    """
    Normalize an angle in degrees to the [-180, 180] range.

    :param angle_deg: Angle to normalize, in degrees
    :type angle_deg: float
    :return: Normalized angle in [-180, 180] degrees
    :rtype: float
    """
    while angle_deg > 180.0:
        angle_deg -= 360.0
    while angle_deg < -180.0:
        angle_deg += 360.0
    return angle_deg


def observer_task_fcn(shares):
    """
    Observer task function (generator) for cooperative multitasking.

    Integrates the robot's full kinematic state using an RK4 solver, combining:
        - Encoder-derived wheel velocities (ticks/us -> m/s)
        - IMU heading (with offset and low-pass filtering) for position update

    The state vector is:
        x = [X, Y, Theta, s, Omega_L, Omega_R]^T

    where:
        X, Y      : Global position (meters)
        Theta     : Heading (radians)
        s         : Arc length traveled (meters)
        Omega_L/R : Wheel linear velocities (m/s)

    Shares (in order of tuple):
        meas_heading_share       : Measured IMU heading (deg)
        meas_yaw_rate_share      : Measured IMU yaw rate (deg/s)
        left_enc_pos             : Left encoder position (unused here)
        right_enc_pos            : Right encoder position (unused here)
        left_enc_speed           : Left encoder speed (ticks/us)
        right_enc_speed          : Right encoder speed (ticks/us)
        obs_heading_share        : Output observer heading (deg)
        obs_yaw_rate_share       : Output observer yaw rate (deg/s)
        left_set_point           : Left motor effort setpoint (unused here)
        right_set_point          : Right motor effort setpoint (unused here)
        observer_calibration_flg : Flag to trigger observer initialization (1=start)
        big_X_share              : Output X position (mm)
        big_Y_share              : Output Y position (mm)
        initial_heading_share    : Initial heading reference (deg)
        end_flg                  : End-of-trial flag (1 = end)

    :param shares: Tuple of task_share.Share objects used by the observer
    :type shares: tuple
    :yield: None; used as a cooperative task generator
    :rtype: None
    """
    (meas_heading_share, meas_yaw_rate_share,
     left_enc_pos, right_enc_pos,
     left_enc_speed, right_enc_speed,
     obs_heading_share, obs_yaw_rate_share,
     left_set_point, right_set_point,
     observer_calibration_flg,
     big_X_share, big_Y_share, initial_heading_share, end_flg) = shares

    # ----------------------------------------------------------------------
    # Kinematics model
    # ----------------------------------------------------------------------
    def kinematics_fun(t, x, omega_L_ms, omega_R_ms, imu_heading_rad):
        """
        Full kinematic model for differential-drive robot.

        State:
            x = [X, Y, Theta, s, Omega_L, Omega_R]^T

        Inputs:
            omega_L_ms, omega_R_ms : Measured wheel linear velocities (m/s)
            imu_heading_rad        : Corrected IMU heading (radians)

        :param t: Time (unused, included for RK4 compatibility)
        :type t: float
        :param x: Current state vector as 6x1 column (ulab/numpy array)
        :type x: ulab.numpy.ndarray
        :param omega_L_ms: Left wheel velocity in m/s
        :type omega_L_ms: float
        :param omega_R_ms: Right wheel velocity in m/s
        :type omega_R_ms: float
        :param imu_heading_rad: Corrected IMU heading in radians
        :type imu_heading_rad: float
        :return: (xd, y) where xd is state derivative, y is output vector
        :rtype: tuple(ulab.numpy.ndarray, ulab.numpy.ndarray)
        """
        w = 0.141  # Track width [m]

        X = x[0, 0]
        Y = x[1, 0]
        Theta = x[2, 0]
        s = x[3, 0]

        # Measured linear wheel velocities (already in m/s)
        v_L = omega_L_ms
        v_R = omega_R_ms

        v_center = (v_L + v_R) / 2.0       # Forward velocity at center
        omega_body = (v_R - v_L) / w       # Yaw rate (rad/s)

        # Position update uses corrected IMU heading
        heading_corrected = imu_heading_rad
        X_dot = v_center * cos(heading_corrected)
        Y_dot = -v_center * sin(heading_corrected)

        Theta_dot = omega_body
        s_dot = v_center

        xd = np.array([
            [X_dot],
            [Y_dot],
            [Theta_dot],
            [s_dot],
            [v_L],
            [v_R],
        ])

        y = np.array([
            [X],
            [Y],
            [Theta],
            [s],
            [v_center],   # Linear velocity
            [omega_body], # Angular velocity
        ])

        return xd, y

    # ----------------------------------------------------------------------
    # Robot parameters & scaling
    # ----------------------------------------------------------------------
    r = 0.035              # Wheel radius [m]
    w = 0.141              # Track width [m]
    TICKS_PER_REV = 1437.1
    WHEEL_CIRC_M = 2 * pi * r

    # Velocity scale factor:
    #   - If observer distance < physical distance, increase VELOCITY_SCALE.
    #   - If observer distance > physical distance, decrease VELOCITY_SCALE.
    VELOCITY_SCALE = 2.0

    FIRST_RUN = True
    run = False
    printed = False
    x = None

    imu_heading_offset_rad = 0.0
    prev_imu_heading_deg = 0.0
    imu_alpha = 0.3  # Low-pass filter coefficient for IMU heading

    # ----------------------------------------------------------------------
    # Main observer loop
    # ----------------------------------------------------------------------
    while True:
        # Initialize observer when calibration flag is set
        if observer_calibration_flg.get() == 1:
            initial_heading_deg = meas_heading_share.get()

            # Robot is assumed pointing +X (0°). Offset aligns IMU reading.
            imu_heading_offset_deg = -initial_heading_deg
            imu_heading_offset_rad = imu_heading_offset_deg * pi / 180.0

            print(f"IMU heading at startup: {initial_heading_deg:.2f}°")
            print(
                "IMU heading offset (to +X = 0°): "
                f"{imu_heading_offset_deg:.2f}° ({imu_heading_offset_rad:.4f} rad)"
            )

            # Use 0° as reference heading in global frame
            initial_heading_share.put(0.0)

            # Initial position in mm, stored to shares (also used below in state)
            initial_X_mm = 100.0
            initial_Y_mm = 800.0
            big_X_share.put(initial_X_mm)
            big_Y_share.put(initial_Y_mm)

            observer_calibration_flg.put(0)
            run = True

        if run:
            if FIRST_RUN:
                # Initial state: position in meters, heading = 0 rad
                x = np.array([
                    [100.0 / 1000.0],  # X = 100 mm
                    [800.0 / 1000.0],  # Y = 800 mm
                    [0.0],             # Theta
                    [0.0],             # s
                    [0.0],             # Omega_L
                    [0.0],             # Omega_R
                ])
                FIRST_RUN = False
                print("[FIRST_RUN] Observer initialized at (100, 800) mm, heading 0° (+X)\n")

            # Encoder speeds in ticks/us
            omega_L_ticks_us = left_enc_speed.get()
            omega_R_ticks_us = right_enc_speed.get()

            # Convert from ticks/us to m/s
            # ticks/us * (meters/tick) * (1e6 us/s) = m/s
            meters_per_tick = WHEEL_CIRC_M / TICKS_PER_REV
            omega_L_ms = omega_L_ticks_us * meters_per_tick * 1e6 * VELOCITY_SCALE
            omega_R_ms = omega_R_ticks_us * meters_per_tick * 1e6 * VELOCITY_SCALE

            # IMU heading in degrees (low-pass filtered)
            imu_heading_deg = meas_heading_share.get()
            imu_heading_deg = (
                imu_alpha * imu_heading_deg
                + (1.0 - imu_alpha) * prev_imu_heading_deg
            )
            prev_imu_heading_deg = imu_heading_deg

            imu_heading_rad = imu_heading_deg * pi / 180.0

            # Apply static offset to align IMU frame with robot frame
            heading_corrected_rad = imu_heading_rad + imu_heading_offset_rad
            heading_corrected_deg = heading_corrected_rad * 180.0 / pi
            heading_corrected_deg = normalize_angle_deg(heading_corrected_deg)

            # Wrapper for RK4 solver (expects f(t, x) -> (xd, y))
            def kinematics_wrapper(t, x_col):
                xd, y = kinematics_fun(
                    t, x_col, omega_L_ms, omega_R_ms, heading_corrected_rad
                )
                return xd, y

            # RK4 integration from 0 to 0.1 s with 0.01 s step
            tout, yout = RK4_solver(kinematics_wrapper, x, [0, 0.1], 0.01)

            # Use final state from integration
            x = np.array([[yout[-1, i]] for i in range(6)])

            X_final = x[0, 0]  # m
            Y_final = x[1, 0]  # m
            s_final = x[3, 0]  # m

            # For output, use corrected IMU heading
            Theta_final_deg = heading_corrected_deg

            X_final_mm = X_final * 1000.0
            Y_final_mm = Y_final * 1000.0

            big_X_share.put(X_final_mm)
            big_Y_share.put(Y_final_mm)
            obs_heading_share.put(Theta_final_deg)
            obs_yaw_rate_share.put(meas_yaw_rate_share.get())

            # Update state heading with corrected IMU heading (radians)
            x[2, 0] = heading_corrected_rad

            # Optional debug summary when trial ends
            if end_flg.get() == 1 and not printed:
                v_avg_ms = (omega_L_ms + omega_R_ms) / (2.0 * VELOCITY_SCALE)
                v_avg_cms = v_avg_ms * 100.0
                print(
                    "X = {:.1f} mm, Y = {:.1f} mm, Heading = {:.2f}° "
                    "(corrected), s = {:.1f} mm | v = {:.1f} cm/s".format(
                        X_final_mm,
                        Y_final_mm,
                        Theta_final_deg,
                        s_final * 1000.0,
                        v_avg_cms,
                    )
                )
                printed = True

        yield
