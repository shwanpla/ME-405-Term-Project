"""
Handler for BNO055 IMU with calibration persistence.

Implements the startup flow:
    1. Check for existing calibration file (calibration.txt)
    2. If present, load calibration from file
    3. Otherwise, perform manual calibration
    4. Save calibration to file once complete
    5. Then run IMU monitoring tasks

This module provides:
    - A calibration task (state machine) to manage startup calibration
    - An IMU monitor task that outputs a continuous (unwrapped) heading
      and yaw rate via task shares
"""

import pyb
from pyb import Pin
try:
    from .IMU_driver import BNO055
    from .CalibrationManager import CalibrationManager
except ImportError:
    from IMU_driver import BNO055
    from CalibrationManager import CalibrationManager


def calibration_task_fun(shares):
    """
    Task function to handle the IMU calibration startup routine.

    This task implements a simple state machine:
        - Initialize IMU and reset line
        - Check for stored calibration file
        - Load calibration from file, or perform manual calibration
        - Save calibration once fully calibrated
        - Indicate completion, then remain in DONE state

    :param shares: List of task shares (not used in this task, kept for API compatibility)
    :type shares: list[task_share.Share] | list
    :yield: Current state value on each iteration
    :rtype: int
    """
    S_INIT = 0
    S_CHECK_FILE = 1
    S_LOAD_CALIB = 2
    S_MANUAL_CALIB = 3
    S_SAVE_CALIB = 4
    S_DONE = 5
    STATE = S_INIT

    imu = None
    calib_mgr = None
    calib_status = None
    done_printed = False

    while True:
        if STATE == S_INIT:
            print("=" * 50)
            print("=== BNO055 Initialization ===")
            print("=" * 50)

            # Hardware reset of BNO055 (active low)
            rst = Pin('B15', Pin.OUT_PP)
            rst.low()
            pyb.delay(10)
            rst.high()
            pyb.delay(700)

            try:
                imu = BNO055(scl_pin='B13', sda_pin='B14', addr=0x28)
                print("[OK] BNO055 hardware initialized")
                calib_mgr = CalibrationManager(imu)
                STATE = S_CHECK_FILE
            except Exception as e:
                print("[FAIL] BNO055 initialization failed: " + str(e))
                print("Check: VIN connected? COM3/ADR pin state?")
                pyb.delay(1000)

        elif STATE == S_CHECK_FILE:
            print("\n=== Checking for Calibration File ===")
            if calib_mgr.calibration_exists():
                print("[OK] calibration.txt found")
                STATE = S_LOAD_CALIB
            else:
                print("[INFO] calibration.txt not found")
                print("Manual calibration required (rotate IMU in all directions)")
                STATE = S_MANUAL_CALIB

        elif STATE == S_LOAD_CALIB:
            print("\n=== Loading Calibration from File ===")
            if calib_mgr.load_calibration():
                pyb.delay(100)
                STATE = S_DONE
            else:
                print("Falling back to manual calibration...")
                STATE = S_MANUAL_CALIB

        elif STATE == S_MANUAL_CALIB:
            print("\n=== Manual Calibration Mode ===")
            print("Waiting for full calibration (Sys: 3/3)...")
            print("Rotate the IMU slowly in all directions...")

            try:
                calib_status = imu.get_calibration_status()
                sys_cal = calib_status['sys']

                if sys_cal == 3:
                    print("[OK] Fully calibrated! (Sys: " + str(sys_cal) + "/3)")
                    STATE = S_SAVE_CALIB
            except Exception as e:
                print("[FAIL] Calibration read error: " + str(e))
                pyb.delay(500)

        elif STATE == S_SAVE_CALIB:
            print("\n=== Saving Calibration ===")
            if calib_mgr.save_calibration():
                STATE = S_DONE
            else:
                print("Warning: Could not save calibration")
                STATE = S_DONE

        elif STATE == S_DONE:
            if not done_printed:
                print("\n" + "=" * 50)
                print("=== Calibration Complete - Starting Main Loop ===")
                print("=" * 50 + "\n")
                done_printed = True
            STATE = S_DONE

        yield STATE


def imu_monitor_task_fun(shares):
    """
    Task to read and publish IMU data after calibration.

    This task alternates between:
        - Publishing a continuous (unwrapped) heading
        - Publishing the yaw rate (gyro Z)

    The continuous heading is computed by "unwrapping" the raw 0–360 degree
    heading to avoid jumps at the 0/360 boundary. For example, if the raw
    heading jumps from 359 to 1 degrees, the continuous heading will advance
    smoothly through 360 instead of wrapping back to 0.

    Shares usage:
        shares[0] : IMU object (BNO055)       [input]
        shares[1] : continuous heading (float) [output]
        shares[2] : yaw rate (float)          [output]

    :param shares: List of three task shares as described above
    :type shares: list[task_share.Share]
    :yield: None; cooperative task scheduler uses the generator
    :rtype: None
    """
    imu_share = shares[0]
    heading_share = shares[1]      # continuous heading out
    yaw_rate_share = shares[2]

    last = None

    # Persistent variables
    prev_heading = None            # last raw IMU heading (0–360)
    continuous_heading = 0.0       # unwrapped heading

    while True:
        imu = imu_share.get()

        if last != "HEADING":
            # Read raw IMU heading (0–360)
            raw_h = imu.get_heading()

            if prev_heading is None:
                # First measurement: initialize
                prev_heading = raw_h
                continuous_heading = raw_h
            else:
                # Compute delta in raw space
                delta = raw_h - prev_heading

                # Unwrap across 0/360 boundary
                if delta > 180:
                    # 359 → small value (e.g., 359 → 1 ~ -358)
                    delta -= 360
                elif delta < -180:
                    # Small value → 359 (e.g., 1 → 359 ~ +358)
                    delta += 360

                continuous_heading += delta
                prev_heading = raw_h

            # Publish continuous heading
            heading_share.put(continuous_heading)
            last = "HEADING"

        else:
            # Publish yaw rate (deg/s)
            yaw_rate = imu.get_yaw_rate()
            yaw_rate_share.put(yaw_rate)
            last = "YAW_RATE"

        yield
