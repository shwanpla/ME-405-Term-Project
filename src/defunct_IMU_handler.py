"""
BNO055 IMU handler tasks for calibration and continuous monitoring.

This module provides cooperative multitasking functions for managing BNO055 IMU
initialization, calibration persistence, and continuous sensor data reading. The
calibration task implements a state machine for automatic calibration management,
while the monitor task provides continuous heading and yaw rate updates.

Tasks:
    - calibration_task_fun: Manages IMU initialization and calibration workflow
    - imu_monitor_task_fun: Continuously reads heading and yaw rate from IMU

Hardware:
    - IMU: BNO055 9-DOF sensor (I2C on B13/B14, reset on B15)
    - Calibration File: calibration.txt for persistent storage

Notes:
    - Calibration task runs once at startup to establish IMU calibration
    - Monitor task produces unwrapped continuous heading (no 0/360 wraparound)
    - Uses cooperative multitasking with cotask scheduler
    - Implements automatic calibration file management
"""

import gc
import pyb
import cotask
import task_share
from pyb import Pin
from defunct_IMU_driver import BNO055
from defunct_CalibrationManager import CalibrationManager


def calibration_task_fun(shares):
    """
    Cooperative task function implementing IMU calibration state machine.

    Manages the complete IMU initialization and calibration workflow using a
    6-state finite state machine. Automatically detects existing calibration
    files, loads calibration if available, or guides the user through manual
    calibration if needed.

    Args:
        shares (tuple): Task shares (currently unused, reserved for future use)

    Yields:
        int: Current state number (0-5)

    State Machine:
        - S_INIT (0): Initialize BNO055 hardware and reset pin
        - S_CHECK_FILE (1): Check for existing calibration.txt file
        - S_LOAD_CALIB (2): Load calibration from file
        - S_MANUAL_CALIB (3): Manual calibration mode (rotate IMU)
        - S_SAVE_CALIB (4): Save calibration to file
        - S_DONE (5): Calibration complete, idle state

    Hardware Setup:
        - Reset pin B15: Pulsed low then high to reset BNO055
        - I2C pins B13/B14: Software I2C communication
        - I2C address: 0x28 (default)

    Calibration Requirements:
        - System calibration status must reach 3/3
        - IMU must be rotated in all directions during manual calibration
        - Calibration data saved to calibration.txt for persistence

    Example Output:
        >>> # First run (no calibration file)
        ==================================================
        === BNO055 Initialization ===
        ==================================================
        [OK] BNO055 hardware initialized

        === Checking for Calibration File ===
        [INFO] calibration.txt not found
        Manual calibration required (rotate IMU in all directions)

        === Manual Calibration Mode ===
        Waiting for full calibration (Sys: 3/3)...
        Rotate the IMU slowly in all directions...
        [OK] Fully calibrated! (Sys: 3/3)

        === Saving Calibration ===
        ✓ Calibration saved to calibration.txt

        ==================================================
        === Calibration Complete - Starting Main Loop ===
        ==================================================
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
                #else:
                    # msg = ("Sys: " + str(calib_status['sys']) + "/3, Gyro: " +
                    #        str(calib_status['gyro']) + "/3, Accel: " +
                    #        str(calib_status['accel']) + "/3, Mag: " +
                    #        str(calib_status['mag']) + "/3")
                    # print("  " + msg)
                    # pyb.delay(500)
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
    Cooperative task function for continuous IMU data monitoring with heading unwrapping.

    Alternately reads heading and yaw rate from the BNO055 IMU, updating shared
    variables for navigation and control tasks. Implements heading unwrapping to
    produce continuous heading values that do not wrap at 0/360 degrees.

    Args:
        shares (tuple): Task shared variables
            - shares[0] (Share): IMU object share (BNO055 instance)
            - shares[1] (Share): Continuous heading output (float, degrees)
            - shares[2] (Share): Yaw rate output (float, deg/s)

    Yields:
        None: Yields control to scheduler after each sensor read

    Heading Unwrapping Algorithm:
        The function maintains a continuous heading by detecting and correcting
        wraparound at the 0/360 degree boundary:

        1. Read raw heading from IMU (0-360 degrees)
        2. Calculate delta from previous reading
        3. Detect wraparound:
           - If delta > 180: Crossed from 359 to 0 (subtract 360)
           - If delta < -180: Crossed from 0 to 359 (add 360)
        4. Add corrected delta to continuous heading
        5. Update previous heading for next iteration

    Example:
        >>> # Heading crosses from 359° to 1°
        >>> # Raw readings: 359, 360, 0, 1, 2
        >>> # Continuous output: 359, 360, 360, 361, 362

    Timing:
        - Alternates between heading and yaw rate reads each yield
        - Typical period: 70ms (14.3 Hz update rate)

    Notes:
        - Continuous heading can exceed 360° or go negative
        - Initial heading matches first IMU reading
        - Yaw rate is read directly without processing
        - IMU object must be calibrated before this task starts
    """
    imu_share      = shares[0]
    heading_share  = shares[1]   # continuous heading out
    yaw_rate_share = shares[2]

    last = None

    # --- persistent variables ---
    prev_heading = None          # last raw IMU heading (0–360)
    continuous_heading = 0.0     # unwrapped heading

    while True:
        imu = imu_share.get()

        if last != "HEADING":
            # Read raw IMU heading (0→360)
            raw_h = imu.get_heading()

            if prev_heading is None:
                # First measurement, initialize
                prev_heading = raw_h
                continuous_heading = raw_h
            else:
                # Compute delta
                delta = raw_h - prev_heading

                # --- unwrap logic ---
                if delta > 180:
                    # Jumped 359 → ~0 (e.g., 359 → 1 ≈ -358)
                    delta -= 360
                elif delta < -180:
                    # Jumped 0 → ~359 (e.g., 1 → 359 ≈ +358)
                    delta += 360

                # Add delta to continuous value
                continuous_heading += delta
                prev_heading = raw_h

            # Output continuous heading
            heading_share.put(continuous_heading)
            last = "HEADING"

        else:
            yaw_rate = imu.get_yaw_rate()
            yaw_rate_share.put(yaw_rate)
            last = "YAW_RATE"

        yield

# def imu_monitor_task_fun(shares):
#     '''Task to read and display IMU data after calibration'''
#     last = None
#     while True:
#         imu = shares[0].get()
#         heading_share = shares[1]
#         yaw_rate_share = shares[2]

#         if last != "HEADING":
#             heading = imu.get_heading()
#             heading_share.put(heading)
#             last = "HEADING"
#         else:
#             yaw_rate = imu.get_yaw_rate()
#             yaw_rate_share.put(yaw_rate)
#             last = "YAW_RATE"
#         yield

