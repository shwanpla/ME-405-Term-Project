'''
Handler for BNO055 IMU with calibration persistence
Implements flowchart: check for calibration.txt -> load or manual calibrate -> run
'''

import gc
import pyb
import cotask
import task_share
from pyb import Pin
from IMU_driver import BNO055
from CalibrationManager import CalibrationManager


def calibration_task_fun(shares):
    '''Task to handle calibration startup routine'''
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
    """Task to read and display IMU data after calibration.
    Produces continuous heading that does NOT wrap at 0/360.
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

