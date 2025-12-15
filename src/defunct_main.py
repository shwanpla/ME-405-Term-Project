"""
Main program for ME 405 Lab autonomous robot system.

Implements a cooperative multitasking scheduler coordinating:
    - Motor drivers and quadrature encoders
    - Closed-loop PI velocity controllers
    - Line-following IR sensor array
    - BNO055 IMU calibration + heading monitoring
    - State observer for filtered orientation & yaw rate
    - High-level navigation task
    - Serial communication for debugging and calibration

Hardware:
    - Motors: 2 DC motors, each with quadrature encoder
    - IR Sensor Array: 7 reflectance sensors
    - IMU: BNO055 with calibration persistence
    - Wheel Diameter: 70 mm (35 mm radius)
    - Track Width: 141 mm

Notes:
    - Uses cotask priority scheduler
    - Velocity control runs every 12 ms
    - PI control runs every 20 ms
    - IMU monitor runs every 70 ms
    - Observer runs every 100 ms
    - Navigation task orchestrates full behavior
"""

import gc
import pyb
import cotask
import task_share
from pyb import Pin, Timer, UART
from time import ticks_us, ticks_diff

from motor import motor_driver
from encoder import Encoder
from multi_sensor_read import multiple_ir_readings
import motor_ctrl_task_V3 as motor_ctrl_task
import CL_control_task_V5 as CL_control_task
from serial_task_V6 import serial_task_fun

try:
    from .defunct_IMU_driver import BNO055
    from .defunct_CalibrationManager import CalibrationManager
    from .defunct_IMU_handler import calibration_task_fun, imu_monitor_task_fun
except ImportError:
    from original_approach.defunct_IMU_driver import BNO055
    from original_approach.defunct_CalibrationManager import CalibrationManager
    from original_approach.defunct_IMU_handler import calibration_task_fun, imu_monitor_task_fun

from original_approach.defunct_observer_fcn import observer_task_fcn
from navigation import navigation_task_fun


# ──────────────────────────────────────────────────────────────────────────────
# IMU Initialization & Calibration
# ──────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    print("\n\nStarting BNO055 Calibration Routine...\n")

    imu_obj = None
    calib_mgr_obj = None

    # IMU Hardware Reset
    rst = Pin('B15', Pin.OUT_PP)
    rst.low()
    pyb.delay(10)
    rst.high()
    pyb.delay(700)

    try:
        imu_obj = BNO055(scl_pin='B13', sda_pin='B14', addr=0x28)
        calib_mgr_obj = CalibrationManager(imu_obj)

        if calib_mgr_obj.calibration_exists():
            print("Calibration file found — loading...")
            calib_mgr_obj.load_calibration()
        else:
            print("No calibration file — starting manual calibration.")
            print("Rotate IMU until system calibration reaches 3/3.\n")

            while True:
                status = imu_obj.get_calibration_status()
                print(
                    f"Sys: {status['sys']}/3, "
                    f"Gyro: {status['gyro']}/3, "
                    f"Accel: {status['accel']}/3, "
                    f"Mag: {status['mag']}/3"
                )

                if status['sys'] == 3:
                    print("Calibration complete.")
                    calib_mgr_obj.save_calibration()
                    break

                pyb.delay(200)

    except Exception as e:
        print("IMU initialization failed:", e)
        while True:
            pyb.delay(1000)

    # ──────────────────────────────────────────────────────────────────────────
    # Motor & Encoder Setup
    # ──────────────────────────────────────────────────────────────────────────
    mot_right = motor_driver(Pin.cpu.C9, Pin.cpu.H1, Pin.cpu.H0,
                             Timer(3, freq=30_000), 4)
    mot_left  = motor_driver(Pin.cpu.B0, Pin.cpu.C12, Pin.cpu.C10,
                             Timer(3, freq=30_000), 3)

    # Encoder pins
    chan_A_1 = Pin(Pin.cpu.A8, mode=Pin.IN)
    chan_B_1 = Pin(Pin.cpu.A9, mode=Pin.IN)
    chan_A_2 = Pin(Pin.cpu.A0, mode=Pin.IN)
    chan_B_2 = Pin(Pin.cpu.B3, mode=Pin.IN)

    enc_right = Encoder(Timer(1, prescaler=0, period=0xFFFF), chan_A_1, chan_B_1)
    enc_left  = Encoder(Timer(2, prescaler=0, period=0xFFFF), chan_A_2, chan_B_2)

    left_motor  = motor_ctrl_task.motor_control_task(mot_left, enc_left)
    right_motor = motor_ctrl_task.motor_control_task(mot_right, enc_right)

    # ──────────────────────────────────────────────────────────────────────────
    # IR Sensor Array
    # ──────────────────────────────────────────────────────────────────────────
    ir_array = multiple_ir_readings(
        Pin.cpu.C4, Pin.cpu.B1, Pin.cpu.A7,
        Pin.cpu.C1, Pin.cpu.A4, Pin.cpu.A1, Pin.cpu.C3
    )

    # ──────────────────────────────────────────────────────────────────────────
    # Closed-Loop Controllers
    # ──────────────────────────────────────────────────────────────────────────
    left_controller  = CL_control_task.CL_control(ir_array, motor="LEFT")
    right_controller = CL_control_task.CL_control(ir_array, motor="RIGHT")

    left_controller_share = task_share.Share('O', thread_protect=False,
                                             name='left_controller_share')
    left_controller_share.put(left_controller)

    right_controller_share = task_share.Share('O', thread_protect=False,
                                              name='right_controller_share')
    right_controller_share.put(right_controller)

    # ──────────────────────────────────────────────────────────────────────────
    # Shared Variables
    # ──────────────────────────────────────────────────────────────────────────
    left_start_flg  = task_share.Share('B', thread_protect=False, name='left_start_flg')
    right_start_flg = task_share.Share('B', thread_protect=False, name='right_start_flg')

    left_end_flg  = task_share.Share('B', thread_protect=False, name='left_end_flg')
    right_end_flg = task_share.Share('B', thread_protect=False, name='right_end_flg')

    left_desired_vel  = task_share.Share('f', thread_protect=False, name='left_desired_vel')
    right_desired_vel = task_share.Share('f', thread_protect=False, name='right_desired_vel')

    left_set_point  = task_share.Share('f', thread_protect=False, name='left_set_point')
    right_set_point = task_share.Share('f', thread_protect=False, name='right_set_point')

    desired_time_ms = task_share.Share('f', thread_protect=False, name='desired_time_ms')
    desired_time_ms.put(75_000 * 1000)

    left_proc_data_flg  = task_share.Share('B', thread_protect=False, name='left_proc_data_flg')
    right_proc_data_flg = task_share.Share('B', thread_protect=False, name='right_proc_data_flg')

    kp_share = task_share.Share('f', thread_protect=False, name='kp_share')
    ki_share = task_share.Share('f', thread_protect=False, name='ki_share')

    # IMU shared data
    imu_share = task_share.Share('O', thread_protect=False, name='imu')
    imu_share.put(imu_obj)

    line_follow_flg = task_share.Share('B', thread_protect=False, name='line_follow_flg')
    line_follow_flg.put(0)

    desired_angle_share = task_share.Share('f', thread_protect=False, name='desired_angle_share')
    desired_angle_share.put(0.0)

    bias_share = task_share.Share('f', thread_protect=False, name='bias_share')
    bias_share.put(1.55)

    force_straight_flg = task_share.Share('B', thread_protect=False, name='force_straight_flg')
    force_straight_flg.put(0)

    bias_timer_flg = task_share.Share('B', thread_protect=False, name='bias_timer_flg')
    bias_timer_flg.put(0)

    nav_stop_flg = task_share.Share('B', thread_protect=False, name='nav_stop_flg')
    nav_stop_flg.put(0)

    # Observer / Navigation shares
    big_X_share = task_share.Share('f', thread_protect=False, name='X')
    big_Y_share = task_share.Share('f', thread_protect=False, name='Y')

    initial_heading_share = task_share.Share('f', thread_protect=False,
                                             name='initial_heading')

    observer_calibration_flg = task_share.Share('B', thread_protect=False,
                                                name='observer_calibration_flg')

    meas_heading_share = task_share.Share('f', thread_protect=False, name='heading')
    meas_heading_share.put(0.0)

    meas_yaw_rate_share = task_share.Share('f', thread_protect=False, name='yaw_rate')
    meas_yaw_rate_share.put(0.0)

    obs_heading_share = task_share.Share('f', thread_protect=False, name='obs_heading')
    obs_heading_share.put(0.0)

    obs_yaw_rate_share = task_share.Share('f', thread_protect=False, name='obs_yaw_rate')
    obs_yaw_rate_share.put(0.0)

    calibration_flg = task_share.Share('B', thread_protect=False, name='calibration_flg')

    nav_rest_flg = task_share.Share('B', thread_protect=False, name='nav_rest_flg')
    nav_turn_flg = task_share.Share('B', thread_protect=False, name='nav_turn_flg')

    # Raw/Processed data queues (unchanged from original)
    left_enc_speed  = task_share.Share('f', thread_protect=False, name='left_enc_speed')
    right_enc_speed = task_share.Share('f', thread_protect=False, name='right_enc_speed')
    left_enc_pos    = task_share.Share('f', thread_protect=False, name='left_enc_pos')
    right_enc_pos   = task_share.Share('f', thread_protect=False, name='right_enc_pos')
    left_exp_time   = task_share.Share('f', thread_protect=False, name='left_exp_time')
    right_exp_time  = task_share.Share('f', thread_protect=False, name='right_exp_time')

    proc_speed_left  = task_share.Queue('f', 10, thread_protect=False, overwrite=False,
                                        name='proc_speed_left')
    proc_speed_right = task_share.Queue('f', 10, thread_protect=False, overwrite=False,
                                        name='proc_speed_right')
    proc_pos_left    = task_share.Queue('f', 10, thread_protect=False, overwrite=False,
                                        name='proc_pos_left')
    proc_pos_right   = task_share.Queue('f', 10, thread_protect=False, overwrite=False,
                                        name='proc_pos_right')
    left_proc_time   = task_share.Queue('f', 10, thread_protect=False,
                                        overwrite=False, name='left_proc_time')
    right_proc_time  = task_share.Queue('f', 10, thread_protect=False,
                                        overwrite=False, name='right_proc_time')

    # ──────────────────────────────────────────────────────────────────────────
    # Task Creation
    # ──────────────────────────────────────────────────────────────────────────
    serial_task = cotask.Task(
        serial_task_fun,
        name="serial_task",
        priority=3,
        period=150,
        profile=True,
        trace=False,
        shares=(
            left_start_flg, right_start_flg,
            left_desired_vel, right_desired_vel,
            left_end_flg, right_end_flg,
            left_proc_data_flg, right_proc_data_flg,
            proc_pos_right, proc_pos_left,
            proc_speed_right, proc_speed_left,
            left_proc_time, right_proc_time,
            calibration_flg, observer_calibration_flg,
            big_X_share, big_Y_share,
            initial_heading_share,
        ),
    )

    left_mot_ctrl_task = cotask.Task(
        left_motor.run,
        name="left_mot_ctrl",
        priority=3,
        period=12,
        profile=True,
        trace=False,
        shares=(
            left_start_flg, left_set_point, left_end_flg,
            left_enc_pos, left_enc_speed,
            left_exp_time, desired_time_ms,
            left_proc_data_flg, left_desired_vel,
        ),
    )

    right_mot_ctrl_task = cotask.Task(
        right_motor.run,
        name="right_mot_ctrl",
        priority=3,
        period=12,
        profile=True,
        trace=False,
        shares=(
            right_start_flg, right_set_point, right_end_flg,
            right_enc_pos, right_enc_speed,
            right_exp_time, desired_time_ms,
            right_proc_data_flg, right_desired_vel,
        ),
    )

    left_CL_control_task = cotask.Task(
        left_controller.run,
        name="left_CL_ctrl",
        priority=3,
        period=20,
        profile=True,
        trace=False,
        shares=(
            left_desired_vel, left_enc_speed, left_set_point, left_end_flg,
            ki_share, kp_share, calibration_flg,
            line_follow_flg, desired_angle_share,
            obs_heading_share,
            nav_rest_flg, nav_turn_flg,
            bias_share, force_straight_flg,
        ),
    )

    right_CL_control_task = cotask.Task(
        right_controller.run,
        name="right_CL_ctrl",
        priority=3,
        period=20,
        profile=True,
        trace=False,
        shares=(
            right_desired_vel, right_enc_speed, right_set_point, right_end_flg,
            ki_share, kp_share, calibration_flg,
            line_follow_flg, desired_angle_share,
            obs_heading_share,
            nav_rest_flg, nav_turn_flg,
            bias_share, force_straight_flg,
        ),
    )

    monitor_task = cotask.Task(
        imu_monitor_task_fun,
        name="monitor",
        priority=3,
        period=70,
        profile=True,
        trace=False,
        shares=(imu_share, meas_heading_share, meas_yaw_rate_share),
    )

    observer_task = cotask.Task(
        observer_task_fcn,
        name="observer",
        priority=3,
        period=100,
        profile=True,
        trace=False,
        shares=(
            meas_heading_share, meas_yaw_rate_share,
            left_enc_pos, right_enc_pos,
            left_enc_speed, right_enc_speed,
            obs_heading_share, obs_yaw_rate_share,
            left_set_point, right_set_point,
            observer_calibration_flg,
            big_X_share, big_Y_share,
            initial_heading_share,
            left_end_flg,
        ),
    )

    navigation_task = cotask.Task(
        navigation_task_fun,
        name="navigation",
        priority=4,
        period=50,
        profile=True,
        trace=False,
        shares=(
            line_follow_flg, big_X_share, big_Y_share,
            obs_heading_share, obs_yaw_rate_share,
            left_controller_share, right_controller_share,
            left_set_point, right_set_point,
            left_desired_vel, right_desired_vel,
            desired_angle_share, nav_rest_flg,
            left_end_flg, bias_share,
            force_straight_flg, bias_timer_flg,
            nav_stop_flg, calibration_flg, nav_turn_flg,
        ),
    )

    # ──────────────────────────────────────────────────────────────────────────
    # Scheduler Setup
    # ──────────────────────────────────────────────────────────────────────────
    cotask.task_list.append(serial_task)
    cotask.task_list.append(left_mot_ctrl_task)
    cotask.task_list.append(right_mot_ctrl_task)
    cotask.task_list.append(left_CL_control_task)
    cotask.task_list.append(right_CL_control_task)
    cotask.task_list.append(monitor_task)
    cotask.task_list.append(observer_task)
    cotask.task_list.append(navigation_task)

    gc.collect()

    # ──────────────────────────────────────────────────────────────────────────
    # Main Scheduler Loop
    # ──────────────────────────────────────────────────────────────────────────
    while True:
        try:
            cotask.task_list.pri_sched()
        except KeyboardInterrupt:
            print(cotask.task_list)
            break
