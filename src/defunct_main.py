"""
Defunct main program for IMU-based obstacle course navigation robot.
This is a previous version that uses BNO055 IMU for heading estimation instead
of encoder-based odometry. Implements cooperative multitasking with IMU calibration,
observer-based state estimation, and navigation control.

Hardware:
    - Motors: 2x DC motors with encoders (1437.1 ticks/rev)
    - IR Sensor Array: 7 reflectance sensors for line detection
    - IMU: BNO055 9-DOF sensor (I2C on B13/B14, reset on B15)
    - Wheel Diameter: 70mm (35mm radius)
    - Track Width: 141mm

Notes:
    - Uses cotask cooperative scheduler with priority-based task execution
    - IMU calibration runs at startup, saves/loads calibration data
    - Observer task fuses IMU heading with encoder data for state estimation
    - Velocity control runs at 12ms period, closed-loop control at 20ms
    - Monitor task reads IMU data at 70ms, observer updates at 100ms
    - Navigation task coordinates line following, turns, and positioning

Terminal Commands:
    - Listing USB serial devices:       ls /dev/tty.*
    - Connecting to board:               screen /dev/tty.usbmodem205F379C39472 115200
    - Exit screen:                       Ctrl-A, K, Y
    - Soft reset board:                  Ctrl-D
"""

import gc
import pyb
import cotask
import task_share
from pyb import Pin, Timer, UART
from time import ticks_us, ticks_diff
from motor import motor_driver
from encoder import Encoder

import motor_ctrl_task_V3 as motor_ctrl_task
import CL_control_task_V5 as CL_control_task
from serial_task_V6 import serial_task_fun
from multi_sensor_read import multiple_ir_readings 

from defunct_IMU_driver import BNO055
from defunct_CalibrationManager import CalibrationManager
from defunct_IMU_handler import calibration_task_fun, imu_monitor_task_fun
from defunct_observer_fcn import observer_task_fcn
from defunct_navigation import navigation_task_fun

# Creating Shares and Queues
#----------------------------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    ###--------------------------- Start IMU calibration code------------------------------###
    print("\n\nStarting BNO055 Calibration Routine...\n")
    imu_obj = None
    calib_mgr_obj = None
    
    rst = Pin('B15', Pin.OUT_PP)
    rst.low()
    pyb.delay(10)
    rst.high()
    pyb.delay(700)
    
    try:
        imu_obj = BNO055(scl_pin='B13', sda_pin='B14', addr=0x28)
        calib_mgr_obj = CalibrationManager(imu_obj)
        
        if calib_mgr_obj.calibration_exists():
            print("Calibration file found - loading...")
            calib_mgr_obj.load_calibration()
        else:
            print("No calibration file - enter manual calibration mode")
            print("Rotate IMU in all directions until Sys = 3/3")
            print("Starting calibration now...\n")
            
            while True:
                cal_status = imu_obj.get_calibration_status()
                msg = ("Sys: " + str(cal_status['sys']) + "/3, Gyro: " +
                       str(cal_status['gyro']) + "/3, Accel: " +
                       str(cal_status['accel']) + "/3, Mag: " +
                       str(cal_status['mag']) + "/3")
                print(msg)
                
                if cal_status['sys'] == 3:
                    print("[OK] Calibration complete!")
                    calib_mgr_obj.save_calibration()
                    break
                
                pyb.delay(200)
    
    except Exception as e:
        print("Initialization failed: " + str(e))
        print("Halting...")
        while True:
            pyb.delay(1000)
###--------------------------- End of IMU calibration code------------------------------###
    # Define motors
    mot_right = motor_driver(Pin.cpu.C9, Pin.cpu.H1, Pin.cpu.H0, Timer(3, freq=30_000), 4) 
    mot_left = motor_driver(Pin.cpu.B0, Pin.cpu.C12, Pin.cpu.C10, Timer(3, freq=30_000), 3)

    # Define encoder pins
    chan_A_1 = Pin(Pin.cpu.A8, mode=Pin.IN)       # Encoder 1 Channel A
    chan_B_1 = Pin(Pin.cpu.A9, mode=Pin.IN)       # Encoder 1 Channel B
    chan_A_2 = Pin(Pin.cpu.A0, mode=Pin.IN)       # Encoder 2 Channel A
    chan_B_2 = Pin(Pin.cpu.B3, mode=Pin.IN)       # Encoder 2 Channel B

    # Define encoders
    enc_right = Encoder(Timer(1, prescaler=0, period=0xFFFF), chan_A_1, chan_B_1)
    enc_left  = Encoder(Timer(2, prescaler=0, period=0xFFFF), chan_A_2, chan_B_2)

    # Create motor objects
    left_motor = motor_ctrl_task.motor_control_task(mot_left, enc_left)
    right_motor = motor_ctrl_task.motor_control_task(mot_right, enc_right)


    ## Define IR Sensor Pins
    ir_pin_1 = Pin.cpu.C4
    ir_pin_2 = Pin.cpu.B1
    ir_pin_3 = Pin.cpu.A7
    ir_pin_4 = Pin.cpu.C1
    ir_pin_5 = Pin.cpu.A4
    ir_pin_6 = Pin.cpu.A1
    ir_pin_7 = Pin.cpu.C3

    ir_array = multiple_ir_readings(ir_pin_1, ir_pin_2, ir_pin_3, ir_pin_4, 
                                    ir_pin_5, ir_pin_6, ir_pin_7)
            
    # Create closed loop controller objects
    left_controller = CL_control_task.CL_control(ir_array, motor="LEFT")
    right_controller = CL_control_task.CL_control(ir_array, motor="RIGHT")

    left_controller_share = task_share.Share('O', thread_protect=False, name='left_controller_share')
    left_controller_share.put(left_controller)
    right_controller_share = task_share.Share('O', thread_protect=False, name='right_controller_share')
    right_controller_share.put(right_controller)
  

    # Create shares and queues
    left_start_flg = task_share.Share('B', thread_protect = False, name = 'left_start_flg') # Flag to start trials
    right_start_flg = task_share.Share('B', thread_protect = False, name = 'right_start_flg') # Flag to start trials
    
    left_end_flg = task_share.Share('B', thread_protect = False, name = 'left_end_flg')     # Flag to end trials
    right_end_flg = task_share.Share('B', thread_protect = False, name = 'right_end_flg')     # Flag to end trials
   
    left_effort_lvl = task_share.Share('h', thread_protect = False, name = 'left_effort_lvl')    # Effort level for trials
    right_effort_lvl = task_share.Share('h', thread_protect = False, name = 'right_effort_lvl')    # Effort level for trials

    left_desired_vel = task_share.Share('f', thread_protect = False, name = 'left_desired_vel')    # Velocity setpoint for closed loop control
    right_desired_vel = task_share.Share('f', thread_protect = False, name = 'right_desired_vel')    # Velocity setpoint for closed loop control

    left_set_point = task_share.Share('f', thread_protect = False, name = 'left_set_point')  # Control effort output for closed loop control
    right_set_point = task_share.Share('f', thread_protect = False, name = 'right_set_point')  # Control effort output for closed loop control

    desired_time_ms = task_share.Share('f', thread_protect = False, name = 'desired_time_ms')
    
    left_proc_data_flg = task_share.Share('B', thread_protect = False, name = 'left_proc_data_flg')
    right_proc_data_flg = task_share.Share('B', thread_protect = False, name = 'right_proc_data_flg')

    kp_share = task_share.Share('f', thread_protect = False, name = 'kp_share')
    ki_share = task_share.Share('f', thread_protect = False, name = 'ki_share')

    imu_share = task_share.Share('O', thread_protect=False, name='imu')
    imu_share.put(imu_obj)

    line_follow_flg = task_share.Share('B', thread_protect=False, name='line_follow_flg')
    line_follow_flg.put(0)  # Default to no line following
    desired_angle_share = task_share.Share('f', thread_protect=False, name='desired_angle_share')
    desired_angle_share.put(0.0)



    # Navigation shares
    big_X_share = task_share.Share('f', thread_protect=False, name='X')
    big_Y_share = task_share.Share('f', thread_protect=False, name='Y')
    initial_heading_share = task_share.Share('f', thread_protect=False, name='initial_heading')
    observer_calibration_flg = task_share.Share('B', thread_protect=False, name='observer_calibration_flg')
    meas_heading_share = task_share.Share('f', thread_protect=False, name='heading')
    meas_heading_share.put(0.0)
    meas_yaw_rate_share = task_share.Share('f', thread_protect=False, name='yaw_rate')
    meas_yaw_rate_share.put(0.0)
    obs_heading_share = task_share.Share('f', thread_protect=False, name='obs_heading')
    obs_heading_share.put(0.0)  
    obs_yaw_rate_share = task_share.Share('f', thread_protect=False, name='obs_yaw_rate')
    obs_yaw_rate_share.put(0.0)

    '''#######################################
       ######## REMOVE THIS LATER ############
       #######################################'''
    desired_time_ms.put(75_000)


    calibration_flg = task_share.Share('B', thread_protect = False, name = 'calibration_flg')
    
    # Navgition stuff
    nav_rest_flg = task_share.Share('B', thread_protect=False, name='nav_rest_flg')
    nav_turn_flg = task_share.Share('B', thread_protect=False, name='nav_turn_flg')
    

    # Queues used to pass raw data from trials
    left_enc_speed = task_share.Share('f', thread_protect = False, name = 'left_enc_speed')
    right_enc_speed = task_share.Share('f', thread_protect = False, name = 'right_enc_speed')
    left_enc_pos = task_share.Share('f', thread_protect = False, name = 'left_enc_pos')
    right_enc_pos = task_share.Share('f', thread_protect = False, name = 'right_enc_pos')
    left_exp_time = task_share.Share('f', thread_protect = False, name = 'left_exp_time')
    right_exp_time = task_share.Share('f', thread_protect = False, name = 'right_exp_time')
    
    # Queues used to pass processed data from trials
    proc_speed_left = task_share.Queue('f', 10, thread_protect = False, 
                                 overwrite = False, name = 'proc_speed_right')
    proc_speed_right = task_share.Queue('f', 10, thread_protect = False, 
                                 overwrite = False, name = 'proc_speed_right')
    proc_pos_left = task_share.Queue('f', 10, thread_protect = False, 
                               overwrite = False, name = 'proc_pos_left')
    proc_pos_right = task_share.Queue('f', 10, thread_protect = False, 
                               overwrite = False, name = 'proc_pos_right')
    left_proc_time = task_share.Queue('f', 10, thread_protect = False, 
                               overwrite = False, name = 'right_proc_time')
    right_proc_time = task_share.Queue('f', 10, thread_protect = False, 
                               overwrite = False, name = 'right_proc_time')
    



    # Creating Tasks

    serial_task = cotask.Task(serial_task_fun, name="serial_task", priority=3, period=150,
                        profile=True, trace=False, shares=(left_start_flg, right_start_flg,
                                                            left_desired_vel, right_desired_vel,
                                                            left_end_flg, right_end_flg,
                                                            left_proc_data_flg, right_proc_data_flg,
                                                            proc_pos_right, proc_pos_left,
                                                            proc_speed_right, proc_speed_left, 
                                                            left_proc_time, right_proc_time,
                                                            calibration_flg, observer_calibration_flg,
                                                            big_X_share, big_Y_share, initial_heading_share))
    
    
    left_mot_ctrl_task = cotask.Task(left_motor.run, name="left_mot_ctrl", priority=3, period=12,
                                profile=True, trace=False, shares=(left_start_flg, left_set_point, left_end_flg,
                                                                   left_enc_pos, left_enc_speed, 
                                                                   left_exp_time, desired_time_ms, 
                                                                   left_proc_data_flg, left_desired_vel))
    right_mot_ctrl_task = cotask.Task(right_motor.run, name="right_mot_ctrl", priority=3, period=12,
                                profile=True, trace=False, shares=(right_start_flg, right_set_point, right_end_flg,
                                                                   right_enc_pos, right_enc_speed, 
                                                                   right_exp_time, desired_time_ms, 
                                                                   right_proc_data_flg, right_desired_vel))
    
    
    left_CL_control_task = cotask.Task(left_controller.run, name="left_CL_ctrl", priority=3, period=20,
                                profile=True, trace=False, shares=(left_desired_vel, left_enc_speed, left_set_point, left_end_flg, 
                                                                   ki_share, kp_share, calibration_flg, 
                                                                   line_follow_flg, desired_angle_share, 
                                                                   obs_heading_share, nav_rest_flg, nav_turn_flg))
    
    right_CL_control_task = cotask.Task(right_controller.run, name="right_CL_ctrl", priority=3, period=20,
                                profile=True, trace=False, shares=(right_desired_vel, right_enc_speed, right_set_point, 
                                                                   right_end_flg, ki_share, kp_share, calibration_flg,
                                                                   line_follow_flg, desired_angle_share, 
                                                                   obs_heading_share, nav_rest_flg, nav_turn_flg))
    monitor_task = cotask.Task(imu_monitor_task_fun, name="monitor", priority=3, period=70, 
                               profile=True, trace=False, shares=(imu_share, meas_heading_share, meas_yaw_rate_share))
    
    observer_task = cotask.Task(observer_task_fcn, name="observer", priority=3, period=100, 
                                profile=True, trace=False, shares=(meas_heading_share, meas_yaw_rate_share, 
                                                                    left_enc_pos, right_enc_pos,
                                                                    left_enc_speed, right_enc_speed,
                                                                    obs_heading_share, obs_yaw_rate_share, 
                                                                    left_set_point, right_set_point,
                                                                    observer_calibration_flg, big_X_share, big_Y_share, initial_heading_share, left_end_flg))
                                
    navigation_task = cotask.Task(navigation_task_fun, name="navigation", priority=4, period=50,
                                 profile=True, trace=False, shares=(line_follow_flg, big_X_share, big_Y_share,
                                                                     obs_heading_share, obs_yaw_rate_share, 
                                                                     left_controller_share, right_controller_share,
                                                                     left_set_point, right_set_point, 
                                                                     left_desired_vel, right_desired_vel,
                                                                     desired_angle_share, nav_rest_flg, left_end_flg, nav_turn_flg))
    
    # Add tasks to the scheduler
    cotask.task_list.append(serial_task)
    cotask.task_list.append(left_mot_ctrl_task)
    cotask.task_list.append(right_mot_ctrl_task)
    cotask.task_list.append(left_CL_control_task)
    cotask.task_list.append(right_CL_control_task)
    cotask.task_list.append(monitor_task)
    cotask.task_list.append(observer_task)
    cotask.task_list.append(navigation_task)

    # Run garbage collector
    gc.collect()

    # Start the scheduler
    while True:
        try:
            cotask.task_list.pri_sched()

        except KeyboardInterrupt:
            print(cotask.task_list)
            break
