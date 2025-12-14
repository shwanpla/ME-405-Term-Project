"""
Main program for autonomous obstacle course navigation robot.
Implements cooperative multitasking scheduler with encoder-based odometry,
line following, PI velocity control, and bump sensor collision detection.

Hardware:
    - Motors: 2x DC motors with encoders (1437.1 ticks/rev)
    - IR Sensor Array: 7 reflectance sensors for line detection
    - Bump Sensor: Collision detection on Pin PC11
    - Wheel Diameter: 70mm (35mm radius)
    - Track Width: 141mm

Notes:
    - Uses cotask cooperative scheduler with priority-based task execution
    - Velocity control runs at 12ms period, closed-loop control at 20ms
    - Encoder heading task computes robot orientation from differential encoders
    - Navigation task coordinates line following, turns, and obstacle avoidance
    - Serial task provides user interface for calibration and control
"""
import gc
from time import ticks_us, ticks_diff

import pyb
from pyb import Pin, Timer, UART
import cotask
import task_share

from motor import motor_driver
from encoder import Encoder
from multi_sensor_read import multiple_ir_readings
import motor_ctrl_task_V3 as motor_ctrl_task
import CL_control_task_V5 as CL_control_task
from serial_task_V6 import serial_task_fun
from encoder_heading_task import encoder_heading_task_fun
from navigation_v2_compact_encoder import navigation_task_fun
from bump_sensor_task import bump_sensor_task_fun

# Creating Shares and Queues
#----------------------------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    print("\n\nStarting Simple Navigation with Bump Sensor...\n")
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
    left_start_flg = task_share.Share('B', thread_protect = False, name = 'left_start_flg')
    right_start_flg = task_share.Share('B', thread_protect = False, name = 'right_start_flg')
    
    left_end_flg = task_share.Share('B', thread_protect = False, name = 'left_end_flg')
    right_end_flg = task_share.Share('B', thread_protect = False, name = 'right_end_flg')
   
    left_effort_lvl = task_share.Share('h', thread_protect = False, name = 'left_effort_lvl')
    right_effort_lvl = task_share.Share('h', thread_protect = False, name = 'right_effort_lvl')

    left_desired_vel = task_share.Share('f', thread_protect = False, name = 'left_desired_vel')
    right_desired_vel = task_share.Share('f', thread_protect = False, name = 'right_desired_vel')

    left_set_point = task_share.Share('f', thread_protect = False, name = 'left_set_point')
    right_set_point = task_share.Share('f', thread_protect = False, name = 'right_set_point')

    desired_time_ms = task_share.Share('f', thread_protect = False, name = 'desired_time_ms')
    desired_time_ms.put(75_000*1000)
    
    left_proc_data_flg = task_share.Share('B', thread_protect = False, name = 'left_proc_data_flg')
    right_proc_data_flg = task_share.Share('B', thread_protect = False, name = 'right_proc_data_flg')

    kp_share = task_share.Share('f', thread_protect = False, name = 'kp_share')
    ki_share = task_share.Share('f', thread_protect = False, name = 'ki_share')

    # Encoder-based navigation shares
    enc_heading_share = task_share.Share('f', thread_protect=False, name='enc_heading')
    enc_heading_share.put(0.0)
    enc_distance_share = task_share.Share('f', thread_protect=False, name='enc_distance')
    enc_distance_share.put(0.0)

    calibration_flg = task_share.Share('B', thread_protect = False, name = 'calibration_flg')
    observer_calibration_flg = task_share.Share('B', thread_protect = False, name = 'observer_calibration_flg')
    observer_calibration_flg.put(0)
    
    # Navigation stuff
    line_follow_flg = task_share.Share('B', thread_protect=False, name='line_follow_flg')
    line_follow_flg.put(0)
    
    force_straight_flg = task_share.Share('B', thread_protect=False, name='force_straight_flg')
    force_straight_flg.put(0)
    
    # Bump sensor share - gets set to 1 when bump detected
    bump_detected_share = task_share.Share('B', thread_protect=False, name='bump_detected_share')
    bump_detected_share.put(0)
    
    bias_timer_flg = task_share.Share('B', thread_protect=False, name='bias_timer_flg')
    bias_timer_flg.put(0)

    nav_stop_flg = task_share.Share('B', thread_protect=False, name='nav_stop_flg')
    nav_stop_flg.put(0)

    # Queues used to pass raw data from trials
    left_enc_speed = task_share.Share('f', thread_protect = False, name = 'left_enc_speed')
    right_enc_speed = task_share.Share('f', thread_protect = False, name = 'right_enc_speed')
    left_enc_pos = task_share.Share('f', thread_protect = False, name = 'left_enc_pos')
    right_enc_pos = task_share.Share('f', thread_protect = False, name = 'right_enc_pos')
    left_exp_time = task_share.Share('f', thread_protect = False, name = 'left_exp_time')
    right_exp_time = task_share.Share('f', thread_protect = False, name = 'right_exp_time')
    
    # Queues used to pass processed data from trials
    proc_speed_left = task_share.Queue('f', 10, thread_protect = False, 
                                 overwrite = False, name = 'proc_speed_left')
    proc_speed_right = task_share.Queue('f', 10, thread_protect = False, 
                                 overwrite = False, name = 'proc_speed_right')
    proc_pos_left = task_share.Queue('f', 10, thread_protect = False, 
                               overwrite = False, name = 'proc_pos_left')
    proc_pos_right = task_share.Queue('f', 10, thread_protect = False, 
                               overwrite = False, name = 'proc_pos_right')
    left_proc_time = task_share.Queue('f', 10, thread_protect = False, 
                               overwrite = False, name = 'left_proc_time')
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
                                                            calibration_flg, observer_calibration_flg))
    
    
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
    
    
    # Additional shares needed for closed-loop control
    desired_angle_share = task_share.Share('f', thread_protect=False, name='desired_angle_share')
    desired_angle_share.put(0.0)
    
    nav_rest_flg = task_share.Share('B', thread_protect=False, name='nav_rest_flg')
    nav_rest_flg.put(0)
    
    nav_turn_flg = task_share.Share('B', thread_protect=False, name='nav_turn_flg')
    nav_turn_flg.put(0)
    
    bias_share = task_share.Share('f', thread_protect=False, name='bias_share')
    bias_share.put(0.0)

    left_CL_control_task = cotask.Task(left_controller.run, name="left_CL_ctrl", priority=3, period=20,
                                profile=True, trace=False, shares=(left_desired_vel, left_enc_speed, left_set_point, left_end_flg, 
                                                                   ki_share, kp_share, calibration_flg, 
                                                                   line_follow_flg, desired_angle_share, enc_heading_share,
                                                                   nav_rest_flg, nav_turn_flg, bias_share, force_straight_flg))
    
    right_CL_control_task = cotask.Task(right_controller.run, name="right_CL_ctrl", priority=3, period=20,
                                profile=True, trace=False, shares=(right_desired_vel, right_enc_speed, right_set_point, 
                                                                   right_end_flg, ki_share, kp_share, calibration_flg,
                                                                   line_follow_flg, desired_angle_share, enc_heading_share,
                                                                   nav_rest_flg, nav_turn_flg, bias_share, force_straight_flg))
    
    encoder_heading_task = cotask.Task(encoder_heading_task_fun, name="enc_heading", priority=3, period=50,
                                       profile=True, trace=False, shares=(left_enc_pos, right_enc_pos, enc_heading_share, enc_distance_share))
    
    # Navigation task - obstacle course with bump sensor support
    navigation_task = cotask.Task(navigation_task_fun, name="navigation", priority=4, period=50,
                                 profile=True, trace=False, shares=(line_follow_flg, enc_heading_share, enc_distance_share,
                                                                     left_controller_share, right_controller_share,
                                                                     left_set_point, right_set_point, 
                                                                     left_desired_vel, right_desired_vel,
                                                                     desired_angle_share, nav_rest_flg, left_end_flg, bias_share,
                                                                     force_straight_flg, bias_timer_flg, nav_stop_flg, calibration_flg, nav_turn_flg,
                                                                     bump_detected_share))
    
    # Bump sensor task
    bump_sensor_task = cotask.Task(bump_sensor_task_fun, name="bump_sensor", priority=3, period=20,
                                  profile=True, trace=False, shares=(bump_detected_share,))
    
    # Add tasks to the scheduler
    cotask.task_list.append(serial_task)
    cotask.task_list.append(left_mot_ctrl_task)
    cotask.task_list.append(right_mot_ctrl_task)
    cotask.task_list.append(left_CL_control_task)
    cotask.task_list.append(right_CL_control_task)
    cotask.task_list.append(encoder_heading_task)
    cotask.task_list.append(navigation_task)
    cotask.task_list.append(bump_sensor_task)

    # Run garbage collector
    gc.collect()

    # Start the scheduler
    while True:
        try:
            cotask.task_list.pri_sched()

        except KeyboardInterrupt:
            print(cotask.task_list)
            break