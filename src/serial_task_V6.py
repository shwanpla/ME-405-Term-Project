"""
Serial communication task for Bluetooth command interface.
Handles calibration commands, parameter setting, and trial control via UART.

Hardware:
    - UART: UART 1 at 460800 baud (Bluetooth module)
    - Protocol: ASCII text commands with newline termination

Notes:
    - Commands: PARAMS, CALIBRATE_BLACK, CALIBRATE_WHITE, CALIBRATION_COMPLETE, END_COMM
    - PARAMS format: "PARAMS,trial_time,desired_velocity"
    - Three-state FSM: Wait, Handle Trials, Send Data (unused)
    - Automatically starts motors after calibration completion
"""
import pyb
from pyb import Pin, Timer, UART
from time import ticks_ms, ticks_diff


def serial_task_fun(shares):
    """
    Cooperative task function for Bluetooth serial communication and control.

    :param shares: Tuple of shared variables for inter-task communication
    :type shares: tuple
    """
    (left_start_flg, right_start_flg,
    left_desired_vel, right_desired_vel,
    left_end_flg, right_end_flg,
    left_proc_data_flg, right_proc_data_flg,
    proc_pos_right, proc_pos_left,
    proc_speed_right, proc_speed_left,
    left_proc_time, right_proc_time,
    calibration_flg, observer_calibration_flg) = shares

    bt = UART(1, 460800)
    S0_WAIT, S1_HANDLE_TRIALS, S2_SEND_DATA = 0, 1, 2
    STATE = S0_WAIT

    desired_vel = 0
    left_proc_data_flg.put(0)
    right_proc_data_flg.put(0)

    i = 0

    while True:
        if STATE == S0_WAIT:
            if bt.any():
                line = bt.readline().decode().strip()
                print("Received:", line)
                if line.startswith("PARAMS"):
                    try:
                        parts = line.split(',')
                        trial_time = float(parts[1])
                        desired_vel = float(parts[2])

                        print("Velocity= ", desired_vel)

                        left_desired_vel.put(desired_vel)
                        right_desired_vel.put(desired_vel)

                    except Exception as e:
                        print("Error parsing parameters:", e)
                    while bt.any():
                        bt.read()

                elif line.startswith("CALIBRATE_BLACK"):
                    calibration_flg.put(1)
                elif line.startswith("CALIBRATE_WHITE"):
                    calibration_flg.put(2)
                elif line.startswith("CALIBRATION_COMPLETE"):
                    calibration_flg.put(3)
                    observer_calibration_flg.put(1)
                if calibration_flg.get() == 3:
                    left_start_flg.put(1)
                    right_start_flg.put(1)
                    STATE = S1_HANDLE_TRIALS

        elif STATE == S1_HANDLE_TRIALS:
            if bt.any():
                new_line = bt.readline().decode().strip()
                if new_line.startswith("END_COMM"):
                    left_end_flg.put(1)
                    right_end_flg.put(1)
                    left_start_flg.put(0)
                    right_start_flg.put(0)
            if left_end_flg.get() == 1 and right_end_flg.get() == 1:
                left_proc_time.clear()
                right_proc_time.clear()
                proc_pos_left.clear()
                proc_speed_left.clear()
                proc_pos_right.clear()
                proc_speed_right.clear()
                left_end_flg.put(0)
                right_end_flg.put(0)

                left_start_flg.put(0)
                right_start_flg.put(0)

                left_proc_data_flg.put(0)
                right_proc_data_flg.put(0)

                STATE = S0_WAIT

        yield STATE