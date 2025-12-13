import pyb
from pyb import Pin, Timer, UART
from time import ticks_ms, ticks_diff

def serial_task_fun(shares):
    (left_start_flg, right_start_flg,
    left_desired_vel, right_desired_vel,
    left_end_flg, right_end_flg,
    left_proc_data_flg, right_proc_data_flg,
    proc_pos_right, proc_pos_left,
    proc_speed_right, proc_speed_left, 
    left_proc_time, right_proc_time,
    calibration_flg, observer_calibration_flg) = shares

    bt = UART(1, 460800)            # Initialie UART 1 for Bluetooth serial communication
    S0_WAIT, S1_HANDLE_TRIALS, S2_SEND_DATA = 0, 1, 2    # States of the FSM
    STATE = S0_WAIT                 # Set initial state

    # Initialize variables
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
                    """Take in params for bluetooth trials and line following
                        1. Desired velocity
                        """
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
            #print("Big X: ", big_X_share.get(), " Big Y: ", big_Y_share.get())
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



 # elif STATE == S2_SEND_DATA:
        #     if _header == 0:
        #         header = (f"Time, IR_1, IR_2, IR_3, IR_4, IR_5, IR_6, IR_7\r\n")
        #         bt.write(header.encode())
        #         _header = 1
            
        #     # Send data while either queue has data
        #     if left_proc_time.any() or right_proc_time.any():
        #         # Get data from each queue if available, otherwise use last known value or 0
        #         tl = left_proc_time.get()
        #         S_1 = IR_1.get()
        #         S_2 = IR_2.get()
        #         S_3 = IR_3.get()
        #         S_4 = IR_4.get()
        #         S_5 = IR_5.get()
        #         S_6 = IR_6.get()
        #         S_7 = IR_7.get()
                

        #         bt.write(f"{tl},{S_1},{S_2},{S_3},{S_4},{S_5},{S_6},{S_7}\r\n".encode())
            
        #     else:   # Both queues are empty - all done
        #         # Clear all processed data queues for next trial
        #         left_proc_time.clear()
        #         right_proc_time.clear()
        #         proc_pos_left.clear()
        #         proc_speed_left.clear()
        #         proc_pos_right.clear()
        #         proc_speed_right.clear()
        #         left_end_flg.put(0)
        #         right_end_flg.put(0)
                
        #         left_start_flg.put(0)
        #         right_start_flg.put(0)
                
        #         left_proc_data_flg.put(0)
        #         right_proc_data_flg.put(0)

        #         print("Data collection finished")
        #         bt.write(b'DONE')
        #         _header = 0

        #         STATE = S0_WAIT