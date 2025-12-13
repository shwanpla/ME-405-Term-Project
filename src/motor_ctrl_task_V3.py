from time import ticks_us, ticks_diff
import pyb
from pyb import Pin, Timer
import gc

class motor_control_task:
    def __init__(self, motor, encoder):
        self.motor = motor
        self.encoder = encoder
        pass

    def run(self, shares):
        ''' Motor Control Task: Handles motor control and data collection'''
        (start_flg, set_point, end_flg, enc_pos, enc_speed, 
        exp_time, desired_time_ms, proc_data_flg, desired_vel) = shares
       
        S0_MOT_START, S1_COLL_DATA = 0, 1   #States of the FSM
        STATE = S0_MOT_START                                  # Set state to run first

        while True:            
            if STATE == S0_MOT_START:
                if start_flg.get() == 1:
                    # Clear all data queues before starting trial
                    enc_pos.put(0)
                    enc_speed.put(0)
                    exp_time.put(0)

                    self.motor.enable()                          # Enable motor
                    self.encoder.zero()                          # Zero encoder

                    #proc_data_flg.put(1)                        # Set process data flag to start processing
                    
                    enc_pos.put(self.encoder.get_position())   # Store initial encoder position
                    enc_speed.put(0)                            # Store initial encoder speed as 0
                    
                    # Handle time collection
                    time_elapsed = 0 # Initialize elapsed time
                    time_0 = ticks_us()
                    exp_time.put(0)

                    # Set effort for both motors and start motion
                    # self.motor.set_effort(desired_vel.get()/10)

                    start_flg.put(0)      # Reset start_flg flag for next trial

                    STATE = S1_COLL_DATA    # Set next state
            
            elif STATE == S1_COLL_DATA:
                gc.collect()
                if end_flg.get() == 0:

                    #Collect data from encoder
                    self.encoder.update()
                    enc_pos.put(self.encoder.get_position())

                    # Handle divide by 0 errors for velocity data
                    try: 
                        enc_speed.put(self.encoder.get_velocity())
                    except ZeroDivisionError:
                        enc_speed.put(0)
                    
                    #print(enc_speed.get())
                    self.motor.set_effort(set_point.get())  # Update effort from CL controller

                    # Collect time data
                    time_now = ticks_us()                              # Gets current time in us 
                    time_elapsed = ticks_diff(time_now, time_0)        # Subtracts initial time from current time
                    exp_time.put(time_elapsed)                         # Subtracts time now from first time measured
                    
                    if time_elapsed >= desired_time_ms.get() * 1_000:  # Check if desired time in microseconds has been reached
                        end_flg.put(1)                                 # Set end flag to 1 to stop data collection
                        print("Time limit reached, toggling end_flg")
                elif end_flg.get() == 1:
                    self.motor.disable()
                    print("Motor disabled")
                    #proc_data_flg.put(0)
                    STATE = S0_MOT_START
            yield STATE
