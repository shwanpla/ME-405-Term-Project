"""
Motor control task with encoder data collection and state management.
Two-state FSM that initializes motors and continuously collects encoder position,
velocity, and timing data until end condition is met.

Hardware:
    - Motor: DC motor with H-bridge driver
    - Encoder: Quadrature encoder (1437.1 ticks/rev)
    - Timer: Microsecond precision timing

Notes:
    - State 0: Motor start/initialization with encoder zeroing
    - State 1: Continuous data collection and motor effort updates
    - Handles divide-by-zero errors in velocity calculations
    - Runs garbage collector during data collection to prevent memory issues
"""
from time import ticks_us, ticks_diff
import gc
import pyb
from pyb import Pin, Timer


class motor_control_task:
    """
    Motor control task for single motor with encoder feedback and data logging.
    """

    def __init__(self, motor, encoder):
        """
        Initialize motor control task.

        :param motor: Motor driver object
        :type motor: motor_driver
        :param encoder: Encoder object for position/velocity feedback
        :type encoder: Encoder
        """
        self.motor = motor
        self.encoder = encoder

    def run(self, shares):
        """
        Cooperative task function implementing motor control state machine.

        :param shares: Tuple of shared variables for inter-task communication
        :type shares: tuple
        """
        (start_flg, set_point, end_flg, enc_pos, enc_speed,
        exp_time, desired_time_ms, proc_data_flg, desired_vel) = shares

        S0_MOT_START, S1_COLL_DATA = 0, 1
        STATE = S0_MOT_START

        while True:
            if STATE == S0_MOT_START:
                if start_flg.get() == 1:
                    enc_pos.put(0)
                    enc_speed.put(0)
                    exp_time.put(0)

                    self.motor.enable()
                    self.encoder.zero()

                    enc_pos.put(self.encoder.get_position())
                    enc_speed.put(0)

                    time_elapsed = 0
                    time_0 = ticks_us()
                    exp_time.put(0)

                    start_flg.put(0)

                    STATE = S1_COLL_DATA

            elif STATE == S1_COLL_DATA:
                gc.collect()
                if end_flg.get() == 0:

                    self.encoder.update()
                    enc_pos.put(self.encoder.get_position())

                    try:
                        enc_speed.put(self.encoder.get_velocity())
                    except ZeroDivisionError:
                        enc_speed.put(0)

                    self.motor.set_effort(set_point.get())

                    time_now = ticks_us()
                    time_elapsed = ticks_diff(time_now, time_0)
                    exp_time.put(time_elapsed)

                    if time_elapsed >= desired_time_ms.get() * 1_000:
                        end_flg.put(1)
                        print("Time limit reached, toggling end_flg")
                elif end_flg.get() == 1:
                    self.motor.disable()
                    print("Motor disabled")
                    STATE = S0_MOT_START
            yield STATE
