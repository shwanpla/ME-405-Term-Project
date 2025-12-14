"""
DC motor driver using PWM control with directional control.
Implements H-bridge motor control with enable/disable and bidirectional drive capability.

Hardware:
    - H-Bridge: DRV8847 or similar motor driver IC
    - PWM Timer: Hardware timer with configurable frequency (typically 30kHz)
    - Control Pins: DIR (direction), nSLP (sleep/enable), PWM (speed)

Notes:
    - Positive effort = DIR low (forward direction)
    - Negative effort = DIR high (reverse direction)
    - Effort range: -100 to +100 (percent duty cycle)
    - nSLP pin must be high to enable motor driver
"""
from pyb import Pin, Timer


class motor_driver:
    """
    H-bridge motor driver with PWM speed control and directional control.
    """

    def __init__(self, PWM_pin: Pin, DIR_pin: Pin, nSLP_pin: Pin, tim: Timer, chan: int):
        """
        Initialize motor driver with PWM and control pins.

        :param PWM_pin: Pin for PWM signal
        :type PWM_pin: Pin
        :param DIR_pin: Pin for direction control
        :type DIR_pin: Pin
        :param nSLP_pin: Pin for enable/sleep control (active high)
        :type nSLP_pin: Pin
        :param tim: Timer object for PWM generation
        :type tim: Timer
        :param chan: Timer channel number for PWM output
        :type chan: int
        """
        self.DIR_pin = Pin(DIR_pin, mode=Pin.OUT_PP)
        self.nSLP_pin = Pin(nSLP_pin, mode=Pin.OUT_PP)
        self.PWM_chan = tim.channel(chan, pin=PWM_pin, mode=Timer.PWM, pulse_width_percent=0)

    def enable(self):
        """
        Enable the motor driver by setting nSLP pin high.
        """
        self.nSLP_pin.high()

    def disable(self):
        """
        Disable the motor driver by setting nSLP pin low.
        """
        self.nSLP_pin.low()

    def set_effort(self, effort: float):
        """
        Set motor effort with direction control.

        :param effort: Motor effort (-100 to +100, positive=forward, negative=reverse)
        :type effort: float
        """
        if effort > 0:
            self.DIR_pin.low()
            self.PWM_chan.pulse_width_percent(effort)
        else:
            self.DIR_pin.high()
            self.PWM_chan.pulse_width_percent(-effort)