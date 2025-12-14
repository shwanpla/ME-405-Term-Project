"""
Quadrature encoder interface using hardware timer with velocity averaging.
Handles 16-bit counter rollover and provides 5-point moving average velocity calculation.

Hardware:
    - Timer: Hardware timer in encoder mode (ENC_AB)
    - Channels: A and B quadrature signals
    - Counter Resolution: 16-bit (0-65535)

Notes:
    - Position increments are inverted (subtracted) for correct direction
    - Velocity averaged over 5 samples to reduce noise
    - Handles timer overflow/underflow automatically
"""

from pyb import Pin, Timer
from time import ticks_us, ticks_diff


class Encoder:
    """
    Quadrature encoder driver with position tracking and averaged velocity.
    """

    def __init__(self, tim, chA_pin, chB_pin):
        """
        Initialize encoder with hardware timer and channel pins.

        :param tim: Hardware timer object
        :type tim: Timer
        :param chA_pin: Pin for encoder channel A
        :type chA_pin: Pin
        :param chB_pin: Pin for encoder channel B
        :type chB_pin: Pin
        """
        self.position = 0
        self.prev_count = 0
        self.delta = 0
        self.timer = tim

        self.chA_pin = chA_pin
        self.chB_pin = chB_pin
        self.chA = self.timer.channel(1, Timer.ENC_AB, pin=self.chA_pin)
        self.chB = self.timer.channel(2, Timer.ENC_AB, pin=self.chB_pin)
        self.timer.counter(0)

        self.prev_count = self.timer.counter()
        self.prev_time = ticks_us()
        self.dt = 0

        self._vel_buf = [0.0] * 5
        self._idx = 0
        self._count = 0
        self._sum = 0.0

    def update(self):
        """
        Update position and velocity from timer counter with rollover handling.
        """
        current = self.timer.counter()
        delta = current - self.prev_count

        now = ticks_us()
        self.dt = ticks_diff(now, self.prev_time)
        self.prev_time = now

        AR = 0xFFFF
        if delta > AR // 2:
            delta -= AR + 1
        elif delta < -AR // 2:
            delta += AR + 1

        self.delta = delta
        self.position -= delta
        self.prev_count = current

        if self.dt > 0:
            vel_inst = -self.delta / self.dt
        else:
            vel_inst = 0.0

        old_val = self._vel_buf[self._idx]
        self._sum -= old_val
        self._vel_buf[self._idx] = vel_inst
        self._sum += vel_inst

        self._idx = (self._idx + 1) % 5
        if self._count < 5:
            self._count += 1

    def get_position(self):
        """
        Get current encoder position in ticks.

        :return: Accumulated position in ticks
        :rtype: int
        """
        return self.position

    def get_velocity(self):
        """
        Get 5-point averaged velocity.

        :return: Velocity in ticks per microsecond
        :rtype: float
        """
        if self._count == 0:
            return 0.0
        return self._sum / self._count

    def zero(self):
        """
        Reset encoder position to zero.
        """
        self.position = 0
        self.prev_count = self.timer.counter()
