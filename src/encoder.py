from pyb import Pin, Timer
from time import ticks_us, ticks_diff

class Encoder:
    """Quadrature encoder interface using a hardware timer with 5-point averaged velocity"""
    def __init__(self, tim, chA_pin, chB_pin):
        self.position   = 0
        self.prev_count = 0
        self.delta      = 0
        self.timer      = tim

        # Configure encoder pins
        self.chA_pin = chA_pin
        self.chB_pin = chB_pin
        self.chA = self.timer.channel(1, Timer.ENC_AB, pin=self.chA_pin)
        self.chB = self.timer.channel(2, Timer.ENC_AB, pin=self.chB_pin)
        self.timer.counter(0)

        # Initialize time tracking
        self.prev_count = self.timer.counter()
        self.prev_time  = ticks_us()
        self.dt         = 0

        # ── New: store recent velocity samples for averaging ────────────
        self._vel_buf = [0.0] * 5
        self._idx     = 0
        self._count   = 0
        self._sum     = 0.0

    def update(self):
        """Read timer counter, handle rollover, update position and store new velocity"""
        current = self.timer.counter()
        delta = current - self.prev_count

        now = ticks_us()
        self.dt = ticks_diff(now, self.prev_time)
        self.prev_time = now

        # Correct for overflow/underflow of 16-bit counter
        AR = 0xFFFF
        if delta > AR // 2:
            delta -= AR + 1
        elif delta < -AR // 2:
            delta += AR + 1

        self.delta = delta
        self.position -= delta
        self.prev_count = current

        # ── Update 5-point moving average buffer ───────────────────────
        if self.dt > 0:
            vel_inst = -self.delta / self.dt
        else:
            vel_inst = 0.0

        # Replace old value in the sum
        old_val = self._vel_buf[self._idx]
        self._sum -= old_val
        self._vel_buf[self._idx] = vel_inst
        self._sum += vel_inst

        # Increment buffer index and count
        self._idx = (self._idx + 1) % 5
        if self._count < 5:
            self._count += 1

    def get_position(self):
        return self.position

    def get_velocity(self):
        """Return 5-point averaged velocity in ticks per microsecond"""
        if self._count == 0:
            return 0.0
        return self._sum / self._count

    def zero(self):
        """Zero the encoder position"""
        self.position = 0
        self.prev_count = self.timer.counter()
