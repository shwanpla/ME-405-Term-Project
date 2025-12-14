"""
Bump sensor monitoring task for collision detection.

Continuously polls a bump sensor on Pin PC11 and sets a shared flag when a collision
is detected. Uses debouncing to prevent false triggers and only detects the first
bump event per run.

Hardware:
    - Bump Sensor Pin: PC11 (active LOW, pull-up resistor enabled)
    - Debounce Time: 30ms

Notes:
    - Sensor is active LOW (pressed = 0, released = 1)
    - Only triggers once per run (bump_already_detected flag)
    - Requires cotask scheduler to yield control
"""

from pyb import Pin
from time import sleep_ms


def bump_sensor_task_fun(shares):
    """
    Cooperative task function that monitors bump sensor and updates shared state.

    :param shares: Tuple containing bump_detected_share (Share object)
    :type shares: tuple
    """
    (bump_detected_share,) = shares
    
    # Configure the bump sensor pin - PC11 as input with pull-up
    bump_pin = Pin(Pin.cpu.C11, mode=Pin.IN, pull=Pin.PULL_UP)
    
    # Track if we've already detected a bump
    bump_already_detected = False
    
    # Debounce parameters
    DEBOUNCE_MS = 30  # Debounce duration in milliseconds
    
    while True:
        # Read the current state of the bump sensor
        bump_state = bump_pin.value()
        
        # Most bump sensors are active LOW (pressed = 0, released = 1)
        # Change to: if bump_state == 1: if your sensor is active HIGH
        if bump_state == 0 and not bump_already_detected:
            # Debounce: wait to confirm the bump
            sleep_ms(DEBOUNCE_MS)
            if bump_pin.value() == 0:  # Confirm bump is still pressed
                bump_detected_share.put(1)
                bump_already_detected = True
                print("[BUMP] Bump sensor triggered!")
        
        # Yield to scheduler
        yield