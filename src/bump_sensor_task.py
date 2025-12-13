"""
Bump Sensor Task - Monitors a bump sensor on Pin PC11
Sets a share value to 1 once when the bump sensor is triggered (once per run)
"""

from pyb import Pin
from time import sleep_ms

def bump_sensor_task_fun(shares):
    """
    Monitors the bump sensor and sets bump_detected_share to 1 once when triggered.
    
    Parameters:
    -----------
    shares : tuple
        Contains:
        - bump_detected_share: Share('B') that gets set to 1 when bump is detected
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