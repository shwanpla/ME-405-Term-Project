"""
Encoder Heading Task - Calculate heading and distance from encoders
Replaces observer_fcn.py and IMU tasks for scheduler efficiency.

Calculates:
1. Heading: Using encoder differential (left vs right ticks)
   - Positive heading = counter-clockwise (robot turning left)
   - Heading starts at 0° (+X direction) at robot startup
2. Distance: Average encoder ticks converted to mm traveled
   - avg_ticks = (left_ticks + right_ticks) / 2
   - distance_mm = avg_ticks * (wheel_circumference_mm / ticks_per_rev)
"""

from math import pi

def encoder_heading_task_fun(shares):
    """
    Task to calculate robot heading and distance from encoder ticks.
    
    Inputs:
    - left_enc_pos, right_enc_pos: Current encoder positions (ticks)
    
    Outputs:
    - enc_heading_share: Calculated heading in degrees (-180 to 180)
    - enc_distance_share: Distance traveled in mm
    """
    
    (left_enc_pos, right_enc_pos, enc_heading_share, enc_distance_share) = shares
    
    # Robot parameters
    WHEEL_RADIUS_M = 0.035  # meters
    TICKS_PER_REV = 1437.1  # encoder ticks per wheel revolution
    TRACK_WIDTH_M = 0.141   # distance between wheels (meters)
    WHEEL_CIRC_M = 2 * pi * WHEEL_RADIUS_M
    WHEEL_CIRC_MM = WHEEL_CIRC_M * 1000  # convert to mm
    
    # Conversion factors
    MM_PER_TICK = WHEEL_CIRC_MM / TICKS_PER_REV
    
    # State tracking
    prev_heading = 0.0  # Previous heading to detect wraparound
    continuous_heading = 0.0  # Unwrapped heading that can exceed [-180, 180]
    
    while True:
        # Read current encoder positions
        left_ticks = left_enc_pos.get()
        right_ticks = right_enc_pos.get()
        
        # Calculate average distance traveled (mm)
        avg_ticks = (left_ticks + right_ticks) / 2.0
        distance_mm = avg_ticks * MM_PER_TICK
        
        # Calculate heading from differential ticks
        # Positive difference means left wheel traveled more = turned left = positive heading
        tick_diff = left_ticks - right_ticks
        
        # Heading is proportional to the track width and tick difference
        # heading (rad) = (tick_diff / TICKS_PER_REV) * (WHEEL_CIRC / TRACK_WIDTH)
        # heading (deg) = (tick_diff / TICKS_PER_REV) * (WHEEL_CIRC / TRACK_WIDTH) * (180/pi)
        heading_rad = (tick_diff / TICKS_PER_REV) * (WHEEL_CIRC_M / TRACK_WIDTH_M)
        heading_deg = heading_rad * (180.0 / pi)
        
        # Unwrap heading to avoid jumps at ±180°
        delta = heading_deg - prev_heading
        if delta > 180:
            delta -= 360
        elif delta < -180:
            delta += 360
        
        continuous_heading += delta
        prev_heading = heading_deg
        
        # Normalize continuous heading to [-180, 180] for output
        normalized_heading = continuous_heading
        while normalized_heading > 180:
            normalized_heading -= 360
        while normalized_heading < -180:
            normalized_heading += 360
        
        # Output
        enc_heading_share.put(normalized_heading)
        enc_distance_share.put(distance_mm)
        
        yield