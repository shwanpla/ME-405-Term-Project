"""
Odometry task for calculating robot heading and distance from encoder data.
Computes heading from encoder differential and distance from average encoder ticks.
Uses unwrapping to handle ±180° discontinuities smoothly.

Hardware:
    - Motor Encoders: 1437.1 ticks/rev
    - Wheel Radius: 35mm
    - Track Width: 141mm

Notes:
    - Positive heading = counter-clockwise rotation (left turn)
    - Heading normalized to [-180, 180] degrees
    - Distance calculated as average of left and right encoder ticks
"""

from math import pi


def encoder_heading_task_fun(shares):
    """
    Cooperative task function that computes heading and distance from encoders.

    :param shares: Tuple of (left_enc_pos, right_enc_pos, enc_heading_share, enc_distance_share)
    :type shares: tuple
    """
    (left_enc_pos, right_enc_pos, enc_heading_share, enc_distance_share) = shares

    WHEEL_RADIUS_M = 0.035
    TICKS_PER_REV = 1437.1
    TRACK_WIDTH_M = 0.141
    WHEEL_CIRC_M = 2 * pi * WHEEL_RADIUS_M
    WHEEL_CIRC_MM = WHEEL_CIRC_M * 1000
    MM_PER_TICK = WHEEL_CIRC_MM / TICKS_PER_REV

    prev_heading = 0.0
    continuous_heading = 0.0
    
    while True:
        left_ticks = left_enc_pos.get()
        right_ticks = right_enc_pos.get()

        avg_ticks = (left_ticks + right_ticks) / 2.0
        distance_mm = avg_ticks * MM_PER_TICK

        tick_diff = left_ticks - right_ticks
        heading_rad = (tick_diff / TICKS_PER_REV) * (WHEEL_CIRC_M / TRACK_WIDTH_M)
        heading_deg = heading_rad * (180.0 / pi)

        delta = heading_deg - prev_heading
        if delta > 180:
            delta -= 360
        elif delta < -180:
            delta += 360

        continuous_heading += delta
        prev_heading = heading_deg

        normalized_heading = continuous_heading
        while normalized_heading > 180:
            normalized_heading -= 360
        while normalized_heading < -180:
            normalized_heading += 360

        enc_heading_share.put(normalized_heading)
        enc_distance_share.put(distance_mm)

        yield