"""
Position-based navigation task for obstacle course using observer state estimation.

This module implements a cooperative multitasking navigation function that uses global
X, Y coordinates from the state observer and IMU heading to navigate an obstacle course.
The navigation task coordinates line following, heading control, and state transitions
based on position thresholds and heading targets.

States:
    - State 0: Line following with dynamic bias control based on X position
    - State 1: Line following with heading adjustment to 90 degrees
    - State 2: Straight line movement maintaining 90-degree heading
    - State 3: Standard line following with zero bias
    - State 4: Placeholder for future waypoint navigation

Hardware:
    - Observer: Provides global X, Y position estimates
    - IMU: Provides heading and yaw rate measurements
    - IR Sensor Array: Line detection for line following mode
    - Motors: Differential drive with velocity control

Notes:
    - Uses position-based state transitions (X, Y thresholds)
    - Observer calibration must complete before navigation begins
    - Heading control uses IMU-based angle with tolerance band
    - Bias values adjust line following behavior for curves
"""

def navigation_task_fun(shares):
    """
    Cooperative task function implementing position-based obstacle course navigation.

    Implements a 5-state finite state machine that uses observer-estimated X, Y coordinates
    and IMU heading to navigate the obstacle course. Coordinates line following, heading
    control, and straight-line movement based on position thresholds.

    Args:
        shares (tuple): Task shared variables containing navigation state and control
            - line_follow_flg (Share): Enable/disable line following (0 or 1)
            - big_X_share (Share): Observer X position estimate (mm)
            - big_Y_share (Share): Observer Y position estimate (mm)
            - obs_heading_share (Share): Observer heading estimate (degrees)
            - obs_yaw_rate_share (Share): Observer yaw rate estimate (deg/s)
            - left_controller_share (Share): Left motor controller object
            - right_controller_share (Share): Right motor controller object
            - left_set_point (Share): Left motor control effort output
            - right_set_point (Share): Right motor control effort output
            - left_desired_vel (Share): Left motor velocity setpoint
            - right_desired_vel (Share): Right motor velocity setpoint
            - desired_angle_share (Share): Target heading for heading control (degrees)
            - nav_rest_flg (Share): Navigation rest flag
            - end_flg (Share): End flag to stop navigation
            - bias_share (Share): Line following bias (-3 to +3)
            - force_straight_flg (Share): Force straight mode (disable line following)
            - bias_timer_flg (Share): Bias timer flag
            - nav_stop_flg (Share): Navigation stop flag
            - calibration_flg (Share): Observer calibration status (0-3)
            - nav_turn_flg (Share): Enable heading-based turning

    Yields:
        int: Current state number (0-4)

    State Machine:
        **State 0: Line Following with Dynamic Bias**
            - Initial line following mode
            - Bias = 0.0 initially, switches to 1.55 when X > 550mm
            - Transition: When Y < 685mm, move to State 1

        **State 1: Heading Adjustment**
            - Line following with simultaneous heading control
            - Target heading: 90 degrees
            - Bias reset to 0.0
            - nav_turn_flg enabled for heading control
            - Transition: When |heading_error| < 4°, move to State 2

        **State 2: Straight Line at 90 Degrees**
            - Disable line following, enable force_straight mode
            - Maintain 90-degree heading using heading control
            - Transition: When Y < 500mm, move to State 3

        **State 3: Standard Line Following**
            - Line following mode with zero bias
            - Force straight disabled
            - Final navigation state for course completion

        **State 4: Placeholder**
            - Reserved for future waypoint navigation
            - Currently inactive

    Position Thresholds:
        - BIAS_SWITCH_X = 550.0mm: Switch to left bias when exceeded
        - STATE_2_Y = 685.0mm: Transition to heading adjustment
        - STATE_3_Y = 500.0mm: Transition to final line following
        - HEADING_ERROR_TOLERANCE = 4.0°: Heading control tolerance

    Bias Values:
        - left_bias_value = 1.55: Bias for left curve navigation
        - center_bias_value = 0.0: No bias (centered line following)

    Observer Initialization:
        The task waits for observer calibration (calibration_flg == 3) and
        initial position estimate (Y > 700mm) before enabling state transitions.
        This prevents false transitions during startup.

    Control Flags:
        - line_follow_flg: Enables IR sensor-based line following
        - force_straight_flg: Disables line following, uses heading control only
        - nav_turn_flg: Enables heading-based turning control
        - bias_share: Adjusts line following steering (-3=left, 0=center, +3=right)

    Example Console Output:
        >>> [NAV_TASK] STATE 0 - X: 234.5 Y: 756.2 Heading: 12.3° Bias: 0.000
        >>> [NAV_TASK] X > 550 - Switching to bias 1.55
        >>> [NAV_TASK] STATE 0 - X: 567.8 Y: 723.1 Heading: 45.6° Bias: 1.550
        >>> [NAV_TASK] Y < 685 - Transitioning to STATE 1 (heading adjustment)
        >>> [NAV_TASK] STATE 1 - X: 589.2 Y: 672.3 Heading: 78.9° (Target: 90.0°) Error: 11.1°
        >>> [NAV_TASK] Heading reached 90° - Transitioning to STATE 2 (straight line)
        >>> [NAV_TASK] STATE 2 - X: 612.4 Y: 543.7 Heading: 91.2°
        >>> [NAV_TASK] Y < 500 - Transitioning to STATE 3 (line following)
        >>> [NAV_TASK] STATE 3 - X: 645.8 Y: 456.9 Heading: 88.7°

    Notes:
        - State transitions are one-way (cannot return to previous states)
        - Observer must complete calibration before navigation begins
        - Heading error calculation handles 360/0 wraparound
        - Debug printing occurs every 10th iteration to reduce output
    """
    (line_follow_flg, big_X_share, big_Y_share,
    obs_heading_share, obs_yaw_rate_share, 
    left_controller_share, right_controller_share,
    left_set_point, right_set_point, 
    left_desired_vel, right_desired_vel,
    desired_angle_share, nav_rest_flg, end_flg, bias_share, 
    force_straight_flg, bias_timer_flg, nav_stop_flg, calibration_flg, nav_turn_flg) = shares

    # State tracking
    STATE = 0
    i = 0
    DEBUG = True
    
    # One-time transition flags
    bias_switched = False
    state_1_triggered = False
    state_2_triggered = False
    state_3_triggered = False
    observer_initialized = False  # Gate for allowing first STATE 0 -> STATE 1 transition
    
    # Position thresholds
    BIAS_SWITCH_X = 550.0  # Switch to bias 1.55 when X > 550
    STATE_2_Y = 685.0  # Y threshold for entering straight line (Y < 685)
    STATE_3_Y = 500.0  # Y threshold for entering line following (Y < 500)
    
    # Heading control tolerance
    HEADING_ERROR_TOLERANCE = 4.0  # Degrees tolerance for heading control transition
    
    # Bias values
    left_bias_value = 1.55
    center_bias_value = 0.0
    current_bias = 0.0
    
    # STATE 0: Line following with bias control
    def state_0_line_follow_bias_control(x_pos, y_pos, heading):
        nonlocal current_bias, i
        
        line_follow_flg.put(1)
        force_straight_flg.put(0)
        nav_turn_flg.put(0)
        
        if end_flg.get() == 0:
            current_bias = left_bias_value if bias_switched else center_bias_value
            bias_share.put(current_bias)
            
            i += 1
            if i % 10 == 0 and DEBUG:
                print("[NAV_TASK] STATE 0 - X: {:.1f} Y: {:.1f} Heading: {:.1f}° Bias: {:.3f}".format(
                    x_pos, y_pos, heading, current_bias))

    # STATE 1: Line following with heading adjustment
    def state_1_line_follow_heading_adjust(x_pos, y_pos, heading, desired_heading, heading_error):
        nonlocal i
        
        line_follow_flg.put(1)
        force_straight_flg.put(0)
        bias_share.put(0.0)
        
        if end_flg.get() == 0:
            i += 1
            if i % 10 == 0 and DEBUG:
                print("[NAV_TASK] STATE 1 - X: {:.1f} Y: {:.1f} Heading: {:.1f}° (Target: {:.1f}°) Error: {:.1f}°".format(
                    x_pos, y_pos, heading, desired_heading, heading_error))

    # STATE 2: Straight line at 90 degrees
    def state_2_straight_line(x_pos, y_pos, heading):
        nonlocal i
        
        line_follow_flg.put(0)
        force_straight_flg.put(1)
        bias_share.put(0.0)
        desired_angle_share.put(90.0)
        
        if end_flg.get() == 0:
            i += 1
            if i % 10 == 0 and DEBUG:
                print("[NAV_TASK] STATE 2 - X: {:.1f} Y: {:.1f} Heading: {:.1f}°".format(
                    x_pos, y_pos, heading))

    # STATE 3: Standard line following
    def state_3_line_following(x_pos, y_pos, heading):
        nonlocal i
        
        line_follow_flg.put(1)
        force_straight_flg.put(0)
        bias_share.put(0.0)
        nav_turn_flg.put(0)
        
        if end_flg.get() == 0:
            i += 1
            if i % 10 == 0 and DEBUG:
                print("[NAV_TASK] STATE 3 - X: {:.1f} Y: {:.1f} Heading: {:.1f}°".format(
                    x_pos, y_pos, heading))

    # STATE 4: Placeholder
    def state_4_placeholder(x_pos, y_pos, heading):
        nonlocal i
        
        if end_flg.get() == 0:
            i += 1
            if i % 10 == 0:
                print("[NAV_TASK] STATE 4 - X: {:.1f} Y: {:.1f} Heading: {:.1f}° (Placeholder)".format(
                    x_pos, y_pos, heading))
    
    # Main control loop
    while True:
        x_pos = big_X_share.get()
        y_pos = big_Y_share.get()
        heading = obs_heading_share.get()
        
        # Pre-calculate values needed by state functions
        desired_heading = desired_angle_share.get()
        heading_error = abs(heading - desired_heading)
        if heading_error > 180:
            heading_error = 360 - heading_error
        
        # Only check position-based transitions after calibration
        if calibration_flg.get() == 3:
            # Set initialization flag once observer is ready (Y is at starting position)
            if not observer_initialized and y_pos > 700:
                observer_initialized = True
            
            # Now allow transitions if observer has been initialized
            if observer_initialized:
                # State transition logic based on position
                if STATE == 0:
                    # Switch bias at X threshold
                    if not bias_switched and x_pos > BIAS_SWITCH_X:
                        bias_switched = True
                        print("[NAV_TASK] X > {:.0f} - Switching to bias 1.55".format(BIAS_SWITCH_X))
                    
                    # Transition to STATE 1 at Y threshold
                    if not state_1_triggered and y_pos < STATE_2_Y:
                        state_1_triggered = True
                        print("[NAV_TASK] Y < {:.0f} - Transitioning to STATE 1 (heading adjustment)".format(STATE_2_Y))
                        desired_angle_share.put(90.0)
                        nav_turn_flg.put(1)
                        STATE = 1
                
                elif STATE == 1:
                    # Transition to STATE 2 when heading reached
                    if not state_2_triggered and heading_error <= HEADING_ERROR_TOLERANCE:
                        state_2_triggered = True
                        print("[NAV_TASK] Heading reached 90° - Transitioning to STATE 2 (straight line)")
                        nav_turn_flg.put(0)
                        force_straight_flg.put(1)
                        STATE = 2
                
                elif STATE == 2:
                    # Transition to STATE 3 when Y threshold reached
                    if not state_3_triggered and y_pos < STATE_3_Y:
                        state_3_triggered = True
                        print("[NAV_TASK] Y < {:.0f} - Transitioning to STATE 3 (line following)".format(STATE_3_Y))
                        force_straight_flg.put(0)
                        line_follow_flg.put(1)
                        STATE = 3
        
        # Execute current state
        if STATE == 0:
            state_0_line_follow_bias_control(x_pos, y_pos, heading)
        elif STATE == 1:
            state_1_line_follow_heading_adjust(x_pos, y_pos, heading, desired_heading, heading_error)
        elif STATE == 2:
            state_2_straight_line(x_pos, y_pos, heading)
        elif STATE == 3:
            state_3_line_following(x_pos, y_pos, heading)
        elif STATE == 4:
            state_4_placeholder(x_pos, y_pos, heading)
        
        yield STATE