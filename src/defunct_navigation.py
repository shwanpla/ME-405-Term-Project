"""
This task will implement different modes and settings for navigation at each stage of the course.
It will use the CL_control_task to control the motors based on the navigation requirements.

State 0: Start line following at bias = 0 for 10 seconds
State 1: Line following at bias = 1.55 for 3 seconds
State 2: Switch bias to 0, reorient to 90 degrees, go in straight line for 6 seconds, maintaining heading
State 3: Standard line following with bias = 0
State 4: Placeholder for future waypoint navigation
"""

def navigation_task_fun(shares):
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