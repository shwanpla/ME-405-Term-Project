"""
IMU-based obstacle course navigation state machine.

Implements a multi-state navigation task using:
    - Line following with adjustable bias for fork navigation
    - Heading control using observer-estimated IMU heading
    - Straight-line segments with heading hold
    - Placeholder for future waypoint navigation

This version uses the observer's X/Y position and heading from the BNO055-based
observer (not encoder-only odometry).

States:
    State 0:
        - Line following with bias switching based on X position
        - Starts with center bias, then switches to left bias at X threshold
    State 1:
        - Line following while turning robot to desired heading (e.g., 90°)
        - Transitions once heading error is within tolerance
    State 2:
        - Straight-line motion with IMU heading hold at 90°
    State 3:
        - Standard line following with zero bias after straight segment
    State 4:
        - Placeholder for future behavior (e.g., waypoint navigation)

Notes:
    - Transitions are driven by observer position (X, Y) and heading
    - Uses line-follow flag, bias share, and force-straight flag to coordinate
      with CL_control_task
"""

def navigation_task_fun(shares):
    """
    Cooperative task implementing IMU-based navigation FSM.

    The navigation task reads observer position and heading, then controls:
        - Line following enable/disable
        - Left/right line-follow bias
        - Heading setpoint for straight-line / turning segments
        - Flags for turn vs. straight modes

    Shares (tuple unpacked in order):
        line_follow_flg      : Enable/disable line following (0/1)
        big_X_share          : Observer X position (mm or arbitrary units)
        big_Y_share          : Observer Y position
        obs_heading_share    : Observer heading (deg)
        obs_yaw_rate_share   : Observer yaw rate (deg/s) [not used directly here]
        left_controller_share: Left CL_control object (not used directly here)
        right_controller_share: Right CL_control object (not used directly here)
        left_set_point       : Left motor effort setpoint (output of CL task)
        right_set_point      : Right motor effort setpoint (output of CL task)
        left_desired_vel     : Left wheel desired velocity
        right_desired_vel    : Right wheel desired velocity
        desired_angle_share  : Heading setpoint for straight-line / turn modes
        nav_rest_flg         : Navigation rest flag (unused here, reserved)
        end_flg              : Trial end flag (1 when trial complete)
        bias_share           : Line-follow bias value
        force_straight_flg   : Force-straight mode flag (1 = ignore line sensors)
        bias_timer_flg       : Bias-timer flag (unused here, reserved)
        nav_stop_flg         : Navigation stop flag (unused here, reserved)
        calibration_flg      : Observer calibration status (3 = ready)
        nav_turn_flg         : Turn-in-place vs. straight flag for CL task

    :param shares: Tuple of task_share.Share objects used for inter-task comms
    :type shares: tuple
    :yield: Current navigation state integer on each scheduler iteration
    :rtype: int
    """
    (line_follow_flg, big_X_share, big_Y_share,
     obs_heading_share, obs_yaw_rate_share,
     left_controller_share, right_controller_share,
     left_set_point, right_set_point,
     left_desired_vel, right_desired_vel,
     desired_angle_share, nav_rest_flg, end_flg, bias_share,
     force_straight_flg, bias_timer_flg, nav_stop_flg,
     calibration_flg, nav_turn_flg) = shares

    # ──────────────────────────────────────────────────────────────────────
    # State machine and configuration
    # ──────────────────────────────────────────────────────────────────────
    STATE = 0
    i = 0
    DEBUG = True

    # One-time transition flags
    bias_switched = False
    state_1_triggered = False
    state_2_triggered = False
    state_3_triggered = False
    observer_initialized = False  # Gate for enabling state transitions

    # Position thresholds (units match observer output)
    BIAS_SWITCH_X = 550.0   # Switch to left bias when X > 550
    STATE_2_Y = 685.0       # Threshold for entering heading-adjustment state
    STATE_3_Y = 500.0       # Threshold for final line-following state

    # Heading control tolerance (deg)
    HEADING_ERROR_TOLERANCE = 4.0

    # Bias values
    left_bias_value = 1.55
    center_bias_value = 0.0
    current_bias = 0.0

    # ──────────────────────────────────────────────────────────────────────
    # State helper functions
    # ──────────────────────────────────────────────────────────────────────

    def state_0_line_follow_bias_control(x_pos, y_pos, heading):
        """
        STATE 0: Line following with bias control.

        - Line following enabled
        - Bias = 0.0 initially, switches to left_bias_value after X threshold
        """
        nonlocal current_bias, i

        line_follow_flg.put(1)
        force_straight_flg.put(0)
        nav_turn_flg.put(0)

        if end_flg.get() == 0:
            current_bias = left_bias_value if bias_switched else center_bias_value
            bias_share.put(current_bias)

            i += 1
            if i % 10 == 0 and DEBUG:
                print(
                    "[NAV_TASK] STATE 0 - X: {:.1f} Y: {:.1f} "
                    "Heading: {:.1f}° Bias: {:.3f}".format(
                        x_pos, y_pos, heading, current_bias
                    )
                )

    def state_1_line_follow_heading_adjust(x_pos, y_pos, heading,
                                           desired_heading, heading_error):
        """
        STATE 1: Line following with heading adjustment.

        - Line following enabled
        - Bias forced to 0.0
        - nav_turn_flg used externally to allow CL task to steer toward
          desired_heading
        """
        nonlocal i

        line_follow_flg.put(1)
        force_straight_flg.put(0)
        bias_share.put(0.0)

        if end_flg.get() == 0:
            i += 1
            if i % 10 == 0 and DEBUG:
                print(
                    "[NAV_TASK] STATE 1 - X: {:.1f} Y: {:.1f} "
                    "Heading: {:.1f}° (Target: {:.1f}°) Error: {:.1f}°".format(
                        x_pos, y_pos, heading, desired_heading, heading_error
                    )
                )

    def state_2_straight_line(x_pos, y_pos, heading):
        """
        STATE 2: Straight-line segment at 90 degrees.

        - Line following disabled
        - Force-straight mode active
        - Heading set to 90 degrees
        """
        nonlocal i

        line_follow_flg.put(0)
        force_straight_flg.put(1)
        bias_share.put(0.0)
        desired_angle_share.put(90.0)

        if end_flg.get() == 0:
            i += 1
            if i % 10 == 0 and DEBUG:
                print(
                    "[NAV_TASK] STATE 2 - X: {:.1f} Y: {:.1f} "
                    "Heading: {:.1f}°".format(x_pos, y_pos, heading)
                )

    def state_3_line_following(x_pos, y_pos, heading):
        """
        STATE 3: Standard line following.

        - Line following enabled
        - Zero bias (center)
        - Used after straight heading-controlled segment
        """
        nonlocal i

        line_follow_flg.put(1)
        force_straight_flg.put(0)
        bias_share.put(0.0)
        nav_turn_flg.put(0)

        if end_flg.get() == 0:
            i += 1
            if i % 10 == 0 and DEBUG:
                print(
                    "[NAV_TASK] STATE 3 - X: {:.1f} Y: {:.1f} "
                    "Heading: {:.1f}°".format(x_pos, y_pos, heading)
                )

    def state_4_placeholder(x_pos, y_pos, heading):
        """
        STATE 4: Placeholder state.

        Reserved for future behaviors such as waypoint navigation or
        additional course features.
        """
        nonlocal i

        if end_flg.get() == 0:
            i += 1
            if i % 10 == 0:
                print(
                    "[NAV_TASK] STATE 4 - X: {:.1f} Y: {:.1f} "
                    "Heading: {:.1f}° (Placeholder)".format(
                        x_pos, y_pos, heading
                    )
                )

    # ──────────────────────────────────────────────────────────────────────
    # Main navigation loop
    # ──────────────────────────────────────────────────────────────────────
    while True:
        x_pos = big_X_share.get()
        y_pos = big_Y_share.get()
        heading = obs_heading_share.get()

        # Precompute heading error for heading-controlled states
        desired_heading = desired_angle_share.get()
        heading_error = abs(heading - desired_heading)
        if heading_error > 180:
            heading_error = 360 - heading_error

        # Only evaluate position-based transitions after observer is ready
        if calibration_flg.get() == 3:
            # Simple observer initialization gate (example: start when Y > 700)
            if not observer_initialized and y_pos > 700:
                observer_initialized = True

            if observer_initialized:
                # ── State transition logic ────────────────────────────────
                if STATE == 0:
                    # Switch bias at X threshold
                    if not bias_switched and x_pos > BIAS_SWITCH_X:
                        bias_switched = True
                        print(
                            "[NAV_TASK] X > {:.0f} - Switching to bias 1.55"
                            .format(BIAS_SWITCH_X)
                        )

                    # Transition to STATE 1 at Y threshold
                    if not state_1_triggered and y_pos < STATE_2_Y:
                        state_1_triggered = True
                        print(
                            "[NAV_TASK] Y < {:.0f} - Transitioning to STATE 1 "
                            "(heading adjustment)".format(STATE_2_Y)
                        )
                        desired_angle_share.put(90.0)
                        nav_turn_flg.put(1)
                        STATE = 1

                elif STATE == 1:
                    # Transition to STATE 2 when heading reached
                    if (not state_2_triggered and
                            heading_error <= HEADING_ERROR_TOLERANCE):
                        state_2_triggered = True
                        print(
                            "[NAV_TASK] Heading reached 90° - "
                            "Transitioning to STATE 2 (straight line)"
                        )
                        nav_turn_flg.put(0)
                        force_straight_flg.put(1)
                        STATE = 2

                elif STATE == 2:
                    # Transition to STATE 3 when Y threshold reached
                    if not state_3_triggered and y_pos < STATE_3_Y:
                        state_3_triggered = True
                        print(
                            "[NAV_TASK] Y < {:.0f} - Transitioning to STATE 3 "
                            "(line following)".format(STATE_3_Y)
                        )
                        force_straight_flg.put(0)
                        line_follow_flg.put(1)
                        STATE = 3

        # ── Execute current state behavior ────────────────────────────────
        if STATE == 0:
            state_0_line_follow_bias_control(x_pos, y_pos, heading)
        elif STATE == 1:
            state_1_line_follow_heading_adjust(
                x_pos, y_pos, heading, desired_heading, heading_error
            )
        elif STATE == 2:
            state_2_straight_line(x_pos, y_pos, heading)
        elif STATE == 3:
            state_3_line_following(x_pos, y_pos, heading)
        elif STATE == 4:
            state_4_placeholder(x_pos, y_pos, heading)

        yield STATE
