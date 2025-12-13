"""
Compact Obstacle Course Navigation - Encoder-Based (optimized for scheduler performance)
Uses encoder distance (mm) instead of state estimation X/Y coordinates.
Uses encoder-calculated heading instead of IMU.

Course Layout (Main Lap):
- State 0-10: Main obstacle course
- State 11: Exit section line following
- State 12: Turn to 180°
- State 13: Go straight at 180° through parking garage until distance > 4474mm
- State 14: Turn to 175°
- State 15: Go straight at 175° until bump detected

Wall Interaction Recovery Sequence (States 16-23):
- State 16: Back up 50mm
- State 17: Turn to 0°
- State 18: Go straight 175mm at 0°
- State 19: Turn to 175°
- State 20: Go straight 75mm at 175°
- State 21: Turn to 180°
- State 22: Line following at bias=0 for 200mm
- State 23: Stop
"""

def navigation_task_fun(shares):
    (line_follow_flg, enc_heading_share, enc_distance_share,
    left_controller_share, right_controller_share,
    left_set_point, right_set_point, 
    left_desired_vel, right_desired_vel,
    desired_angle_share, nav_rest_flg, end_flg, bias_share, 
    force_straight_flg, bias_timer_flg, nav_stop_flg, calibration_flg, nav_turn_flg,
    bump_detected_share) = shares

    # ════════════════════════════════════════════════════════════════════════════════
    # CONFIGURABLE PARAMETERS - Distance thresholds in MM (measured from obstacle course)
    # ════════════════════════════════════════════════════════════════════════════════
    
    # STRAIGHT INTO FORK
    STATE_0_DIST = 650.0                                        # Start to fork entrance
    STATE_1_DIST = STATE_0_DIST + (2*3.14*200/4) - 50          # Fork quarter-circle
    
    # STRAIGHT THRU DIAMOND
    STATE_3_DIST = STATE_1_DIST + 140                          # Straight section after -90° turn
    
    # U-TURN AND STRAIGHT
    STATE_4_DIST = STATE_3_DIST + (2*3.14*200/4) + 225         # U-turn semicircle
    STATE_6_DIST = STATE_4_DIST + 240                          # Straight section before 90° turn
    
    # SECOND FORK AND LARGE U TURN
    STATE_7_DIST = STATE_6_DIST + (2*3.14*150/4) + 20          # Second fork quarter-circle
    STATE_8_DIST = STATE_7_DIST + (2*3.14*325/4) + 150 + (2*3.14*225/4) + 125  # Two quarter circles with straights
    
    # STRAIGHT ACROSS ZIG ZAG
    STATE_10_DIST = STATE_8_DIST + 285                         # Approach to finish line
    
    # PARKING GARAGE
    STATE_11_DIST = STATE_10_DIST + 225                        # Enter parking garage
    STATE_13_DIST = STATE_11_DIST + 700                        # Through parking garage
    STATE_15_DIST = STATE_13_DIST + 350                        # Exit parking garage
    
    # WALL INTERACTION RECOVERY SEQUENCE (triggered by bump sensor at STATE 15)
    STATE_16_BACKUP = 50                                        # Back up distance (mm)
    STATE_17_TURN = 0.0                                         # Turn to 0 degrees
    STATE_18_STRAIGHT = 175                                     # Go straight 175mm at 0°
    STATE_19_TURN = 90.0                                        # Turn to 90 degrees
    STATE_20_STRAIGHT = 200                                     # Go straight 200mm at 90°
    STATE_21_TURN = 180.0                                       # Turn to 180 degrees
    STATE_22_STRAIGHT = 200                                     # Go straight 200mm at 180°
    
    # Heading targets (FLIPPED due to encoder direction)
    HEADING_NEG_90 = -90.0    # Was 90 in physical orientation
    HEADING_90 = 90.0         # Was -90 in physical orientation
    HEADING_180 = 180.0
    HEADING_TOLERANCE = 4.0   # Accept heading within ±4° of target
    
    STATE = 0
    observer_initialized = False
    transition_flags = [False] * 24  # Flags for 24 states (0-23)
    saved_velocity = 0
    state_heading_set = False
    state_start_distances = {}  # Track start distance for each state
    
    def calc_heading_error(current_heading, target_heading):
        """Calculate normalized heading error (-180 to 180)"""
        error = target_heading - current_heading
        while error > 180:
            error -= 360
        while error < -180:
            error += 360
        return error
    
    while True:
        distance = enc_distance_share.get()
        heading = enc_heading_share.get()
        
        # Gate: wait for calibration and some motion
        if calibration_flg.get() == 3:
            if not observer_initialized and distance > 50:
                observer_initialized = True
                print(f"[NAV] Initialized - Distance={distance:.0f}mm, Heading={heading:.1f}°")
            
            if observer_initialized:
                # ════════════════════════════════════════════════════════════════════════════
                # STATE 0: Line following at bias=0 until distance > 650mm
                # ════════════════════════════════════════════════════════════════════════════
                if STATE == 0:
                    line_follow_flg.put(1)
                    force_straight_flg.put(0)
                    nav_turn_flg.put(0)
                    bias_share.put(0.0)
                    
                    if not transition_flags[0] and distance > STATE_0_DIST:
                        transition_flags[0] = True
                        print(f"[NAV] STATE 0→1: Distance {distance:.0f}mm > {STATE_0_DIST:.0f}mm - Entering fork")
                        STATE = 1
                    else:
                        print(f"[NAV] STATE 0 - Distance={distance:.0f}mm, Heading={heading:.1f}°")
                
                # ════════════════════════════════════════════════════════════════════════════
                # STATE 1: Line following at bias=1.55 (fork quarter-circle) until distance > 914mm
                # ════════════════════════════════════════════════════════════════════════════
                elif STATE == 1:
                    line_follow_flg.put(1)
                    force_straight_flg.put(0)
                    nav_turn_flg.put(0)
                    bias_share.put(1.55)
                    
                    if not transition_flags[1] and distance > STATE_1_DIST:
                        transition_flags[1] = True
                        saved_velocity = left_desired_vel.get()
                        state_heading_set = False
                        print(f"[NAV] STATE 1→2: Distance {distance:.0f}mm > {STATE_1_DIST:.0f}mm - Fork quarter-circle complete")
                        STATE = 2
                    else:
                        print(f"[NAV] STATE 1 - Distance={distance:.0f}mm, Heading={heading:.1f}°")
                
                # ════════════════════════════════════════════════════════════════════════════
                # STATE 2: Turn to -90° (flipped heading)
                # ════════════════════════════════════════════════════════════════════════════
                elif STATE == 2:
                    if not state_heading_set:
                        desired_angle_share.put(HEADING_NEG_90)
                        state_heading_set = True
                    
                    line_follow_flg.put(0)
                    force_straight_flg.put(1)
                    nav_turn_flg.put(1)
                    bias_share.put(0.0)
                    left_desired_vel.put(0)
                    right_desired_vel.put(0)
                    
                    heading_error = calc_heading_error(heading, HEADING_NEG_90)
                    if not transition_flags[2] and abs(heading_error) < HEADING_TOLERANCE:
                        transition_flags[2] = True
                        state_heading_set = False
                        left_desired_vel.put(saved_velocity)
                        right_desired_vel.put(saved_velocity)
                        print(f"[NAV] STATE 2→3: Heading {heading:.1f}° reached {HEADING_NEG_90}° - Ready for straight section")
                        STATE = 3
                    else:
                        print(f"[NAV] STATE 2 - Heading={heading:.1f}° (Target: {HEADING_NEG_90}°)")
                
                # ════════════════════════════════════════════════════════════════════════════
                # STATE 3: Go straight at -90° (150mm) until distance > 1114mm
                # ════════════════════════════════════════════════════════════════════════════
                elif STATE == 3:
                    line_follow_flg.put(0)
                    force_straight_flg.put(1)
                    nav_turn_flg.put(0)
                    bias_share.put(0.0)
                    desired_angle_share.put(HEADING_NEG_90)
                    
                    # Allow heading to drift by +/- 5 degrees (3 + 2) during straight
                    heading_error = calc_heading_error(heading, HEADING_NEG_90)
                    if abs(heading_error) > 5.0:
                        bias_share.put(heading_error * 0.1)
                    
                    if not transition_flags[3] and distance > STATE_3_DIST:
                        transition_flags[3] = True
                        print(f"[NAV] STATE 3→4: Distance {distance:.0f}mm > {STATE_3_DIST:.0f}mm - Entering U-turn")
                        STATE = 4
                    else:
                        print(f"[NAV] STATE 3 - Distance={distance:.0f}mm, Heading={heading:.1f}°")
                
                # ════════════════════════════════════════════════════════════════════════════
                # STATE 4: Line following at bias=0 (U-turn semicircle) until distance > 1742mm
                # ════════════════════════════════════════════════════════════════════════════
                elif STATE == 4:
                    line_follow_flg.put(1)
                    force_straight_flg.put(0)
                    nav_turn_flg.put(0)
                    bias_share.put(0.0)
                    
                    if not transition_flags[4] and distance > STATE_4_DIST:
                        transition_flags[4] = True
                        saved_velocity = left_desired_vel.get()
                        state_heading_set = False
                        print(f"[NAV] STATE 4→5: Distance {distance:.0f}mm > {STATE_4_DIST:.0f}mm - U-turn complete")
                        STATE = 5
                    else:
                        print(f"[NAV] STATE 4 - Distance={distance:.0f}mm, Heading={heading:.1f}°")
                
                # ════════════════════════════════════════════════════════════════════════════
                # STATE 5: Turn to 90° (flipped heading)
                # ════════════════════════════════════════════════════════════════════════════
                elif STATE == 5:
                    if not state_heading_set:
                        desired_angle_share.put(HEADING_90)
                        state_heading_set = True
                    
                    line_follow_flg.put(0)
                    force_straight_flg.put(1)
                    nav_turn_flg.put(1)
                    bias_share.put(0.0)
                    left_desired_vel.put(0)
                    right_desired_vel.put(0)
                    
                    heading_error = calc_heading_error(heading, HEADING_90)
                    if not transition_flags[5] and abs(heading_error) < HEADING_TOLERANCE:
                        transition_flags[5] = True
                        state_heading_set = False
                        left_desired_vel.put(saved_velocity)
                        right_desired_vel.put(saved_velocity)
                        print(f"[NAV] STATE 5→6: Heading {heading:.1f}° reached {HEADING_90}° - Ready for straight section")
                        STATE = 6
                    else:
                        print(f"[NAV] STATE 5 - Heading={heading:.1f}° (Target: {HEADING_90}°)")
                
                # ════════════════════════════════════════════════════════════════════════════
                # STATE 6: Go straight at 90° (250mm) until distance > 1992mm
                # ════════════════════════════════════════════════════════════════════════════
                elif STATE == 6:
                    line_follow_flg.put(0)
                    force_straight_flg.put(1)
                    nav_turn_flg.put(0)
                    bias_share.put(0.0)
                    desired_angle_share.put(HEADING_90)
                    
                    # Allow heading to drift by +/- 5 degrees (3 + 2) during straight
                    heading_error = calc_heading_error(heading, HEADING_90)
                    if abs(heading_error) > 5.0:
                        bias_share.put(heading_error * 0.1)
                    
                    if not transition_flags[6] and distance > STATE_6_DIST:
                        transition_flags[6] = True
                        print(f"[NAV] STATE 6→7: Distance {distance:.0f}mm > {STATE_6_DIST:.0f}mm - Entering second fork")
                        STATE = 7
                    else:
                        print(f"[NAV] STATE 6 - Distance={distance:.0f}mm, Heading={heading:.1f}°")
                
                # ════════════════════════════════════════════════════════════════════════════
                # STATE 7: Line following at bias=1.8 (90° quarter-circle) until distance > 2227mm
                # ════════════════════════════════════════════════════════════════════════════
                elif STATE == 7:
                    line_follow_flg.put(1)
                    force_straight_flg.put(0)
                    nav_turn_flg.put(0)
                    bias_share.put(1.8)
                    
                    if not transition_flags[7] and distance > STATE_7_DIST:
                        transition_flags[7] = True
                        print(f"[NAV] STATE 7→8: Distance {distance:.0f}mm > {STATE_7_DIST:.0f}mm - Second fork quarter-circle complete")
                        STATE = 8
                    else:
                        print(f"[NAV] STATE 7 - Distance={distance:.0f}mm, Heading={heading:.1f}°")
                
                # ════════════════════════════════════════════════════════════════════════════
                # STATE 8: Line following at bias=0 (complex section: 2x quarter circles + straights) until distance > 3549mm
                # ════════════════════════════════════════════════════════════════════════════
                elif STATE == 8:
                    line_follow_flg.put(1)
                    force_straight_flg.put(0)
                    nav_turn_flg.put(0)
                    bias_share.put(0.0)
                    
                    if not transition_flags[8] and distance > STATE_8_DIST:
                        transition_flags[8] = True
                        saved_velocity = left_desired_vel.get()
                        state_heading_set = False
                        print(f"[NAV] STATE 8→9: Distance {distance:.0f}mm > {STATE_8_DIST:.0f}mm - Complex section complete")
                        STATE = 9
                    else:
                        print(f"[NAV] STATE 8 - Distance={distance:.0f}mm, Heading={heading:.1f}°")
                
                # ════════════════════════════════════════════════════════════════════════════
                # STATE 9: Turn to 180°
                # ════════════════════════════════════════════════════════════════════════════
                elif STATE == 9:
                    if not state_heading_set:
                        desired_angle_share.put(HEADING_180)
                        state_heading_set = True
                    
                    line_follow_flg.put(0)
                    force_straight_flg.put(1)
                    nav_turn_flg.put(1)
                    bias_share.put(0.0)
                    left_desired_vel.put(0)
                    right_desired_vel.put(0)
                    
                    heading_error = calc_heading_error(heading, HEADING_180)
                    if not transition_flags[9] and abs(heading_error) < HEADING_TOLERANCE:
                        transition_flags[9] = True
                        state_heading_set = False
                        left_desired_vel.put(saved_velocity)
                        right_desired_vel.put(saved_velocity)
                        print(f"[NAV] STATE 9→10: Heading {heading:.1f}° reached {HEADING_180}° - Ready for final straight")
                        STATE = 10
                    else:
                        print(f"[NAV] STATE 9 - Heading={heading:.1f}° (Target: {HEADING_180}°)")
                
                # ════════════════════════════════════════════════════════════════════════════
                # STATE 10: Go straight at 180° (300mm approach) until distance > 3849mm
                # ════════════════════════════════════════════════════════════════════════════
                elif STATE == 10:
                    line_follow_flg.put(0)
                    force_straight_flg.put(1)
                    nav_turn_flg.put(0)
                    bias_share.put(0.0)
                    desired_angle_share.put(HEADING_180)
                    
                    if not transition_flags[10] and distance > STATE_10_DIST:
                        transition_flags[10] = True
                        saved_velocity = left_desired_vel.get()
                        state_heading_set = False
                        print(f"[NAV] STATE 10→11: Distance {distance:.0f}mm > {STATE_10_DIST:.0f}mm - Starting second lap")
                        STATE = 11
                    else:
                        print(f"[NAV] STATE 10 - Distance={distance:.0f}mm, Heading={heading:.1f}°")
                
                # ════════════════════════════════════════════════════════════════════════════
                # STATE 11: Line following at bias=0.2 (exit section ~250mm) until distance > 4099mm
                # ════════════════════════════════════════════════════════════════════════════
                elif STATE == 11:
                    line_follow_flg.put(1)
                    force_straight_flg.put(0)
                    nav_turn_flg.put(0)
                    bias_share.put(0.2)
                    
                    if not transition_flags[11] and distance > STATE_11_DIST:
                        transition_flags[11] = True
                        saved_velocity = left_desired_vel.get()
                        state_heading_set = False
                        bias_share.put(0.0)  # Reset bias before turning
                        print(f"[NAV] STATE 11→12: Distance {distance:.0f}mm > {STATE_11_DIST:.0f}mm - Exit section complete, preparing turn")
                        STATE = 12
                    else:
                        print(f"[NAV] STATE 11 - Distance={distance:.0f}mm, Heading={heading:.1f}°")
                
                # ════════════════════════════════════════════════════════════════════════════
                # STATE 12: Turn to 180° (should be close)
                # ════════════════════════════════════════════════════════════════════════════
                elif STATE == 12:
                    if not state_heading_set:
                        desired_angle_share.put(HEADING_180)
                        state_heading_set = True
                    
                    line_follow_flg.put(0)
                    force_straight_flg.put(1)
                    nav_turn_flg.put(1)
                    bias_share.put(0.0)
                    left_desired_vel.put(0)
                    right_desired_vel.put(0)
                    
                    heading_error = calc_heading_error(heading, HEADING_180)
                    if not transition_flags[12] and abs(heading_error) < HEADING_TOLERANCE:
                        transition_flags[12] = True
                        state_heading_set = False
                        left_desired_vel.put(saved_velocity)
                        right_desired_vel.put(saved_velocity)
                        print(f"[NAV] STATE 12→13: Heading {heading:.1f}° reached {HEADING_180}° - Ready for straight section")
                        STATE = 13
                    else:
                        print(f"[NAV] STATE 12 - Heading={heading:.1f}° (Target: {HEADING_180}°)")
                
                # ════════════════════════════════════════════════════════════════════════════
                # STATE 13: Go straight at 180° through parking garage until distance > STATE_13_DIST
                # ════════════════════════════════════════════════════════════════════════════
                elif STATE == 13:
                    line_follow_flg.put(0)
                    force_straight_flg.put(1)
                    nav_turn_flg.put(0)
                    bias_share.put(0.0)
                    desired_angle_share.put(HEADING_180)
                    left_desired_vel.put(saved_velocity if saved_velocity > 0 else 100)
                    right_desired_vel.put(saved_velocity if saved_velocity > 0 else 100)
                    
                    # Check for excessive heading drift and correct with bias
                    heading_error = calc_heading_error(heading, HEADING_180)
                    if abs(heading_error) > 8.0:
                        bias_share.put(heading_error * 0.15)
                    
                    if not transition_flags[13] and distance > STATE_13_DIST:
                        transition_flags[13] = True
                        left_desired_vel.put(0)
                        right_desired_vel.put(0)
                        state_heading_set = False
                        print(f"[NAV] STATE 13→14: Distance {distance:.0f}mm > {STATE_13_DIST:.0f}mm - Approaching final turn")
                        STATE = 14
                    else:
                        print(f"[NAV] STATE 13 - Distance={distance:.0f}mm, Heading={heading:.1f}°")
                
                # ════════════════════════════════════════════════════════════════════════════
                # STATE 14: Turn to 90° to exit garage
                # ════════════════════════════════════════════════════════════════════════════
                elif STATE == 14:
                    if not state_heading_set:
                        desired_angle_share.put(HEADING_90)
                        state_heading_set = True
                    
                    line_follow_flg.put(0)
                    force_straight_flg.put(1)
                    nav_turn_flg.put(1)
                    bias_share.put(0.0)
                    left_desired_vel.put(0)
                    right_desired_vel.put(0)
                    
                    heading_error = calc_heading_error(heading, HEADING_90)
                    if not transition_flags[14] and abs(heading_error) < HEADING_TOLERANCE:
                        transition_flags[14] = True
                        state_heading_set = False
                        left_desired_vel.put(saved_velocity)
                        right_desired_vel.put(saved_velocity)
                        print(f"[NAV] STATE 14→15: Heading {heading:.1f}° reached {HEADING_90}° - Ready for final straight")
                        STATE = 15
                    else:
                        print(f"[NAV] STATE 14 - Heading={heading:.1f}° (Target: {HEADING_90}°)")
                
                # ════════════════════════════════════════════════════════════════════════════
                # STATE 15: Go straight at 90° until BUMP DETECTED (wall) or distance threshold
                # ════════════════════════════════════════════════════════════════════════════
                elif STATE == 15:
                    line_follow_flg.put(0)
                    force_straight_flg.put(1)
                    nav_turn_flg.put(0)
                    bias_share.put(0.0)
                    desired_angle_share.put(HEADING_90)
                    left_desired_vel.put(saved_velocity)
                    right_desired_vel.put(saved_velocity)
                    
                    # Check for bump sensor contact (wall)
                    if bump_detected_share.get() == 1:
                        left_desired_vel.put(0)
                        right_desired_vel.put(0)
                        print(f"[NAV] STATE 15→16: BUMP DETECTED at {distance:.0f}mm - Entering wall routine")
                        STATE = 16
                    elif not transition_flags[15] and distance >= STATE_15_DIST:
                        transition_flags[15] = True
                        left_desired_vel.put(0)
                        right_desired_vel.put(0)
                        print(f"[NAV] STATE 15 (FINISH): Distance {distance:.0f}mm >= {STATE_15_DIST:.0f}mm - COURSE COMPLETE!")
                        STATE = 15  # Stay in STATE 15 if no bump
                    else:
                        print(f"[NAV] STATE 15 - Distance={distance:.0f}mm, Heading={heading:.1f}°")
                
                # ════════════════════════════════════════════════════════════════════════════════
                # WALL INTERACTION SEQUENCE (States 16-23)
                # Triggered when bump sensor detects wall contact during STATE 15
                # ════════════════════════════════════════════════════════════════════════════════
                
                # STATE 16: Back up until distance decreases by 50mm
                elif STATE == 16:
                    # FIXED: Initialize saved_velocity and reset integral flag on first entry
                    if STATE not in state_start_distances:
                        state_start_distances[STATE] = distance
                        saved_velocity = 50  # Set recovery velocity
                        print(f"[NAV] STATE 16: Starting backup from {distance:.0f}mm, target: {distance - STATE_16_BACKUP:.0f}mm")
                    
                    line_follow_flg.put(0)
                    force_straight_flg.put(0)
                    nav_turn_flg.put(0)
                    bias_share.put(0.0)
                    
                    # Set negative velocity to back up
                    left_desired_vel.put(-50)
                    right_desired_vel.put(-50)
                    
                    # Check if distance threshold reached (absolute distance, not relative)
                    if distance <= state_start_distances[STATE] - STATE_16_BACKUP:
                        left_desired_vel.put(0)
                        right_desired_vel.put(0)
                        state_heading_set = False
                        print(f"[NAV] STATE 16→17: Reached distance threshold {distance:.0f}mm")
                        del state_start_distances[STATE]
                        STATE = 17
                    else:
                        print(f"[NAV] STATE 16 - Distance: {distance:.0f}mm (target: {state_start_distances[STATE] - STATE_16_BACKUP:.0f}mm)")
                
                # STATE 17: Turn to 0 degrees
                elif STATE == 17:
                    if not state_heading_set:
                        desired_angle_share.put(STATE_17_TURN)
                        nav_turn_flg.put(1)  # Enable turning
                        state_heading_set = True
                    
                    line_follow_flg.put(0)
                    force_straight_flg.put(1)
                    bias_share.put(0.0)
                    left_desired_vel.put(0)
                    right_desired_vel.put(0)
                    
                    heading_error = calc_heading_error(heading, STATE_17_TURN)
                    if not transition_flags[17] and abs(heading_error) < HEADING_TOLERANCE + 5.0:
                        transition_flags[17] = True
                        state_heading_set = False
                        nav_turn_flg.put(0)  # Disable turning
                        # FIXED: Keep saved_velocity at 50 from backup, will use for STATE 18
                        print(f"[NAV] STATE 17→18: Heading {heading:.1f}° reached {STATE_17_TURN}°")
                        STATE = 18
                    else:
                        print(f"[NAV] STATE 17 - Heading={heading:.1f}° (Target: {STATE_17_TURN}°)")
                
                # STATE 18: Straight at 0° until distance threshold reached
                elif STATE == 18:
                    line_follow_flg.put(0)
                    force_straight_flg.put(1)  # Force straight motion
                    nav_turn_flg.put(0)
                    bias_share.put(0.0)  # FIXED: Static bias, NO dynamic updates
                    desired_angle_share.put(STATE_17_TURN)
                    left_desired_vel.put(saved_velocity if saved_velocity > 0 else 50)
                    right_desired_vel.put(saved_velocity if saved_velocity > 0 else 50)
                    
                    if STATE not in state_start_distances:
                        state_start_distances[STATE] = distance
                        transition_flags[18] = False  # Reset transition flag on entry
                        print(f"[NAV] STATE 18: Starting from {distance:.0f}mm, target: {distance + STATE_18_STRAIGHT:.0f}mm")
                    
                    # REMOVED: Dynamic bias adjustment
                    # The force_straight_flg prevents line following, so no bias adjustment should happen
                    # Any heading correction should come from the motor control's proportional gain
                    
                    if distance >= state_start_distances[STATE] + STATE_18_STRAIGHT:
                        left_desired_vel.put(0)
                        right_desired_vel.put(0)
                        saved_velocity = 0
                        state_heading_set = False
                        del state_start_distances[STATE]
                        print(f"[NAV] STATE 18→19: Reached distance threshold {distance:.0f}mm")
                        STATE = 19
                    else:
                        print(f"[NAV] STATE 18 - Distance: {distance:.0f}mm (target: {state_start_distances[STATE] + STATE_18_STRAIGHT:.0f}mm)")
                
                # STATE 19: Turn to 90 degrees
                elif STATE == 19:
                    if not state_heading_set:
                        desired_angle_share.put(STATE_19_TURN)
                        nav_turn_flg.put(1)  # Enable turning
                        state_heading_set = True
                    
                    line_follow_flg.put(0)
                    force_straight_flg.put(1)
                    bias_share.put(0.0)
                    left_desired_vel.put(0)
                    right_desired_vel.put(0)
                    
                    heading_error = calc_heading_error(heading, STATE_19_TURN)
                    if not transition_flags[19] and abs(heading_error) < 11.0:  # +/- 11 degrees (6 + 5)
                        transition_flags[19] = True
                        state_heading_set = False
                        nav_turn_flg.put(0)  # Disable turning
                        left_desired_vel.put(saved_velocity)
                        right_desired_vel.put(saved_velocity)
                        print(f"[NAV] STATE 19→20: Heading {heading:.1f}° reached {STATE_19_TURN}°")
                        STATE = 20
                    else:
                        print(f"[NAV] STATE 19 - Heading={heading:.1f}° (Target: {STATE_19_TURN}°)")
                
                # STATE 20: Straight until distance threshold reached
                elif STATE == 20:
                    line_follow_flg.put(0)
                    force_straight_flg.put(1)
                    nav_turn_flg.put(0)
                    bias_share.put(0.0)  # FIXED: Static bias, NO dynamic updates
                    desired_angle_share.put(STATE_19_TURN)
                    left_desired_vel.put(saved_velocity if saved_velocity > 0 else 50)
                    right_desired_vel.put(saved_velocity if saved_velocity > 0 else 50)
                    
                    if STATE not in state_start_distances:
                        state_start_distances[STATE] = distance
                        transition_flags[20] = False
                        print(f"[NAV] STATE 20: Starting from {distance:.0f}mm, target: {distance + STATE_20_STRAIGHT:.0f}mm")
                    
                    # REMOVED: Dynamic bias adjustment - conflicts with force_straight_flg
                    
                    if distance >= state_start_distances[STATE] + STATE_20_STRAIGHT:
                        left_desired_vel.put(0)
                        right_desired_vel.put(0)
                        saved_velocity = 0
                        state_heading_set = False
                        del state_start_distances[STATE]
                        print(f"[NAV] STATE 20→21: Reached distance threshold {distance:.0f}mm")
                        STATE = 21
                    else:
                        print(f"[NAV] STATE 20 - Distance: {distance:.0f}mm (target: {state_start_distances[STATE] + STATE_20_STRAIGHT:.0f}mm)")
                
                # STATE 21: Turn to 180 degrees
                elif STATE == 21:
                    if not state_heading_set:
                        desired_angle_share.put(STATE_21_TURN)
                        nav_turn_flg.put(1)  # Enable turning
                        state_heading_set = True
                    
                    line_follow_flg.put(0)
                    force_straight_flg.put(1)
                    bias_share.put(0.0)
                    left_desired_vel.put(0)
                    right_desired_vel.put(0)
                    
                    heading_error = calc_heading_error(heading, STATE_21_TURN)
                    if not transition_flags[21] and abs(heading_error) < 11.0:  # +/- 11 degrees (6 + 5)
                        transition_flags[21] = True
                        state_heading_set = False
                        nav_turn_flg.put(0)  # Disable turning
                        left_desired_vel.put(saved_velocity)
                        right_desired_vel.put(saved_velocity)
                        print(f"[NAV] STATE 21→22: Heading {heading:.1f}° reached {STATE_21_TURN}°")
                        STATE = 22
                    else:
                        print(f"[NAV] STATE 21 - Heading={heading:.1f}° (Target: {STATE_21_TURN}°)")
                
                # STATE 22: Line following at bias=0 until distance threshold
                elif STATE == 22:
                    line_follow_flg.put(1)
                    force_straight_flg.put(0)
                    nav_turn_flg.put(0)
                    bias_share.put(0.0)
                    left_desired_vel.put(saved_velocity if saved_velocity > 0 else 50)
                    right_desired_vel.put(saved_velocity if saved_velocity > 0 else 50)
                    
                    if STATE not in state_start_distances:
                        state_start_distances[STATE] = distance
                        transition_flags[22] = False
                        print(f"[NAV] STATE 22: Starting line follow from {distance:.0f}mm, target: {distance + STATE_22_STRAIGHT:.0f}mm")
                    
                    if distance >= state_start_distances[STATE] + STATE_22_STRAIGHT:
                        left_desired_vel.put(0)
                        right_desired_vel.put(0)
                        del state_start_distances[STATE]
                        print(f"[NAV] STATE 22→23: Reached distance threshold {distance:.0f}mm")
                        STATE = 23
                    else:
                        print(f"[NAV] STATE 22 - Distance: {distance:.0f}mm (target: {state_start_distances[STATE] + STATE_22_STRAIGHT:.0f}mm)")
                
                # STATE 23: STOP
                elif STATE == 23:
                    line_follow_flg.put(0)
                    force_straight_flg.put(0)
                    nav_turn_flg.put(0)
                    bias_share.put(0.0)
                    left_desired_vel.put(0)
                    right_desired_vel.put(0)
                    
                    if not transition_flags[23]:
                        transition_flags[23] = True
                        print(f"[NAV] STATE 23 (COURSE COMPLETE): Distance={distance:.0f}mm - STOPPED")
                    
                    print(f"[NAV] STATE 23 (STOPPED) - Distance={distance:.0f}mm, Heading={heading:.1f}°")
        
        yield STATE