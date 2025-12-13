'''
Closed Loop Control Task
@brief this task handles closed loop control of motor efforts and line following 
with different states for rest and heading adjustment
'''
import battery_adc
from time import ticks_us, ticks_diff
from multi_sensor_read import multiple_ir_readings
from line_follow_V1 import LineFollower

class CL_control:
    def __init__(self, multiple_ir_readings_object, motor):
        self.sensor_array = multiple_ir_readings_object
        self.motor = motor  # "LEFT" or "RIGHT"
        self.reset_integral_flg = 0 # Flag to reset the accumulated integral error after turning


    def run(self, shares):
        (desired_vel, enc_speed, set_point, 
        end_flg, ki_share, kp_share, calibration_flg, line_follow_flg, 
        desired_angle_share, enc_heading_share, nav_rest_flg, nav_turn_flg, bias_share, 
        force_straight_flg) = shares
        S0_INIT, S1_RUN, S2_REST, S3_TURN = 0, 1, 2, 3
        STATE = S0_INIT

        # MOTOR CONTROL GAINS

        kp_share.put(0.07) # Proportional gain for motor control
        ki_share.put(0.035) # Integral gain for motor control

        U_MAX = 100 # Max set point
        TICKS_PER_REV = 1437.1 # Encoder ticks per revolution
        WHEEL_CIRC_MM = 2 * 3.14159 * 35 # Wheel circumference in mm
        batt_v = battery_adc.battery_voltage() # Get battery voltage from adc 
        v_nom = 3.07 # nominal battery voltage for gain scaling
        _calibrated = 0 # Calibration flag for ir sensor
        ln_integral = 0 # Initial integral error for line following

        # TURNING CONTROL GAINS

        Kp_turn = 0.45  # Proportional gain - INCREASED for faster turning
        Ki_turn = 0.01  # Integral gain - small for fine control
        turn_integral = 0
        TURN_EFFORT_MAX = 20  # Cap turn effort to prevent overshoot

        """These gains worked on 11/21 for high speeds"""
        # ln_const = 8.1 # Proportional gain constant for line following
        # ln_integ_const = 3 # Integral gain constant for line following

        ln_const = 1.7       # Reduced from 2.0 to reduce overshoot in line following
        ln_integ_const = 1.0   # Reduced from 2.0 to reduce oscillation
        i = 0 # Simple counter for print statements
        timer_const = 0 # Time constant to reduce gains at the start to prevent overshooting
        _calibration_step = 1 # Initial step of calibration, 1 for black, 2 for white, 3 for line follower object creation
        t_prev = ticks_us() # Previous time for dt calculation
        counter = 0

        while True:
            Kp = kp_share.get()
            Ki = ki_share.get()

            if STATE == S0_INIT:
                integ = 0.0         # Initialize integrated error to 0
                set_point.put(0.0)  # Set motor efforts to 0
                STATE = S1_RUN

            elif STATE == S1_RUN:
                sensors = self.sensor_array # Assign IR reflectance sensors
                if end_flg.get() == 1:  # Reset calibration status when a trial ends
                    calibration_flg.put(0)
                # Calibrate black
                if calibration_flg.get() == 1 and _calibration_step == 1:
                    #black = LineFollower.calibrate(sensors)   # Read IR reflectance sensors for calibration
                    black = 2500  # Manually set black value for consistency
                    print("[CL_CONTROL] Black calibrated, value: ", black)
                    _calibration_step = 2
                # Calibrate white
                if calibration_flg.get() == 2 and _calibration_step == 2:
                    #white = LineFollower.calibrate(sensors)    # Read IR reflectance sensors for calibration
                    white = 500
                    print("[CL_CONTROL] White calibrated, value: ", white)
                    _calibration_step = 3
                # Create LineFollower Object
                if calibration_flg.get() == 3 and _calibration_step == 3:
                    # Get initial bias from share (default 1.55 for fork)
                    initial_bias = bias_share.get()
                    line = LineFollower(sensors, black, white, bias=initial_bias)  # Initialize line follower object
                    _calibration_step = 1
                    _calibrated = 1 
                # Once calibration is complete, ROMI can begin line following
                if _calibrated == 1:
                    _calibration_step = 1 # Reset calibration step for next time
                    # Get desired velocity in mmps
                    des_mmps = desired_vel.get()   # Get the desired vlocity in mm/s
                    if des_mmps >= 745:     # Clamp max velocity because one motor cannot exceed 745 mm/s
                        des_mmps = 745 
                    ticks_per_us = enc_speed.get()  # Get raw speed in ticks/us speed from encoder speed share

                    # Convert ticks/us -> mm/s
                    vel_mmps = ticks_per_us * (WHEEL_CIRC_MM / TICKS_PER_REV) * 1_000_000.0

                    # dt
                    now = ticks_us()
                    dt_us = ticks_diff(now, t_prev) or 1
                    dt = dt_us / 1_000_000.0        # Convert microseconds to seconds
                    t_prev = now
                    # Adjust gains based on battery voltage
                    Kp = v_nom/batt_v * Kp # Adjust gains current based on battery voltage
                    Ki = v_nom/batt_v * Ki 
                    # Get line error only if line following is enabled via line follow flag
                    if line_follow_flg.get() == 1:
                        ln_error = line.calculate_error()
                    else: 
                        ln_error = 0

                    # Depending on the motor, effort must be increased or decreased
                    if self.motor == "LEFT":
                        ln_error = -ln_error
                    if self.motor == "RIGHT":
                        ln_error = ln_error
                    
                    if self.reset_integral_flg == 1:
                        integ = 0
                        ln_integral = 0
                        line_adjustment = 0
                        counter += 1
                        if counter >= 3:
                            self.reset_integral_flg = 0

                    # If force_straight_flg is active, override line error to command straight line
                    if force_straight_flg.get() == 1:
                        ln_error = 0.0  # Force line error to zero for straight motion
                        # Print once when entering straight line mode
                        if not hasattr(self, 'straight_line_active'):
                            print("[CL_CONTROL] Straight line mode activated - forcing line error to 0")
                            self.straight_line_active = True
                    else:
                        # Reset flag when exiting straight line mode
                        if hasattr(self, 'straight_line_active') and self.straight_line_active:
                            print("[CL_CONTROL] Exiting straight line mode - resuming normal line following")
                            self.straight_line_active = False

                    # PI control with simple anti-windup
                    e = des_mmps - vel_mmps
                    
                    line_adjustment = (ln_error * ln_const) + (ln_integral * ln_integ_const)
                    u_unsat = Kp * e + Ki * integ + line_adjustment

                    if u_unsat > U_MAX:
                        u = U_MAX
                    elif u_unsat < -U_MAX:
                        u = -U_MAX
                    else:
                        u = u_unsat

                    satur_hi = (u >= U_MAX - 1e-9)
                    satur_lo = (u <= -U_MAX + 1e-9)
                    if (not satur_hi or e < 0) and (not satur_lo or e > 0):
                        integ += e * dt
                        ln_integral += ln_error * dt

                    if timer_const > 10:
                        set_point.put(round(u))
                    else:
                        set_point.put(round(u)/10)
                        integ = 0
                        ln_integral = 0
                        timer_const += 1
                    # i += 1
                    # if i > 10 and end_flg.get() == 0:
                    #     print(self.motor, " ", "Set Point: ", set_point.get(), "Line Error: ", ln_error, "\n\rLine adjustment: ", line_adjustment)
                    #     i = 0
                    
                    # Check if bias has been updated by navigation task
                    current_bias = bias_share.get()
                    if hasattr(self, 'last_bias'):
                        if current_bias != self.last_bias:
                            print("[CL_CONTROL] Bias updated to: ", current_bias)
                            line.set_bias(current_bias)
                            self.last_bias = current_bias
                    else:
                        self.last_bias = current_bias
                    
                    if nav_rest_flg.get() == 1:
                        STATE = S2_REST
                    if nav_turn_flg.get() == 1:
                        STATE = S3_TURN
            
            elif STATE == S2_REST:
                set_point.put(0.0)
                if nav_rest_flg.get() == 0:
                    STATE = S1_RUN
            
            elif STATE == S3_TURN:

                # Calculate normalized angle error for close_theta check
                angle_err = enc_heading_share.get() - desired_angle_share.get()
                while angle_err > 180:
                    angle_err -= 360
                while angle_err < -180:
                    angle_err += 360
                close_theta = abs(angle_err) <= 5

                if desired_angle_share.get() != enc_heading_share.get():
                    # Calculate raw error
                    raw_error = desired_angle_share.get() - enc_heading_share.get()
                    
                    # Normalize error to shortest path (always within ±180°)
                    while raw_error > 180:
                        raw_error -= 360
                    while raw_error < -180:
                        raw_error += 360
                    
                    turn_error = raw_error
                    
                    # Anti-windup: only accumulate integral when close to target
                    if abs(turn_error) < 25:  # Increased threshold from 15° to allow faster approach
                        turn_integral += turn_error
                    else:
                        turn_integral = 0  # Reset if too far away to prevent overshoot
                    
                    # OVERSHOOT PREVENTION: Reset integral if error changes sign (past target)
                    if hasattr(self, 'prev_turn_error'):
                        if (self.prev_turn_error > 0 and turn_error < 0) or (self.prev_turn_error < 0 and turn_error > 0):
                            turn_integral = 0  # Reset integral on sign change to prevent overshoot
                    self.prev_turn_error = turn_error
                    
                    turn_effort = Kp_turn * (turn_error) + Ki_turn * turn_integral
                    
                    # Saturate turn effort to prevent overshoot
                    if turn_effort > TURN_EFFORT_MAX:
                        turn_effort = TURN_EFFORT_MAX
                    elif turn_effort < -TURN_EFFORT_MAX:
                        turn_effort = -TURN_EFFORT_MAX
                    
                    # Apply motor-specific scaling: left motor is negated, right motor is positive
                    if self.motor == "LEFT":
                        set_point.put(turn_effort)  # Left motor - positive effort for left turn
                        print(f"[CL_CONTROL] {self.motor} motor - Effort: {turn_effort:.2f}, Heading: {enc_heading_share.get():.1f}°, Target: {desired_angle_share.get()}°, Error: {turn_error:.1f}°")
                    else:
                        set_point.put(-turn_effort)   # Right motor - negative effort for left turn
                        print(f"[CL_CONTROL] {self.motor} motor - Effort: {-turn_effort:.2f}, Heading: {enc_heading_share.get():.1f}°, Target: {desired_angle_share.get()}°, Error: {turn_error:.1f}°")
                
                if close_theta:
                    print("[CL_CONTROL] Angle Reached")
                    nav_turn_flg.put(0)
                    self.reset_integral_flg = 1 # Reset the accumulated integral error 
                    STATE = S1_RUN
                

                    

            yield STATE