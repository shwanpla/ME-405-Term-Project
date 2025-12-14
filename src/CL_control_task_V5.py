"""
Closed-loop PI control task for dual motor velocity and heading control.
Implements velocity PI control with line-following adjustment and heading-based
turning control. Supports calibration, rest states, and force-straight mode.

Hardware:
    - IR Sensor Array: 8 reflectance sensors for line detection
    - Motor Encoders: 1437.1 ticks/rev
    - Wheel Diameter: 70mm (35mm radius)

Notes:
    - Gains are scaled by battery voltage for consistent performance
    - Anti-windup prevents integral accumulation during saturation
    - Force-straight mode overrides line-following for precise heading control
"""
import battery_adc
from time import ticks_us, ticks_diff
from multi_sensor_read import multiple_ir_readings
from line_follow_V1 import LineFollower


class CL_control:
    """
    Closed-loop PI controller for a single motor with line-following capability.
    """

    def __init__(self, multiple_ir_readings_object, motor):
        """
        Initialize the closed-loop controller.

        :param multiple_ir_readings_object: IR sensor array object
        :type multiple_ir_readings_object: multiple_ir_readings
        :param motor: Motor identifier ("LEFT" or "RIGHT")
        :type motor: str
        """
        self.sensor_array = multiple_ir_readings_object
        self.motor = motor
        self.reset_integral_flg = 0

    def run(self, shares):
        """
        Cooperative task function implementing PI velocity control with line following.

        :param shares: Tuple of shared variables for inter-task communication
        :type shares: tuple
        """
        (desired_vel, enc_speed, set_point,
        end_flg, ki_share, kp_share, calibration_flg, line_follow_flg,
        desired_angle_share, enc_heading_share, nav_rest_flg, nav_turn_flg, bias_share,
        force_straight_flg) = shares
        S0_INIT, S1_RUN, S2_REST, S3_TURN = 0, 1, 2, 3
        STATE = S0_INIT

        # Motor control gains
        kp_share.put(0.07)
        ki_share.put(0.035)
        U_MAX = 100
        TICKS_PER_REV = 1437.1
        WHEEL_CIRC_MM = 2 * 3.14159 * 35
        batt_v = battery_adc.battery_voltage()
        v_nom = 3.07
        _calibrated = 0
        ln_integral = 0

        # Turning control gains
        Kp_turn = 0.45
        Ki_turn = 0.01
        turn_integral = 0
        TURN_EFFORT_MAX = 20

        # Line following gains
        ln_const = 1.7
        ln_integ_const = 1.0
        i = 0
        timer_const = 0
        _calibration_step = 1
        t_prev = ticks_us()
        counter = 0

        while True:
            Kp = kp_share.get()
            Ki = ki_share.get()

            if STATE == S0_INIT:
                integ = 0.0
                set_point.put(0.0)
                STATE = S1_RUN

            elif STATE == S1_RUN:
                sensors = self.sensor_array
                if end_flg.get() == 1:
                    calibration_flg.put(0)

                if calibration_flg.get() == 1 and _calibration_step == 1:
                    black = 2500
                    print("[CL_CONTROL] Black calibrated, value: ", black)
                    _calibration_step = 2

                if calibration_flg.get() == 2 and _calibration_step == 2:
                    white = 500
                    print("[CL_CONTROL] White calibrated, value: ", white)
                    _calibration_step = 3

                if calibration_flg.get() == 3 and _calibration_step == 3:
                    initial_bias = bias_share.get()
                    line = LineFollower(sensors, black, white, bias=initial_bias)
                    _calibration_step = 1
                    _calibrated = 1

                if _calibrated == 1:
                    _calibration_step = 1
                    des_mmps = desired_vel.get()
                    if des_mmps >= 745:
                        des_mmps = 745
                    ticks_per_us = enc_speed.get()

                    vel_mmps = ticks_per_us * (WHEEL_CIRC_MM / TICKS_PER_REV) * 1_000_000.0

                    now = ticks_us()
                    dt_us = ticks_diff(now, t_prev) or 1
                    dt = dt_us / 1_000_000.0
                    t_prev = now

                    Kp = v_nom/batt_v * Kp
                    Ki = v_nom/batt_v * Ki

                    if line_follow_flg.get() == 1:
                        ln_error = line.calculate_error()
                    else:
                        ln_error = 0

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

                    if force_straight_flg.get() == 1:
                        ln_error = 0.0
                        if not hasattr(self, 'straight_line_active'):
                            print("[CL_CONTROL] Straight line mode activated")
                            self.straight_line_active = True
                    else:
                        if hasattr(self, 'straight_line_active') and self.straight_line_active:
                            print("[CL_CONTROL] Exiting straight line mode")
                            self.straight_line_active = False

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
                angle_err = enc_heading_share.get() - desired_angle_share.get()
                while angle_err > 180:
                    angle_err -= 360
                while angle_err < -180:
                    angle_err += 360
                close_theta = abs(angle_err) <= 5

                if desired_angle_share.get() != enc_heading_share.get():
                    raw_error = desired_angle_share.get() - enc_heading_share.get()

                    while raw_error > 180:
                        raw_error -= 360
                    while raw_error < -180:
                        raw_error += 360

                    turn_error = raw_error

                    if abs(turn_error) < 25:
                        turn_integral += turn_error
                    else:
                        turn_integral = 0

                    if hasattr(self, 'prev_turn_error'):
                        if (self.prev_turn_error > 0 and turn_error < 0) or (self.prev_turn_error < 0 and turn_error > 0):
                            turn_integral = 0
                    self.prev_turn_error = turn_error

                    turn_effort = Kp_turn * (turn_error) + Ki_turn * turn_integral

                    if turn_effort > TURN_EFFORT_MAX:
                        turn_effort = TURN_EFFORT_MAX
                    elif turn_effort < -TURN_EFFORT_MAX:
                        turn_effort = -TURN_EFFORT_MAX

                    if self.motor == "LEFT":
                        set_point.put(turn_effort)
                        print(f"[CL_CONTROL] {self.motor} motor - Effort: {turn_effort:.2f}, Heading: {enc_heading_share.get():.1f}°, Target: {desired_angle_share.get()}°, Error: {turn_error:.1f}°")
                    else:
                        set_point.put(-turn_effort)
                        print(f"[CL_CONTROL] {self.motor} motor - Effort: {-turn_effort:.2f}, Heading: {enc_heading_share.get():.1f}°, Target: {desired_angle_share.get()}°, Error: {turn_error:.1f}°")

                if close_theta:
                    print("[CL_CONTROL] Angle Reached")
                    nav_turn_flg.put(0)
                    self.reset_integral_flg = 1
                    STATE = S1_RUN
                
            yield STATE