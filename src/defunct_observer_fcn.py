import ulab
from ulab import numpy as np
from math import pi, cos, sin
import battery_adc
from RK4_solver import RK4_solver

def normalize_angle_deg(angle_deg):
    '''!@brief Normalize angle to [-180, 180] range
    @param angle_deg Angle in degrees
    @return Normalized angle in degrees
    '''
    while angle_deg > 180.0:
        angle_deg -= 360.0
    while angle_deg < -180.0:
        angle_deg += 360.0
    return angle_deg

def observer_task_fcn(shares):
    '''!@brief Observer task - integrates full kinematic state using RK4
    Maintains state: [X, Y, Theta, s, Omega_L, Omega_R]
    Uses IMU heading for heading correction during integration
    Runs once per motor control update period to stay synchronized
    '''
    
    (meas_heading_share, meas_yaw_rate_share, 
    left_enc_pos, right_enc_pos,
    left_enc_speed, right_enc_speed,
    obs_heading_share, obs_yaw_rate_share, 
    left_set_point, right_set_point,
    observer_calibration_flg,
    big_X_share, big_Y_share, initial_heading_share, end_flg) = shares
    
    # Define kinematics function locally
    def kinematics_fun(t, x, omega_L_ms, omega_R_ms, imu_heading_rad):
        '''!@brief Full kinematic model for Romi robot
        State: x = [X, Y, Theta, s, Omega_L, Omega_R] (6 states)
        Uses MEASURED wheel velocities in m/s and IMU heading
        '''
        
        # Robot parameters
        r = 0.035                     # Wheel radius [m]
        w = 0.141                     # Track width [m]
        
        # Extract state variables
        X = x[0, 0]          # Global X position [m]
        Y = x[1, 0]          # Global Y position [m]
        Theta = x[2, 0]      # Heading [rad]
        s = x[3, 0]          # Arc length traveled [m]
        
        # Use the PASSED-IN measured velocities (already in m/s)
        v_L = omega_L_ms  # linear velocity m/s
        v_R = omega_R_ms  # linear velocity m/s
        
        # Average velocity and angular velocity
        v_center = (v_L + v_R) / 2.0  # Forward velocity at center
        omega_body = (v_R - v_L) / w   # Angular velocity (yaw rate)
        
        # Use IMU heading directly for position calculation
        # This ensures X, Y are calculated with correct heading reference
        # NOTE: If the robot veers off track, try adjusting the heading by 180° or 90°
        # heading_corrected = imu_heading_rad + pi  # Uncomment to flip 180°
        heading_corrected = imu_heading_rad
        X_dot = v_center * cos(heading_corrected)
        Y_dot = -v_center * sin(heading_corrected)
        
        # Heading derivative from wheel velocities
        Theta_dot = omega_body
        
        # Arc length derivative
        s_dot = v_center
        
        # State derivative vector
        xd = np.array([
            [X_dot],
            [Y_dot],
            [Theta_dot],
            [s_dot],
            [v_L],      
            [v_R]       
        ])
        
        # Output vector
        y = np.array([
            [X],
            [Y],
            [Theta],
            [s],
            [v_center],      # Linear velocity
            [omega_body]     # Angular velocity
        ])
        
        return xd, y
    
    # Robot parameters
    r = 0.035                     # Wheel radius [m]
    w = 0.141                     # Track width [m]
    TICKS_PER_REV = 1437.1
    WHEEL_CIRC_M = 2 * pi * r  # meters
    
    # Velocity calibration factor - adjust if scale is wrong
    # If observer shows half the distance traveled, set this to 2.0
    # If observer shows double the distance, set this to 0.5
    VELOCITY_SCALE = 2.0  # <-- TUNE THIS if scale is off
    
    FIRST_RUN = True
    run = False
    printed = False
    x = None
    imu_heading_offset_rad = 0.0  # Offset to correct IMU heading
    prev_imu_heading_deg = 0.0  # For smoothing
    imu_alpha = 0.3  # Low-pass filter coefficient (0.0 = all previous, 1.0 = all current)
    
    while True:
        if observer_calibration_flg.get() == 1:
            # Get initial heading from IMU (in degrees)
            initial_heading_deg = meas_heading_share.get()
            
            # The robot is pointing in +X direction (0°), so calculate the offset
            # If IMU reads -79.63°, offset = -(-79.63°) = +79.63°
            imu_heading_offset_deg = -initial_heading_deg
            imu_heading_offset_rad = imu_heading_offset_deg * pi / 180.0
            
            print(f"IMU heading at startup: {initial_heading_deg:.2f}°")
            print(f"IMU heading offset (to correct to +X = 0°): {imu_heading_offset_deg:.2f}° ({imu_heading_offset_rad:.4f} rad)")
            
            initial_heading_share.put(0.0)  # Store 0° as the reference (robot pointing +X)
            
            initial_X_mm = 100  # mm
            initial_Y_mm = 800  # mm
            big_X_share.put(initial_X_mm)
            big_Y_share.put(initial_Y_mm)
            
            observer_calibration_flg.put(0)
            run = True
        
        if run:
            if FIRST_RUN:
                # Initialize state: [X, Y, Theta, s, Omega_L, Omega_R]
                # Position in mm converted to m, heading = 0° (pointing +X)
                x = np.array([
                    [100.0 / 1000.0],          # X = 100 mm = 0.1 m
                    [800.0 / 1000.0],          # Y = 800 mm = 0.8 m
                    [0.0],                     # Theta = 0° (pointing +X direction)
                    [0.0],                     # s = 0 (no distance traveled yet)
                    [0.0],                     # Omega_L = 0
                    [0.0]                      # Omega_R = 0
                ])
                FIRST_RUN = False
                print(f"[FIRST_RUN] Observer initialized at (100, 800) mm pointing +X (0°)\n")
            
            # Get CURRENT measured wheel velocities (in ticks/us from encoder)
            omega_L_ticks_us = left_enc_speed.get()
            omega_R_ticks_us = right_enc_speed.get()
            
            # Convert from ticks/us to m/s
            # ticks/us * (meters/tick) * (1e6 us/s) = m/s
            omega_L_ms = omega_L_ticks_us * (WHEEL_CIRC_M / TICKS_PER_REV) * 1e6 * VELOCITY_SCALE
            omega_R_ms = omega_R_ticks_us * (WHEEL_CIRC_M / TICKS_PER_REV) * 1e6 * VELOCITY_SCALE
            
            # Get current IMU heading (in degrees) and convert to radians
            imu_heading_deg = meas_heading_share.get()
            
            # Low-pass filter the IMU heading to reduce jitter
            # This helps smooth out rapid heading oscillations
            imu_heading_deg = imu_alpha * imu_heading_deg + (1.0 - imu_alpha) * prev_imu_heading_deg
            prev_imu_heading_deg = imu_heading_deg
            
            imu_heading_rad = imu_heading_deg * pi / 180.0
            
            # Apply offset correction to align IMU heading with robot frame
            # Robot pointing +X = 0°
            heading_corrected_rad = imu_heading_rad + imu_heading_offset_rad
            heading_corrected_deg = heading_corrected_rad * 180.0 / pi
            heading_corrected_deg = normalize_angle_deg(heading_corrected_deg)
            
            # Wrapper function for RK4 solver (RK4_solver expects fcn(t, x) format)
            def kinematics_wrapper(t, x_col):
                '''Wrapper to convert column vector to/from kinematics_fun format'''
                # Use corrected heading for position integration
                xd, y = kinematics_fun(t, x_col, omega_L_ms, omega_R_ms, heading_corrected_rad)
                return xd, y
            
            # Run RK4 integration for 100ms with 10ms steps
            tout, yout = RK4_solver(kinematics_wrapper, x, [0, 0.1], 0.01)
            
            # Extract final state from the last row
            x = np.array([[yout[-1, i]] for i in range(6)])
            
            # Extract final state
            X_final = x[0, 0]  # in meters
            Y_final = x[1, 0]  # in meters
            Theta_final = x[2, 0]  # in radians (from encoder integration)
            s_final = x[3, 0]  # arc length in meters
            
            # Set heading to corrected IMU value for output
            Theta_final_deg = heading_corrected_deg
            
            # Convert position to mm
            X_final_mm = X_final * 1000.0
            Y_final_mm = Y_final * 1000.0
            
            # Store outputs
            big_X_share.put(X_final_mm)
            big_Y_share.put(Y_final_mm)
            obs_heading_share.put(Theta_final_deg)
            obs_yaw_rate_share.put(meas_yaw_rate_share.get())
            
            # Update state heading with corrected IMU value for next iteration
            x[2, 0] = heading_corrected_rad
            
            # Debug: Print scale factor diagnosis (uncomment to debug velocity scaling)
            # If observer distance / physical distance ≠ 1.0, there's a scaling issue
            # observer_distance_mm = s_final * 1000.0
            # physical_distance_mm = 650  # From 100 to 750 in X
            # scale_factor = observer_distance_mm / physical_distance_mm
            # print(f"Velocity scale factor: {scale_factor:.3f} (should be 1.0)")
            
            # Debug output with velocity info
            v_avg_cms = ((omega_L_ms + omega_R_ms) / (2.0 * VELOCITY_SCALE)) * 100  # Convert m/s to cm/s
            omega_yaw_degs = ((omega_R_ms - omega_L_ms) / w) * 180.0 / pi  # Angular velocity in deg/s
            if end_flg.get() == 1 and not printed:
                print(f"X = {X_final_mm:.1f} mm, Y = {Y_final_mm:.1f} mm, Heading = {Theta_final_deg:.2f}° (corrected), s = {s_final*1000:.1f} mm | v={v_avg_cms:.1f}cm/s")
                printed = True
        
        yield