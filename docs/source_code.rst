Source Code
===========

Main Files
----------

**main.py** - System Initialization and Scheduler

- Sets up all hardware (motors, encoders, sensors)
- Creates task objects and shares
- Runs the cotask scheduler
- ~240 lines

**navigation_v2_compact_encoder.py** - Navigation State Machine

- 24-state obstacle course navigation
- Encoder-based distance and heading tracking
- Bump sensor recovery sequence (7 states)
- Queue-based logging for Bluetooth telemetry
- ~670 lines

**CL_control_task_V5.py** - Closed-Loop Control

- Dual PI velocity controllers
- Heading stabilization
- Line-following bias adjustment
- Conflicts with force_straight_flg handled properly

**motor_ctrl_task_V3.py** - Motor Control

- Low-level PWM and H-bridge control
- Encoder reading and velocity calculation
- Motor effort limits

**encoder_heading_task.py** - Odometry Calculation

- Encoder position accumulation
- Heading angle calculation (atan2 method)
- Distance tracking

**bump_sensor_task.py** - Collision Detection

- Polls bump sensor
- Sets collision flag
- Triggers recovery sequence

**serial_task_V6.py** - Bluetooth Communication

- Receives calibration commands
- Parses velocity parameters
- Relays navigation logs via queue
- Manages trial start/stop

**UI_Line_Follow.py** - Python Control Interface

- Sends calibration commands
- Monitors robot output
- Allows manual start/stop
- Saves terminal output for analysis

Key Code Patterns
-----------------

State Machine Pattern
~~~~~~~~~~~~~~~~~~~~~

::

    while True:
        if STATE == 0:
            # State initialization
            if not state_heading_set:
                desired_angle_share.put(target_heading)
                state_heading_set = True
            
            # Continuous actions
            line_follow_flg.put(1)
            bias_share.put(0.0)
            
            # Transition condition
            if not transition_flags[0] and distance > STATE_0_DIST:
                transition_flags[0] = True
                log(f"[NAV] STATE 0→1: Distance {distance:.0f}mm > {STATE_0_DIST:.0f}mm")
                STATE = 1
        
        yield STATE

Conflict Resolution: force_straight_flg
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

::

    # In CL Control Task:
    if force_straight_flg.get() == 1:
        # Straight motion - use only heading PID, no line bias
        bias_share.put(0.0)  # Static bias
        motor_effort = Kp * heading_error  # PID correction only
    else:
        # Line following - apply line bias in addition to heading
        bias_share.put(calc_line_bias())  # Dynamic line bias
        motor_effort = Kp * heading_error + line_bias_adjustment

Instant Heading Capture
~~~~~~~~~~~~~~~~~~~~~~~

::

    # In STATE 3 (straight after turn):
    if not state_heading_set:
        desired_angle_share.put(heading)  # Capture current heading
        state_heading_set = True
    
    # Robot maintains whatever heading it's already at
    # No delay waiting for motor to adjust - instant transition

Queue-Based Logging
~~~~~~~~~~~~~~~~~~~

::

    # In Navigation Task:
    def log(msg):
        print(msg)
        try:
            nav_log_queue.put(msg)
        except:
            pass
    
    # In Serial Task:
    while nav_log_queue.any():
        msg = nav_log_queue.get()
        bt.write(msg.encode() + b'\n')

Recovery Sequence Pattern
~~~~~~~~~~~~~~~~~~~~~~~~~

::

    # STATE 15: Collision detected
    if bump_detected_share.get() == 1:
        saved_velocity = 50
        STATE = 16
    
    # STATE 16: Backup
    # STATE 17: Turn to 0°
    # STATE 18: Straight 175mm at 0°
    # STATE 19: Turn to 90°
    # STATE 20: Straight 200mm at 90°
    # STATE 21: Turn to 180°
    # STATE 22: Line follow
    # STATE 23: Stop

Performance Optimization
------------------------

**Timer-Based Scheduling**: cotask scheduler runs all tasks based on period rather than blocking

**Encoder Odometry**: Avoids expensive IMU calculations; simple delta position subtraction

**Queue-Based Logging**: Doesn't block navigation; messages buffered and sent during serial task

**Static Bias in Straights**: Prevents motor control oscillation; proportional PID handles fine corrections

**Instant Heading Capture**: Eliminates overshoot on turn-to-straight transitions

File Sizes
----------

.. list-table::
   :header-rows: 1

   * - File
     - Lines
     - Purpose
   * - main.py
     - 240
     - Initialization
   * - navigation_v2_compact_encoder.py
     - 670
     - Navigation (24 states)
   * - CL_control_task_V5.py
     - ~300
     - Control loops
   * - motor_ctrl_task_V3.py
     - ~150
     - Motor driver
   * - encoder_heading_task.py
     - ~50
     - Odometry
   * - serial_task_V6.py
     - ~100
     - Bluetooth
   * - bump_sensor_task.py
     - ~30
     - Collision detection
