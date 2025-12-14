===========
Source Code
===========

This page provides comprehensive documentation for all Python modules used in the autonomous obstacle course navigation robot.

--------

main.py
=======

Overview
--------

Main program for autonomous obstacle course navigation robot. Implements cooperative multitasking scheduler with encoder-based odometry, line following, PI velocity control, and bump sensor collision detection.

Hardware Configuration
----------------------

Motors
^^^^^^
- **Type**: 2x DC motors with quadrature encoders
- **Encoder Resolution**: 1437.1 ticks/rev
- **Control**: H-bridge motor drivers with PWM

Sensors
^^^^^^^
- **IR Sensor Array**: 7 reflectance sensors for line detection
- **Bump Sensor**: Collision detection on Pin PC11
- **Battery Monitor**: ADC-based voltage monitoring on Pin PC0

Physical Specifications
^^^^^^^^^^^^^^^^^^^^^^^
- **Wheel Diameter**: 70mm (35mm radius)
- **Track Width**: 141mm (distance between wheels)
- **Wheel Circumference**: 219.91mm

Module Imports
--------------

Standard Library
^^^^^^^^^^^^^^^^
.. code-block:: python

   import gc                    # Garbage collector for memory management
   from time import ticks_us, ticks_diff  # Microsecond timing utilities

MicroPython Hardware
^^^^^^^^^^^^^^^^^^^^
.. code-block:: python

   import pyb                   # MicroPython board-specific functions
   from pyb import Pin, Timer, UART  # Hardware peripherals

Task Scheduler
^^^^^^^^^^^^^^
.. code-block:: python

   import cotask                # Cooperative multitasking scheduler
   import task_share            # Inter-task communication primitives

Custom Modules
^^^^^^^^^^^^^^
.. code-block:: python

   from motor import motor_driver              # Motor driver class
   from encoder import Encoder                 # Quadrature encoder interface
   from multi_sensor_read import multiple_ir_readings  # IR sensor array
   import motor_ctrl_task_V3 as motor_ctrl_task       # Motor control task
   import CL_control_task_V5 as CL_control_task       # Closed-loop PI control
   from serial_task_V6 import serial_task_fun         # Bluetooth serial interface
   from encoder_heading_task import encoder_heading_task_fun  # Odometry
   from navigation_v2_compact_encoder import navigation_task_fun  # Navigation FSM
   from bump_sensor_task import bump_sensor_task_fun  # Collision detection

Pin Assignments
---------------

Motor Pins
^^^^^^^^^^

Right Motor
"""""""""""
- **PWM**: Pin C9 (Timer 3, Channel 4)
- **Direction**: Pin H1
- **Enable (nSLP)**: Pin H0

Left Motor
""""""""""
- **PWM**: Pin B0 (Timer 3, Channel 3)
- **Direction**: Pin C12
- **Enable (nSLP)**: Pin C10

Encoder Pins
^^^^^^^^^^^^

Right Encoder (Encoder 1)
""""""""""""""""""""""""""
- **Channel A**: Pin A8 (Timer 1)
- **Channel B**: Pin A9 (Timer 1)

Left Encoder (Encoder 2)
"""""""""""""""""""""""""
- **Channel A**: Pin A0 (Timer 2)
- **Channel B**: Pin B3 (Timer 2)

IR Sensor Pins (Left to Right)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
- **Sensor 1**: Pin C4 (leftmost)
- **Sensor 2**: Pin B1
- **Sensor 3**: Pin A7
- **Sensor 4**: Pin C1 (center)
- **Sensor 5**: Pin A4
- **Sensor 6**: Pin A1
- **Sensor 7**: Pin C3 (rightmost)

Other Sensors
^^^^^^^^^^^^^
- **Bump Sensor**: Pin C11 (active LOW with pull-up)

Shared Variables
----------------

Shared variables enable communication between cooperative tasks. All shares use ``thread_protect=False`` for performance in single-threaded cooperative multitasking.

Motor Control Shares
^^^^^^^^^^^^^^^^^^^^

Start Flags
"""""""""""
.. code-block:: python

   left_start_flg = Share('B')   # Boolean: Start left motor trial
   right_start_flg = Share('B')  # Boolean: Start right motor trial

End Flags
"""""""""
.. code-block:: python

   left_end_flg = Share('B')     # Boolean: End left motor trial
   right_end_flg = Share('B')    # Boolean: End right motor trial

Velocity Shares
"""""""""""""""
.. code-block:: python

   left_desired_vel = Share('f')  # Float: Left motor target velocity (mm/s)
   right_desired_vel = Share('f') # Float: Right motor target velocity (mm/s)

Set Points
""""""""""
.. code-block:: python

   left_set_point = Share('f')   # Float: Left motor control output
   right_set_point = Share('f')  # Float: Right motor control output

Encoder Shares
^^^^^^^^^^^^^^

Position
""""""""
.. code-block:: python

   left_enc_pos = Share('f')     # Float: Left encoder position (ticks)
   right_enc_pos = Share('f')    # Float: Right encoder position (ticks)

Velocity
""""""""
.. code-block:: python

   left_enc_speed = Share('f')   # Float: Left encoder velocity (ticks/µs)
   right_enc_speed = Share('f')  # Float: Right encoder velocity (ticks/µs)

Odometry Shares
^^^^^^^^^^^^^^^
.. code-block:: python

   enc_heading_share = Share('f')   # Float: Robot heading (degrees, -180 to +180)
   enc_distance_share = Share('f')  # Float: Total distance traveled (mm)

Navigation Shares
^^^^^^^^^^^^^^^^^

State Flags
"""""""""""
.. code-block:: python

   line_follow_flg = Share('B')     # Boolean: Enable line following mode
   force_straight_flg = Share('B')  # Boolean: Override line following for straight motion
   nav_rest_flg = Share('B')        # Boolean: Put motors in rest state
   nav_turn_flg = Share('B')        # Boolean: Enable turning mode
   nav_stop_flg = Share('B')        # Boolean: Stop navigation

Navigation Parameters
"""""""""""""""""""""
.. code-block:: python

   desired_angle_share = Share('f') # Float: Target heading for turns (degrees)
   bias_share = Share('f')          # Float: Line following bias (-3 to +3)
   bias_timer_flg = Share('B')      # Boolean: Bias timer flag

Calibration Shares
^^^^^^^^^^^^^^^^^^
.. code-block:: python

   calibration_flg = Share('B')          # Byte: Calibration state (0=none, 1=black, 2=white, 3=complete)
   observer_calibration_flg = Share('B') # Boolean: Observer calibration status

Sensor Shares
^^^^^^^^^^^^^
.. code-block:: python

   bump_detected_share = Share('B')      # Boolean: Bump sensor collision detected

Control Tuning Shares
^^^^^^^^^^^^^^^^^^^^^
.. code-block:: python

   kp_share = Share('f')         # Float: Proportional gain
   ki_share = Share('f')         # Float: Integral gain

Queues
------

Queues store time-series data from trials. All queues have capacity of 10 items with ``overwrite=False``.

.. code-block:: python

   proc_speed_left = Queue('f', 10)      # Float queue: Left motor processed velocities
   proc_speed_right = Queue('f', 10)     # Float queue: Right motor processed velocities
   proc_pos_left = Queue('f', 10)        # Float queue: Left motor processed positions
   proc_pos_right = Queue('f', 10)       # Float queue: Right motor processed positions
   left_proc_time = Queue('f', 10)       # Float queue: Left motor timestamps
   right_proc_time = Queue('f', 10)      # Float queue: Right motor timestamps

Hardware Objects
----------------

Motors
^^^^^^
.. code-block:: python

   mot_right = motor_driver(
       PWM_pin=Pin.cpu.C9,
       DIR_pin=Pin.cpu.H1,
       nSLP_pin=Pin.cpu.H0,
       tim=Timer(3, freq=30_000),
       chan=4
   )

   mot_left = motor_driver(
       PWM_pin=Pin.cpu.B0,
       DIR_pin=Pin.cpu.C12,
       nSLP_pin=Pin.cpu.C10,
       tim=Timer(3, freq=30_000),
       chan=3
   )

- **PWM Frequency**: 30 kHz (ultrasonic, silent operation)
- **Timer**: Timer 3 shared between both motors
- **Channels**: 3 (left), 4 (right)

Encoders
^^^^^^^^
.. code-block:: python

   enc_right = Encoder(
       tim=Timer(1, prescaler=0, period=0xFFFF),
       chA_pin=Pin(Pin.cpu.A8, mode=Pin.IN),
       chB_pin=Pin(Pin.cpu.A9, mode=Pin.IN)
   )

   enc_left = Encoder(
       tim=Timer(2, prescaler=0, period=0xFFFF),
       chA_pin=Pin(Pin.cpu.A0, mode=Pin.IN),
       chB_pin=Pin(Pin.cpu.B3, mode=Pin.IN)
   )

- **Mode**: Hardware encoder mode (ENC_AB)
- **Resolution**: 16-bit (0-65535)
- **Prescaler**: 0 (no division, full resolution)

IR Sensor Array
^^^^^^^^^^^^^^^
.. code-block:: python

   ir_array = multiple_ir_readings(
       ir_pin_1=Pin.cpu.C4,
       ir_pin_2=Pin.cpu.B1,
       ir_pin_3=Pin.cpu.A7,
       ir_pin_4=Pin.cpu.C1,
       ir_pin_5=Pin.cpu.A4,
       ir_pin_6=Pin.cpu.A1,
       ir_pin_7=Pin.cpu.C3
   )

- **Sensors**: 7 analog IR reflectance sensors
- **Sampling**: Timer-synchronized ADC reads
- **Resolution**: 12-bit ADC (0-4095)

Task Configuration
------------------

Motor Control Tasks
^^^^^^^^^^^^^^^^^^^
.. code-block:: python

   left_mot_ctrl_task = cotask.Task(
       left_motor.run,
       name="left_mot_ctrl",
       priority=3,
       period=12,  # 12ms = ~83Hz
       profile=True,
       trace=False
   )

   right_mot_ctrl_task = cotask.Task(
       right_motor.run,
       name="right_mot_ctrl",
       priority=3,
       period=12,
       profile=True,
       trace=False
   )

- **Period**: 12ms (83.3 Hz update rate)
- **Priority**: 3 (high priority)
- **Function**: Motor PWM control and encoder data collection

Closed-Loop Control Tasks
^^^^^^^^^^^^^^^^^^^^^^^^^^
.. code-block:: python

   left_CL_control_task = cotask.Task(
       left_controller.run,
       name="left_CL_ctrl",
       priority=3,
       period=20,  # 20ms = 50Hz
       profile=True,
       trace=False
   )

   right_CL_control_task = cotask.Task(
       right_controller.run,
       name="right_CL_ctrl",
       priority=3,
       period=20,
       profile=True,
       trace=False
   )

- **Period**: 20ms (50 Hz update rate)
- **Priority**: 3 (high priority)
- **Function**: PI velocity control with line following

Navigation Task
^^^^^^^^^^^^^^^
.. code-block:: python

   navigation_task = cotask.Task(
       navigation_task_fun,
       name="navigation",
       priority=4,  # Highest priority
       period=50,
       profile=True,
       trace=False
   )

- **Period**: 50ms (20 Hz update rate)
- **Priority**: 4 (highest priority for state machine coordination)
- **Function**: 24-state obstacle course navigation FSM

Task Scheduling Summary
-----------------------

+-------------------+------------+---------------+---------------------------+
| Task              | Period (ms)| Frequency (Hz)| Function                  |
+===================+============+===============+===========================+
| Motor Control     | 12         | 83.3          | PWM and encoder updates   |
+-------------------+------------+---------------+---------------------------+
| CL Control        | 20         | 50.0          | PI velocity control       |
+-------------------+------------+---------------+---------------------------+
| Bump Sensor       | 20         | 50.0          | Collision detection       |
+-------------------+------------+---------------+---------------------------+
| Encoder Heading   | 50         | 20.0          | Odometry calculation      |
+-------------------+------------+---------------+---------------------------+
| Navigation        | 50         | 20.0          | State machine             |
+-------------------+------------+---------------+---------------------------+
| Serial            | 150        | 6.7           | Bluetooth communication   |
+-------------------+------------+---------------+---------------------------+

Main Loop
---------

.. code-block:: python

   if __name__ == "__main__":
       print("\n\nStarting Simple Navigation with Bump Sensor...\n")
       
       # Hardware initialization
       # Task creation
       # Add tasks to scheduler
       
       gc.collect()  # Run garbage collector
       
       while True:
           try:
               cotask.task_list.pri_sched()
           except KeyboardInterrupt:
               print(cotask.task_list)
               break

--------

motor.py
========

Overview
--------

DC motor driver using PWM control with directional control. Implements H-bridge motor control with enable/disable and bidirectional drive capability.

Hardware
--------
- **H-Bridge**: DRV8847 or similar motor driver IC
- **PWM Timer**: Hardware timer with configurable frequency (typically 30kHz)
- **Control Pins**: DIR (direction), nSLP (sleep/enable), PWM (speed)

Classes
-------

motor_driver
^^^^^^^^^^^^

H-bridge motor driver with PWM speed control and directional control.

.. code-block:: python

   class motor_driver:
       def __init__(self, PWM_pin: Pin, DIR_pin: Pin, nSLP_pin: Pin, 
                    tim: Timer, chan: int)

Constructor Parameters
""""""""""""""""""""""

:param PWM_pin: Pin for PWM signal
:type PWM_pin: Pin
:param DIR_pin: Pin for direction control
:type DIR_pin: Pin
:param nSLP_pin: Pin for enable/sleep control (active high)
:type nSLP_pin: Pin
:param tim: Timer object for PWM generation
:type tim: Timer
:param chan: Timer channel number for PWM output
:type chan: int

Methods
"""""""

.. py:method:: enable()

   Enable the motor driver by setting nSLP pin high.

.. py:method:: disable()

   Disable the motor driver by setting nSLP pin low.

.. py:method:: set_effort(effort: float)

   Set motor effort with direction control.
   
   :param effort: Motor effort (-100 to +100, positive=forward, negative=reverse)
   :type effort: float

Notes
-----
- Positive effort = DIR low (forward direction)
- Negative effort = DIR high (reverse direction)
- Effort range: -100 to +100 (percent duty cycle)
- nSLP pin must be high to enable motor driver

--------

encoder.py
==========

Overview
--------

Quadrature encoder interface using hardware timer with velocity averaging. Handles 16-bit counter rollover and provides 5-point moving average velocity calculation.

Hardware
--------
- **Timer**: Hardware timer in encoder mode (ENC_AB)
- **Channels**: A and B quadrature signals
- **Counter Resolution**: 16-bit (0-65535)

Classes
-------

Encoder
^^^^^^^

Quadrature encoder driver with position tracking and averaged velocity.

.. code-block:: python

   class Encoder:
       def __init__(self, tim, chA_pin, chB_pin)

Constructor Parameters
""""""""""""""""""""""

:param tim: Hardware timer object
:type tim: Timer
:param chA_pin: Pin for encoder channel A
:type chA_pin: Pin
:param chB_pin: Pin for encoder channel B
:type chB_pin: Pin

Attributes
""""""""""

:ivar position: Accumulated encoder position (ticks)
:vartype position: int
:ivar prev_count: Previous timer counter value
:vartype prev_count: int
:ivar delta: Change in position since last update
:vartype delta: int
:ivar dt: Time elapsed since last update (microseconds)
:vartype dt: int

Methods
"""""""

.. py:method:: update()

   Update position and velocity from timer counter with rollover handling.
   
   Calculates delta position handling 16-bit wraparound, updates velocity buffer
   with 5-point moving average.

.. py:method:: get_position()

   Get current encoder position in ticks.
   
   :return: Accumulated position in ticks
   :rtype: int

.. py:method:: get_velocity()

   Get 5-point averaged velocity.
   
   :return: Velocity in ticks per microsecond
   :rtype: float

.. py:method:: zero()

   Reset encoder position to zero.

Notes
-----
- Position increments are inverted (subtracted) for correct direction
- Velocity averaged over 5 samples to reduce noise
- Handles timer overflow/underflow automatically

--------

CL_control_task_V5.py
=====================

Overview
--------

Closed-loop PI control task for dual motor velocity and heading control. Implements velocity PI control with line-following adjustment and heading-based turning control.

Hardware
--------
- **IR Sensor Array**: 8 reflectance sensors for line detection
- **Motor Encoders**: 1437.1 ticks/rev
- **Wheel Diameter**: 70mm (35mm radius)

Classes
-------

CL_control
^^^^^^^^^^

Closed-loop PI controller for a single motor with line-following capability.

.. code-block:: python

   class CL_control:
       def __init__(self, multiple_ir_readings_object, motor)

Constructor Parameters
""""""""""""""""""""""

:param multiple_ir_readings_object: IR sensor array object
:type multiple_ir_readings_object: multiple_ir_readings
:param motor: Motor identifier ("LEFT" or "RIGHT")
:type motor: str

States
""""""

:S0_INIT: Initialize control parameters
:S1_RUN: Main control loop with line following
:S2_REST: Motors stopped
:S3_TURN: Heading-based turning control

Control Parameters
""""""""""""""""""

Velocity Control
''''''''''''''''
- **Kp**: 0.07 (proportional gain)
- **Ki**: 0.035 (integral gain)
- **U_MAX**: 100 (maximum control output)
- **Battery Scaling**: Gains scaled by v_nom/v_battery for consistent performance

Line Following
''''''''''''''
- **ln_const**: 1.7 (line error proportional gain)
- **ln_integ_const**: 1.0 (line error integral gain)

Turning Control
'''''''''''''''
- **Kp_turn**: 0.45 (heading proportional gain)
- **Ki_turn**: 0.01 (heading integral gain)
- **TURN_EFFORT_MAX**: 20 (maximum turning effort)

Methods
"""""""

.. py:method:: run(shares)

   Cooperative task function implementing PI velocity control with line following.
   
   :param shares: Tuple of shared variables for inter-task communication
   :type shares: tuple

Features
--------

Anti-Windup
^^^^^^^^^^^
Prevents integral accumulation during saturation:

.. code-block:: python

   satur_hi = (u >= U_MAX - 1e-9)
   satur_lo = (u <= -U_MAX + 1e-9)
   if (not satur_hi or e < 0) and (not satur_lo or e > 0):
       integ += e * dt
       ln_integral += ln_error * dt

Battery Voltage Compensation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Scales gains to maintain consistent performance across battery discharge:

.. code-block:: python

   Kp = v_nom/batt_v * Kp
   Ki = v_nom/batt_v * Ki

Force Straight Mode
^^^^^^^^^^^^^^^^^^^
Overrides line-following for precise heading control:

.. code-block:: python

   if force_straight_flg.get() == 1:
       ln_error = 0.0  # Disable line following adjustment

Notes
-----
- Gains are scaled by battery voltage for consistent performance
- Anti-windup prevents integral accumulation during saturation
- Force-straight mode overrides line-following for precise heading control

--------

line_follow_V1.py
=================

Overview
--------

Line following controller using centroid-based error calculation. Computes steering error from IR sensor array with configurable bias for directional preference.

Hardware
--------
- **IR Sensor Array**: 7 sensors (indexed -3 to +3 from left to right)
- **ADC Resolution**: 12-bit (0-4095)

Classes
-------

LineFollower
^^^^^^^^^^^^

Line following controller with weighted centroid error calculation.

.. code-block:: python

   class LineFollower:
       def __init__(self, multi_sensor_read_object, black, white, bias=0.0)

Constructor Parameters
""""""""""""""""""""""

:param multi_sensor_read_object: IR sensor array object
:type multi_sensor_read_object: multiple_ir_readings
:param black: ADC value for black surface (line)
:type black: int
:param white: ADC value for white surface (background)
:type white: int
:param bias: Steering bias (positive=right, negative=left, range: -3 to +3)
:type bias: float

Attributes
""""""""""

:ivar weights: Sensor position weights [-3, -2, -1, 0, 1, 2, 3]
:vartype weights: list

Methods
"""""""

.. py:method:: calculate_error()

   Calculate steering error using weighted centroid of normalized sensor readings.
   
   :return: Steering error (-3 to +3, including bias)
   :rtype: float
   
   Algorithm:
   
   1. Normalize each sensor reading: ``norm = (reading - white) / (black - white)``
   2. Clamp to [0, 1]
   3. Calculate weighted centroid: ``error = sum(weight * norm) / sum(norm)``
   4. Add bias: ``biased_error = error + bias``

.. py:method:: set_bias(bias)

   Update steering bias dynamically.
   
   :param bias: New steering bias value
   :type bias: float

.. py:method:: get_raw_readings()

   Get raw sensor readings for debugging or calibration.
   
   :return: List of raw ADC values from all sensors
   :rtype: list

.. py:function:: calibrate(multi_sensor_read_object)

   Calculate average sensor reading for calibration.
   
   :param multi_sensor_read_object: IR sensor array object
   :type multi_sensor_read_object: multiple_ir_readings
   :return: Average ADC value across all 7 sensors
   :rtype: float

Error Convention
----------------
- **Positive error**: Line to the right → turn right
- **Negative error**: Line to the left → turn left
- **Bias**: Allows preferential steering for forks or curves

--------

navigation_v2_compact_encoder.py
=================================

Overview
--------

Compact obstacle course navigation using encoder-based odometry. Implements 24-state finite state machine for autonomous navigation through complex course with bump sensor collision recovery.

States
------

Course Navigation (States 0-15)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

:State 0: Line following at bias=0 until 650mm
:State 1: Fork quarter-circle line following (bias=1.55) until 914mm
:State 2: Turn to -90°
:State 3: Straight at -90° through diamond until 1114mm
:State 4: U-turn semicircle line following until 1742mm
:State 5: Turn to 90°
:State 6: Straight at 90° through parking garage until 1992mm
:State 7: Fork quarter-circle (bias=1.8) until 2227mm
:State 8: Complex section with 2x quarter circles until 3549mm
:State 9: Turn to 180°
:State 10: Straight at 180° until 3849mm
:State 11: Line following (bias=0.2) until 4099mm
:State 12: Turn to 180°
:State 13: Straight through parking garage until 4799mm
:State 14: Turn to 90°
:State 15: Straight until bump detected or 5149mm

Wall Recovery Sequence (States 16-23)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Triggered when bump sensor detects wall during State 15:

:State 16: Back up 50mm
:State 17: Turn to 0°
:State 18: Go straight 175mm at 0°
:State 19: Turn to 90°
:State 20: Go straight 200mm at 90°
:State 21: Turn to 180°
:State 22: Line following 200mm at bias=0
:State 23: STOP - Course complete

Functions
---------

.. py:function:: navigation_task_fun(shares)

   Cooperative task function implementing 24-state obstacle course FSM.
   
   :param shares: Tuple containing all navigation-related shared variables
   :type shares: tuple

Helper Functions
^^^^^^^^^^^^^^^^

.. py:function:: calc_heading_error(current_heading, target_heading)

   Calculate normalized heading error in range [-180, 180].
   
   :param current_heading: Current robot heading in degrees
   :type current_heading: float
   :param target_heading: Desired heading in degrees
   :type target_heading: float
   :return: Heading error in degrees
   :rtype: float

Configuration Parameters
------------------------

Distance Thresholds (mm)
^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: python

   STATE_0_DIST = 650.0          # Start to fork entrance
   STATE_1_DIST = 914.0          # Fork quarter-circle
   STATE_3_DIST = 1114.0         # Through diamond
   STATE_4_DIST = 1742.0         # After U-turn
   STATE_6_DIST = 1992.0         # Before second fork
   STATE_7_DIST = 2227.0         # Second fork complete
   STATE_8_DIST = 3549.0         # Complex section complete
   STATE_10_DIST = 3849.0        # Approach to finish
   STATE_11_DIST = 4099.0        # Enter parking garage
   STATE_13_DIST = 4799.0        # Exit parking garage
   STATE_15_DIST = 5149.0        # Maximum distance

Heading Targets
^^^^^^^^^^^^^^^

.. code-block:: python

   HEADING_NEG_90 = -90.0        # Diamond section
   HEADING_90 = 90.0             # Parking garage exit
   HEADING_180 = 180.0           # Return path
   HEADING_TOLERANCE = 4.0       # ±4° acceptance

Control Modes
-------------

Line Following Mode
^^^^^^^^^^^^^^^^^^^
- Sets ``line_follow_flg = 1``
- Uses IR sensor centroid tracking
- Configurable bias for directional preference

Force Straight Mode
^^^^^^^^^^^^^^^^^^^
- Sets ``force_straight_flg = 1``
- Disables line following
- Maintains precise heading using encoder feedback

Turning Mode
^^^^^^^^^^^^
- Sets ``nav_turn_flg = 1``
- Uses heading-based PI control
- Zero velocity during turn

Notes
-----
- Uses encoder distance (mm) instead of X/Y coordinates
- Uses encoder-calculated heading instead of IMU
- Bump sensor triggers wall recovery sequence
- State transitions based on distance thresholds and heading targets

--------

encoder_heading_task.py
=======================

Overview
--------

Odometry task for calculating robot heading and distance from encoder data. Computes heading from encoder differential and distance from average encoder ticks.

Hardware
--------
- **Motor Encoders**: 1437.1 ticks/rev
- **Wheel Radius**: 35mm
- **Track Width**: 141mm

Functions
---------

.. py:function:: encoder_heading_task_fun(shares)

   Cooperative task function that computes heading and distance from encoders.
   
   :param shares: Tuple of (left_enc_pos, right_enc_pos, enc_heading_share, enc_distance_share)
   :type shares: tuple

Constants
---------

.. code-block:: python

   WHEEL_RADIUS_M = 0.035        # Wheel radius in meters
   TICKS_PER_REV = 1437.1        # Encoder ticks per revolution
   TRACK_WIDTH_M = 0.141         # Distance between wheels in meters
   WHEEL_CIRC_M = 0.21991        # Wheel circumference in meters
   MM_PER_TICK = 0.1530          # Millimeters per encoder tick

Calculations
------------

Distance
^^^^^^^^
.. code-block:: python

   avg_ticks = (left_ticks + right_ticks) / 2.0
   distance_mm = avg_ticks * MM_PER_TICK

Heading
^^^^^^^
.. code-block:: python

   tick_diff = left_ticks - right_ticks
   heading_rad = (tick_diff / TICKS_PER_REV) * (WHEEL_CIRC_M / TRACK_WIDTH_M)
   heading_deg = heading_rad * (180.0 / pi)

Heading Unwrapping
^^^^^^^^^^^^^^^^^^
Handles ±180° discontinuities smoothly:

.. code-block:: python

   delta = heading_deg - prev_heading
   if delta > 180:
       delta -= 360
   elif delta < -180:
       delta += 360
   continuous_heading += delta

Notes
-----
- Positive heading = counter-clockwise rotation (left turn)
- Heading normalized to [-180, 180] degrees
- Distance calculated as average of left and right encoder ticks
- Uses unwrapping to prevent discontinuities at ±180°

--------

bump_sensor_task.py
===================

Overview
--------

Bump sensor monitoring task for collision detection. Continuously polls bump sensor and sets shared flag when collision is detected.

Hardware
--------
- **Bump Sensor Pin**: PC11 (active LOW, pull-up resistor enabled)
- **Debounce Time**: 30ms

Functions
---------

.. py:function:: bump_sensor_task_fun(shares)

   Cooperative task function that monitors bump sensor and updates shared state.
   
   :param shares: Tuple containing bump_detected_share (Share object)
   :type shares: tuple

Algorithm
---------

1. Read bump sensor pin state
2. If pressed (LOW) and not already detected:
   
   a. Wait 30ms (debounce)
   b. Confirm still pressed
   c. Set ``bump_detected_share = 1``
   d. Set ``bump_already_detected = True``
   e. Print detection message

3. Yield to scheduler

Notes
-----
- Sensor is active LOW (pressed = 0, released = 1)
- Only triggers once per run (bump_already_detected flag)
- Requires cotask scheduler to yield control
- 30ms debounce prevents false triggers

--------

battery_adc.py
==============

Overview
--------

Battery voltage monitoring module using ADC (Analog-to-Digital Converter). Reads voltage directly from pin with configurable averaging and sampling.

Hardware
--------
- **ADC Pin**: PC0 (default, configurable)
- **Reference Voltage**: 3.3V
- **Resolution**: 12-bit ADC (0-4095)

Classes
-------

BatteryMonitor
^^^^^^^^^^^^^^

ADC-based battery voltage monitor with configurable averaging.

.. code-block::
