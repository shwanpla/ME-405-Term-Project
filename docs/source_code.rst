===========
Source Code
===========

API Reference
-------------

This section provides detailed API documentation for each source code module, automatically generated from docstrings in the code. Each module includes class definitions, method signatures, parameters, return types, and usage notes.

Battery ADC Module
~~~~~~~~~~~~~~~~~~

The battery ADC module provides voltage monitoring functionality using the microcontroller's analog-to-digital converter. It reads voltage directly from pin PC0 with configurable averaging to reduce noise. The module offers both a class-based interface for custom configurations and convenience functions for quick voltage readings.

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
