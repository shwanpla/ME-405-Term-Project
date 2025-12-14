===========
Source Code
===========

API Reference
=============

This section documents the core modules from simple motor control up to complex navigation, automatically generated from source docstrings. Each section includes classes, methods, and usage parameters.

Motor Driver
============

The motor driver provides PWM control of DC motors through an H-bridge interface. This is the fundamental building block for velocity control across the platform.

.. automodule:: motor
   :members:
   :undoc-members:

Motor Driver Class
------------------

.. py:class:: motor_driver(PWM_pin, DIR_pin, nSLP_pin, tim, chan)

   H-bridge motor driver with PWM speed control and directional control.

   :param PWM_pin: Pin for PWM signal
   :type PWM_pin: pyb.Pin
   :param DIR_pin: Pin for direction control
   :type DIR_pin: pyb.Pin
   :param nSLP_pin: Pin for enable/sleep control (active high)
   :type nSLP_pin: pyb.Pin
   :param tim: Timer object for PWM generation
   :type tim: pyb.Timer
   :param chan: Timer channel number for PWM output
   :type chan: int

.. py:method:: enable()

   Enable the motor driver by setting nSLP pin high.

.. py:method:: disable()

   Disable the motor driver by setting nSLP pin low.

.. py:method:: set_effort(effort)

   Set motor effort with direction control.
   
   :param effort: Motor effort (-100 to +100, positive=forward, negative=reverse)
   :type effort: float

Motor Control Notes
"""""""""""""""""""

- Positive effort → DIR low (forward direction)
- Negative effort → DIR high (reverse direction)
- Effort range: -100 to +100 (percent duty cycle)
- nSLP pin must be high to enable driver IC

--------


Encoder
=======

Position and velocity feedback is critical for both motor control and navigation. The encoder module provides quadrature decoding with hardware timer support and averaged velocity calculation to smooth sensor noise.

.. automodule:: encoder
   :members:
   :undoc-members:

Encoder Class
-------------

.. py:class:: Encoder(tim, chA_pin, chB_pin)

   Quadrature encoder driver with position tracking and averaged velocity.

   :param tim: Hardware timer object configured in encoder mode
   :type tim: pyb.Timer
   :param chA_pin: Pin for encoder channel A
   :type chA_pin: pyb.Pin
   :param chB_pin: Pin for encoder channel B
   :type chB_pin: pyb.Pin

.. py:method:: update()

   Update position and velocity from timer counter with rollover handling.
   
   Calculates delta position handling 16-bit wraparound, updates velocity buffer with 5-point moving average.

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

Encoder Hardware Notes
""""""""""""""""""""""

- **Timer**: Must be configured in :code:`ENC_AB` encoder mode
- **Channels**: A and B quadrature signals on timer channels 1 and 2
- **Counter Resolution**: 16-bit (0-65535) with automatic rollover handling
- **Position**: Accumulates to avoid counter reset issues
- **Velocity**: 5-sample moving average reduces high-frequency noise

--------


Closed-Loop Velocity Control
=============================

With motors and encoders in place, we implement PI control to maintain desired velocities. The closed-loop controller handles both straight-line tracking with IR-based line following and heading-based turning for navigation.

.. automodule:: CL_control_task_V5
   :members:
   :undoc-members:

CL_control Class
----------------

.. py:class:: CL_control(multiple_ir_readings_object, motor)

   Closed-loop PI controller for a single motor with line-following capability.

   :param multiple_ir_readings_object: IR sensor array object for line sensing
   :type multiple_ir_readings_object: multiple_ir_readings
   :param motor: Motor identifier (``"LEFT"`` or ``"RIGHT"``)
   :type motor: str

.. py:method:: run(shares)

   Cooperative task function implementing PI velocity control with line following.
   
   Executes state machine: initialize → run (with line following) → optional rest and turning states.
   
   :param shares: Tuple containing all shared variables for inter-task communication
   :type shares: tuple

Velocity Control Parameters
"""""""""""""""""""""""""""

.. code-block:: python

   Kp = 0.07              # Proportional gain
   Ki = 0.035             # Integral gain
   U_MAX = 100            # Maximum control output (% effort)
   TICKS_PER_REV = 1437.1 # Encoder resolution
   WHEEL_CIRC_MM = 220    # Wheel circumference in mm

**Battery Compensation**: Gains are scaled by :math:`K = K_{nominal} \times \frac{V_{nominal}}{V_{battery}}` to maintain consistent performance as battery voltage drops.

Line Following Parameters
"""""""""""""""""""""""""

.. code-block:: python

   ln_const = 1.7         # Line error proportional gain
   ln_integ_const = 1.0   # Line error integral gain

When :code:`line_follow_flg = 1`, steering error from the IR sensor array modulates the differential velocity between left and right motors.

Heading Control Parameters
""""""""""""""""""""""""""

.. code-block:: python

   Kp_turn = 0.45         # Heading proportional gain
   Ki_turn = 0.01         # Heading integral gain
   TURN_EFFORT_MAX = 20   # Maximum turning effort

When :code:`nav_turn_flg = 1`, heading error from the encoder odometry drives motor differential.

Anti-Windup Protection
""""""""""""""""""""""

Prevents integral accumulation during saturation:

.. code-block:: python

   satur_hi = (u >= U_MAX - 1e-9)
   satur_lo = (u <= -U_MAX + 1e-9)
   if (not satur_hi or e < 0) and (not satur_lo or e > 0):
       integ += e * dt

Force-Straight Mode
"""""""""""""""""""

When :code:`force_straight_flg = 1`, line following is disabled (:code:`ln_error = 0`) and only heading-based control is active, providing precise directional control for turns.

--------


Line Following
===============

For the vehicle to track a line, we extract steering error from the IR sensor array using centroid-based weighting. This error then feeds into the closed-loop controller's differential velocity command.

.. automodule:: line_follow_V1
   :members:
   :undoc-members:

LineFollower Class
------------------

.. py:class:: LineFollower(multi_sensor_read_object, black, white, bias=0.0)

   Line following controller with weighted centroid error calculation.

   :param multi_sensor_read_object: IR sensor array object
   :type multi_sensor_read_object: multiple_ir_readings
   :param black: ADC value for black surface (line)
   :type black: int
   :param white: ADC value for white surface (background)
   :type white: int
   :param bias: Steering bias (positive=right, negative=left, range: -3 to +3)
   :type bias: float

.. py:method:: calculate_error()

   Calculate steering error using weighted centroid of normalized sensor readings.
   
   **Algorithm:**
   
   1. Normalize: :math:`norm = \frac{reading - white}{black - white}`, clamped to [0, 1]
   2. Compute centroid: :math:`error = \frac{\sum w_i \cdot norm_i}{\sum norm_i}`
   3. Apply bias: :math:`final\_error = error + bias`
   
   :return: Steering error (-3 to +3, including bias)
   :rtype: float

.. py:method:: set_bias(bias)

   Update steering bias dynamically for curves or fork detection.
   
   :param bias: New steering bias value
   :type bias: float

.. py:method:: get_raw_readings()

   Get raw sensor readings for debugging or calibration.
   
   :return: List of raw ADC values from all sensors
   :rtype: list

.. py:function:: calibrate(multi_sensor_read_object)

   Calculate average sensor reading for calibration reference.
   
   :param multi_sensor_read_object: IR sensor array object
   :type multi_sensor_read_object: multiple_ir_readings
   :return: Average ADC value across all 7 sensors
   :rtype: float

Error Convention
"""""""""""""""

- **Positive error**: Line to the right → turn right
- **Negative error**: Line to the left → turn left
- **Bias**: Allows preferential steering for complex course sections (forks, curves)

--------


Autonomous Navigation
======================

The navigation task orchestrates the entire system through a 24-state finite state machine. It sequences line following, heading control, and collision recovery to autonomously navigate the obstacle course.

.. automodule:: navigation_v2_compact_encoder
   :members:
   :undoc-members:

Navigation Task Function
------------------------

.. py:function:: navigation_task_fun(shares)

   Cooperative task function implementing 24-state obstacle course FSM.
   
   Handles course progression, state transitions based on distance/heading thresholds, and wall collision recovery.
   
   :param shares: Tuple containing all navigation-related shared variables
   :type shares: tuple

Course Navigation States (0–15)
"""""""""""""""""""""""""""""""

The robot progresses through distinct course sections, each with distance targets and heading setpoints:

.. code-block:: python

   State 0:  Line follow (bias=0)          → 650 mm
   State 1:  Fork quarter-circle (bias=1.55) → 914 mm
   State 2:  Turn to heading -90°
   State 3:  Straight at -90°            → 1114 mm (diamond section)
   State 4:  U-turn semicircle            → 1742 mm
   State 5:  Turn to heading 90°
   State 6:  Straight at 90°              → 1992 mm (parking garage)
   State 7:  Fork quarter-circle (bias=1.8) → 2227 mm
   State 8:  Complex 2× quarter-circles   → 3549 mm
   State 9:  Turn to heading 180°
   State 10: Straight at 180°             → 3849 mm
   State 11: Line follow (bias=0.2)       → 4099 mm
   State 12: Turn to heading 180°
   State 13: Straight through parking     → 4799 mm
   State 14: Turn to heading 90°
   State 15: Straight until bump or      → 5149 mm

Wall Recovery Sequence (States 16–23)
""""""""""""""""""""""""""""""""""""""

When bump sensor detects collision during State 15:

.. code-block:: python

   State 16: Back up 50 mm
   State 17: Turn to heading 0°
   State 18: Go straight 175 mm at 0°
   State 19: Turn to heading 90°
   State 20: Go straight 200 mm at 90°
   State 21: Turn to heading 180°
   State 22: Line follow 200 mm (bias=0)
   State 23: STOP — course complete

Configuration Parameters
------------------------

Distance Thresholds (millimeters)
"""""""""""""""""""""""""""""""""

.. code-block:: python

   STATE_0_DIST   = 650.0         # Start to fork entrance
   STATE_1_DIST   = 914.0         # Fork quarter-circle
   STATE_3_DIST   = 1114.0        # Through diamond
   STATE_4_DIST   = 1742.0        # After U-turn
   STATE_6_DIST   = 1992.0        # Before second fork
   STATE_7_DIST   = 2227.0        # Second fork complete
   STATE_8_DIST   = 3549.0        # Complex section complete
   STATE_10_DIST  = 3849.0        # Approach finish
   STATE_11_DIST  = 4099.0        # Enter final parking
   STATE_13_DIST  = 4799.0        # Exit final parking
   STATE_15_DIST  = 5149.0        # Maximum distance

Heading Targets (degrees)
"""""""""""""""""""""""""

.. code-block:: python

   HEADING_NEG_90   = -90.0       # Diamond section heading
   HEADING_90       = 90.0        # Parking garage heading
   HEADING_180      = 180.0       # Return path heading
   HEADING_TOLERANCE = 4.0        # ±4° acceptance band

Control Modes
"""""""""""""

**Line Following Mode** (``line_follow_flg = 1``)
  Uses IR sensor centroid with configurable bias for directional tracking.

**Force Straight Mode** (``force_straight_flg = 1``)
  Disables line following; uses encoder-based heading control for precision turns.

**Turning Mode** (``nav_turn_flg = 1``)
  Heading-based PI control with zero velocity until target heading reached.

Helper Functions
""""""""""""""""

.. py:function:: calc_heading_error(current_heading, target_heading)

   Calculate normalized heading error in range [-180, 180].
   
   Handles angle wrapping to find shortest path to target heading.
   
   :param current_heading: Current robot heading in degrees
   :type current_heading: float
   :param target_heading: Desired heading in degrees
   :type target_heading: float
   :return: Heading error in degrees
   :rtype: float

Navigation Notes
""""""""""""""""

- Odometry uses encoder distance (not X/Y coordinates)
- Heading is encoder-calculated (not IMU-based)
- Bump sensor in State 15 triggers wall recovery sequence
- State transitions are distance-driven or heading-driven
- Bias parameter tunes line following for complex curves

--------


Encoder Odometry
=================

To navigate the course, we need to track position and heading. The encoder odometry task reads dual encoders and computes distance traveled and heading angle using kinematic relationships.

.. automodule:: encoder_heading_task
   :members:
   :undoc-members:

Encoder Odometry Function
--------------------------

.. py:function:: encoder_heading_task_fun(shares)

   Cooperative task function that computes heading and distance from encoders.
   
   Updates shared state from left and right encoder positions at each iteration.
   
   :param shares: Tuple of ``(left_enc_pos, right_enc_pos, enc_heading_share, enc_distance_share)``
   :type shares: tuple

Hardware Constants
""""""""""""""""""

.. code-block:: python

   WHEEL_RADIUS_MM = 35.0         # Wheel radius in millimeters
   TICKS_PER_REV = 1437.1         # Encoder ticks per complete revolution
   TRACK_WIDTH_MM = 141.0         # Distance between left and right wheels
   WHEEL_CIRC_MM = 220.0          # Wheel circumference: 2π × radius
   MM_PER_TICK = 0.153            # Millimeters per encoder tick

Odometry Calculations
"""""""""""""""""""""

**Distance Traveled**

.. math::

   \text{distance} = \frac{\text{left\_ticks} + \text{right\_ticks}}{2.0} \times \text{MM\_PER\_TICK}

Average of both encoders smooths odometry error and provides forward displacement in millimeters.

**Heading Angle**

.. math::

   \text{tick\_diff} = \text{left\_ticks} - \text{right\_ticks}

.. math::

   \text{heading\_rad} = \frac{\text{tick\_diff}}{\text{TICKS\_PER\_REV}} \times \frac{\text{WHEEL\_CIRC\_MM}}{\text{TRACK\_WIDTH\_MM}}

.. math::

   \text{heading\_deg} = \text{heading\_rad} \times \frac{180}{\pi}

Positive heading indicates counter-clockwise rotation (left turn).

**Heading Unwrapping**

To handle the ±180° discontinuity smoothly, delta heading is checked and adjusted:

.. code-block:: python

   delta = heading_deg - prev_heading
   if delta > 180:
       delta -= 360
   elif delta < -180:
       delta += 360
   continuous_heading += delta

Odometry Notes
""""""""""""""

- Positive heading = counter-clockwise rotation (left turn)
- Heading normalized to [-180, 180]° range
- Heading unwrapping prevents jumps at ±180° boundary
- Distance accumulation provides cumulative traveled distance

--------


Supporting Sensors
===================

Bump Sensor
-----------

.. automodule:: bump_sensor_task
   :members:
   :undoc-members:

.. py:function:: bump_sensor_task_fun(shares)

   Cooperative task function that monitors bump sensor and updates shared state.
   
   Detects wall collisions during navigation and signals recovery sequence.
   
   :param shares: Tuple containing ``bump_detected_share`` (Share object)
   :type shares: tuple

Bump Sensor Hardware
""""""""""""""""""""

- **Pin**: :code:`PC11` (active LOW, pull-up resistor enabled)
- **Debounce Time**: 30 ms
- **Trigger**: Pressed (logic 0) → sets :code:`bump_detected_share = 1`

Algorithm
"""""""""

1. Read bump sensor pin state
2. If pressed (LOW) and not already detected:

   a. Wait 30 ms (debounce time)
   b. Confirm still pressed
   c. Set :code:`bump_detected_share = 1`
   d. Set :code:`bump_already_detected = True` (prevents re-trigger)
   e. Print detection message

3. Yield to scheduler

Bump Sensor Notes
"""""""""""""""""

- Sensor is active LOW: pressed = 0, released = 1
- Only triggers once per run (one-shot behavior)
- 30 ms debounce prevents false triggers from noise
- Used in navigation State 15 to trigger wall recovery

Battery Monitor
---------------

.. automodule:: battery_adc
   :members:
   :undoc-members:

Battery voltage monitoring is critical for compensating motor control gains as battery discharges during operation.

.. py:class:: BatteryMonitor(adc_pin=None, samples=10)

   ADC-based battery voltage monitor with configurable averaging.

   :param adc_pin: ADC pin for voltage measurement (default: :code:`PC0`)
   :type adc_pin: pyb.Pin or str
   :param samples: Number of samples for averaging
   :type samples: int

Battery Hardware
""""""""""""""""

- **ADC Pin**: :code:`PC0` (default, configurable)
- **Reference Voltage**: 3.3 V
- **ADC Resolution**: 12-bit (0–4095)
- **Voltage Scaling**: Depends on voltage divider on battery circuit

.. py:function:: battery_voltage()

   Get current battery voltage (convenience function).
   
   :return: Battery voltage in volts
   :rtype: float

Battery Monitor Notes
"""""""""""""""""""""

- Voltage readings are averaged to reduce noise
- Used in ``CL_control`` to scale PI gains
- Maintains consistent control performance across battery discharge
- Nominal battery voltage: 3.07 V (example reference)

--------
