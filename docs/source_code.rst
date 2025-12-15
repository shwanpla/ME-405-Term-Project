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

   State 0:  Line follow (bias=0)          to 650 mm
   State 1:  Fork quarter-circle (bias=1.55) to 914 mm
   State 2:  Turn to heading -90 deg
   State 3:  Straight at -90 deg          to 1114 mm (diamond section)
   State 4:  U-turn semicircle            to 1742 mm
   State 5:  Turn to heading 90 deg
   State 6:  Straight at 90 deg           to 1992 mm (parking garage)
   State 7:  Fork quarter-circle (bias=1.8) to 2227 mm
   State 8:  Complex 2x quarter-circles   to 3549 mm
   State 9:  Turn to heading 180 deg
   State 10: Straight at 180 deg          to 3849 mm
   State 11: Line follow (bias=0.2)       to 4099 mm
   State 12: Turn to heading 180 deg
   State 13: Straight through parking     to 4799 mm
   State 14: Turn to heading 90 deg
   State 15: Straight until bump or       to 5149 mm

Wall Recovery Sequence (States 16–23)
""""""""""""""""""""""""""""""""""""""

When bump sensor detects collision during State 15:

.. code-block:: python

   State 16: Back up 50 mm
   State 17: Turn to heading 0 deg
   State 18: Go straight 175 mm at 0 deg
   State 19: Turn to heading 90 deg
   State 20: Go straight 200 mm at 90 deg
   State 21: Turn to heading 180 deg
   State 22: Line follow 200 mm (bias=0)
   State 23: STOP - course complete

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
   HEADING_TOLERANCE = 4.0        # +/- 4 deg acceptance band

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

Distance Traveled
-----------------

.. math::

    \text{distance} = 
    \frac{\text{left_ticks} + \text{right_ticks}}{2}
    \cdot \text{mm_per_tick}

Average of both encoders smooths odometry error and provides forward displacement in millimeters.

Heading Angle
-------------

.. math::

    \text{ticks_diff} = \text{left_ticks} - \text{right_ticks}

.. math::

    \theta_{\text{rad}} =
    \frac{\text{ticks_diff}}{\text{TICKS_PER_REV}}
    \left( \frac{\text{WHEEL_CIRC_MM}}{\text{TRACK_WIDTH_MM}} \right)

.. math::

    \theta_{\text{deg}} = \theta_{\text{rad}} \cdot \frac{180}{\pi}



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
- Heading normalized to [-180, 180] deg range
- Heading unwrapping prevents jumps at +/- 180 deg boundary
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


IR Sensor Array
===============

The multi-sensor IR array provides simultaneous synchronized readings from all 7 sensors using timer-triggered ADC conversions. This ensures all sensors are sampled at the same instant for accurate centroid calculations.

.. automodule:: multi_sensor_read
   :members:
   :undoc-members:

multiple_ir_readings Class
---------------------------

.. py:class:: multiple_ir_readings(pin_1, pin_2, pin_3, pin_4, pin_5, pin_6, pin_7)

   Seven-sensor IR array interface with timer-synchronized ADC sampling.

   :param pin_1: Pin object for sensor 1 (leftmost)
   :type pin_1: pyb.Pin
   :param pin_2: Pin object for sensor 2
   :type pin_2: pyb.Pin
   :param pin_3: Pin object for sensor 3
   :type pin_3: pyb.Pin
   :param pin_4: Pin object for sensor 4 (center)
   :type pin_4: pyb.Pin
   :param pin_5: Pin object for sensor 5
   :type pin_5: pyb.Pin
   :param pin_6: Pin object for sensor 6
   :type pin_6: pyb.Pin
   :param pin_7: Pin object for sensor 7 (rightmost)
   :type pin_7: pyb.Pin

.. py:method:: read()

   Read all 7 IR sensors simultaneously using timed ADC sampling.

   Uses :code:`ADC.read_timed_multi()` to ensure all sensors are sampled at the exact same instant, eliminating timing skew that could affect centroid calculations.

   :return: List of 7 averaged ADC values [sensor1, sensor2, ..., sensor7]
   :rtype: list

IR Array Hardware
"""""""""""""""""

- **Sensors**: 7 analog IR reflectance sensors
- **ADC Resolution**: 12-bit (0-4095)
- **Sampling Timer**: Timer 8 at 100Hz
- **Buffer Size**: 1 (instantaneous readings without averaging)
- **Sensor Spacing**: ~8mm between sensors
- **Sensor Indexing**: 1 (left) to 7 (right)

Timed Multi-ADC Algorithm
""""""""""""""""""""""""""

The :code:`read_timed_multi()` function synchronizes all 7 ADC conversions to a single timer event:

1. Timer 8 triggers at 100Hz
2. All 7 ADCs sample simultaneously on timer event
3. Results stored in pre-allocated arrays
4. Average calculated (trivial with buffer_size=1)

This ensures temporal coherence across the sensor array, critical for accurate line position estimation.

--------


Motor Control Task
==================

The motor control task manages individual motors through a two-state FSM, handling initialization, encoder data collection, and motor effort updates. This task provides the foundation for velocity control.

.. automodule:: motor_ctrl_task_V3
   :members:
   :undoc-members:

motor_control_task Class
-------------------------

.. py:class:: motor_control_task(motor, encoder)

   Motor control task for single motor with encoder feedback and data logging.

   :param motor: Motor driver object
   :type motor: motor_driver
   :param encoder: Encoder object for position/velocity feedback
   :type encoder: Encoder

.. py:method:: run(shares)

   Cooperative task function implementing motor control state machine.

   Executes two-state FSM: initialize motor → continuous data collection and effort updates.

   :param shares: Tuple of shared variables for inter-task communication
   :type shares: tuple

   **Shared Variables:**

   - ``start_flg`` (Share): Start signal (0 or 1)
   - ``set_point`` (Share): Motor control effort (-100 to +100)
   - ``end_flg`` (Share): End signal (0 or 1)
   - ``enc_pos`` (Share): Encoder position output (ticks)
   - ``enc_speed`` (Share): Encoder velocity output (ticks/us)
   - ``exp_time`` (Share): Elapsed time output (microseconds)
   - ``desired_time_ms`` (Share): Maximum run time (milliseconds)
   - ``proc_data_flg`` (Share): Data processing flag
   - ``desired_vel`` (Share): Desired velocity setpoint

State Machine
"""""""""""""

**State 0: Motor Start/Initialization**

.. code-block:: python

   if start_flg == 1:
       - Zero encoder position
       - Enable motor driver
       - Record start time
       - Transition to State 1

**State 1: Continuous Data Collection**

.. code-block:: python

   While end_flg == 0:
       - Update encoder position and velocity
       - Apply motor effort from set_point
       - Track elapsed time
       - Check time limit
   When end_flg == 1:
       - Disable motor
       - Return to State 0

Motor Control Notes
"""""""""""""""""""

- Runs at 12ms period (highest frequency task)
- Handles divide-by-zero in velocity calculations
- Calls garbage collector to prevent memory fragmentation
- Time limit enforcement prevents runaway operation
- Zero-crossing initialization ensures clean startup

--------


Serial Communication
====================

The serial task provides Bluetooth command interface for calibration, parameter configuration, and trial control. It bridges the gap between user commands and robot operation.

.. automodule:: serial_task_V6
   :members:
   :undoc-members:

Serial Task Function
---------------------

.. py:function:: serial_task_fun(shares)

   Cooperative task function for Bluetooth serial communication and control.

   Implements three-state FSM: wait for commands → handle trials → send data (unused).

   :param shares: Tuple of shared variables for inter-task communication
   :type shares: tuple

   **Shared Variables:**

   - ``left_start_flg`` (Share): Left motor start flag
   - ``right_start_flg`` (Share): Right motor start flag
   - ``left_desired_vel`` (Share): Left motor velocity setpoint
   - ``right_desired_vel`` (Share): Right motor velocity setpoint
   - ``left_end_flg`` (Share): Left motor end flag
   - ``right_end_flg`` (Share): Right motor end flag
   - ``left_proc_data_flg`` (Share): Left data processing flag
   - ``right_proc_data_flg`` (Share): Right data processing flag
   - ``proc_pos_right`` (Queue): Right position data queue
   - ``proc_pos_left`` (Queue): Left position data queue
   - ``proc_speed_right`` (Queue): Right speed data queue
   - ``proc_speed_left`` (Queue): Left speed data queue
   - ``left_proc_time`` (Queue): Left timing data queue
   - ``right_proc_time`` (Queue): Right timing data queue
   - ``calibration_flg`` (Share): Line sensor calibration status (0-3)
   - ``observer_calibration_flg`` (Share): Observer calibration trigger

Command Protocol
""""""""""""""""

**PARAMS Command**

.. code-block:: text

   Format: PARAMS,trial_time,desired_velocity
   Example: PARAMS,0,50

   Sets trial duration and desired velocity for both motors.

**CALIBRATE_BLACK Command**

.. code-block:: text

   Format: CALIBRATE_BLACK

   Triggers black surface calibration (sets calibration_flg = 1).

**CALIBRATE_WHITE Command**

.. code-block:: text

   Format: CALIBRATE_WHITE

   Triggers white surface calibration (sets calibration_flg = 2).

**CALIBRATION_COMPLETE Command**

.. code-block:: text

   Format: CALIBRATION_COMPLETE

   Signals calibration finished (sets calibration_flg = 3).
   Automatically starts motors after this command.

**END_COMM Command**

.. code-block:: text

   Format: END_COMM

   Stops current trial and resets system to wait state.

Serial Hardware
"""""""""""""""

- **UART**: UART 1 (PA9/PA10)
- **Baud Rate**: 460800
- **Protocol**: ASCII text with newline termination
- **Module**: HC-05 Bluetooth (or similar)
- **Data Bits**: 8
- **Stop Bits**: 1
- **Parity**: None

State Machine Workflow
"""""""""""""""""""""""

**State 0: Wait for Commands**

- Listen for PARAMS, CALIBRATE_*, CALIBRATION_COMPLETE
- Parse parameters and update shared variables
- Transition to State 1 when calibration complete

**State 1: Handle Trials**

- Monitor for END_COMM command
- Check for trial completion (both end flags set)
- Clear data queues on trial completion
- Return to State 0 when trial ends

**State 2: Send Data** (Currently Unused)

- Reserved for future telemetry transmission

Serial Task Notes
"""""""""""""""""

- Runs at 150ms period (lowest priority)
- Automatic motor start after calibration
- Robust command parsing with exception handling
- Clears UART buffer after parameter reception
- Synchronizes left and right motor operations

--------


PC User Interface
=================

The PC-side user interface provides a simple command-line tool for controlling the robot via Bluetooth. This program runs on the user's laptop, sending calibration commands and monitoring robot output.

.. automodule:: UI_Line_Follow
   :members:
   :undoc-members:

Main Function
-------------

.. py:function:: main()

   Main program loop for robot control interface.

   Establishes serial connection to robot and runs trials based on user input. Handles connection setup, trial execution, and graceful shutdown.

Trial Execution Function
-------------------------

.. py:function:: run_trial(bt)

   Execute single trial with automatic calibration and monitoring.

   Sends calibration sequence, then continuously monitors and displays robot debug output until user interrupts with Ctrl+C.

   :param bt: Serial connection object to robot
   :type bt: serial.Serial

User Interface Workflow
------------------------

**1. Connection Establishment**

.. code-block:: python

   port = 'COM6'  # Windows
   # port = '/dev/tty.HC-05'  # macOS/Linux
   bt = serial.Serial(port, baudrate=460800, timeout=1)

**2. Trial Sequence**

.. code-block:: text

   User presses Enter
   ↓
   Send "PARAMS,0,50\n" (50 mm/s velocity)
   ↓
   Send "CALIBRATE_BLACK\n" (preset calibration)
   ↓
   Send "CALIBRATE_WHITE\n" (preset calibration)
   ↓
   Send "CALIBRATION_COMPLETE\n" (start robot)
   ↓
   Monitor MCU debug output (continuous)
   ↓
   User presses Ctrl+C
   ↓
   Send "END_COMM\n" (stop robot)

**3. Output Monitoring**

The interface continuously polls the serial port and displays all MCU output in real-time, providing visibility into robot state, navigation progress, and debug messages.

UI Configuration
"""""""""""""""""

**Serial Port**

- **Windows**: :code:`COM6` (check Device Manager)
- **macOS/Linux**: :code:`/dev/tty.HC-05` or :code:`/dev/ttyUSB0`

**Preset Parameters**

.. code-block:: python

   trial_time = 0        # Unlimited duration (until END_COMM)
   desired_velocity = 50  # mm/s forward velocity

**Timing Delays**

.. code-block:: python

   command_delay = 0.5s  # Between calibration commands
   poll_interval = 0.1s  # MCU output polling rate

UI Notes
""""""""

- **Platform**: Runs on user's PC (Windows/macOS/Linux)
- **Dependencies**: pyserial library (:code:`pip install pyserial`)
- **Calibration**: Uses preset values (no user input required)
- **Monitoring**: Real-time display of all MCU debug output
- **Termination**: Ctrl+C sends END_COMM and returns to prompt
- **Restart**: Press Enter to start another trial

Example Session
"""""""""""""""

.. code-block:: text

   $ python UI_Line_Follow.py
   Connected to COM6
   Press Enter to start bot...

   Sending: PARAMS,0,50
   Sent: CALIBRATE_BLACK (preset)
   Sent: CALIBRATE_WHITE (preset)
   Sent: CALIBRATION_COMPLETE
   MCU response: [OK] Starting navigation...

   Bot running (press Ctrl+C to stop)...
   MCU: [NAV] State 0 - Distance: 123.4 mm
   MCU: [NAV] State 1 - Distance: 678.9 mm
   ^C

   Stopping trial...
   Press Enter to start bot...

--------


Main Program
============

The main program orchestrates all subsystems through cooperative multitasking. It initializes hardware, creates shared variables, and launches the task scheduler for autonomous obstacle course navigation.

.. automodule:: main
   :members:
   :undoc-members:

System Architecture
-------------------

The main program coordinates eight cooperative tasks using the cotask scheduler:

1. **Serial Task** (Priority 3, Period 150ms)
   - Bluetooth command interface for calibration and control
   - Lowest period for non-critical communication

2. **Left Motor Control Task** (Priority 3, Period 12ms)
   - Updates left motor PWM and collects encoder data
   - Fastest task for responsive motor control

3. **Right Motor Control Task** (Priority 3, Period 12ms)
   - Updates right motor PWM and collects encoder data
   - Matches left motor for synchronized operation

4. **Left Closed-Loop Control Task** (Priority 3, Period 20ms)
   - PI velocity control for left motor
   - Line following and heading control integration

5. **Right Closed-Loop Control Task** (Priority 3, Period 20ms)
   - PI velocity control for right motor
   - Line following and heading control integration

6. **Encoder Heading Task** (Priority 3, Period 50ms)
   - Computes robot heading and distance from dual encoders
   - Provides odometry for navigation

7. **Navigation Task** (Priority 4, Period 50ms)
   - 24-state obstacle course FSM
   - Highest priority for critical decision-making

8. **Bump Sensor Task** (Priority 3, Period 20ms)
   - Collision detection and wall recovery trigger

Hardware Configuration
""""""""""""""""""""""

**Motors and Encoders:**

.. code-block:: python

   # Right motor on pins C9 (dir), H1 (PWM), H0 (nSLP), timer 3 channel 4
   mot_right = motor_driver(Pin.cpu.C9, Pin.cpu.H1, Pin.cpu.H0, Timer(3, freq=30_000), 4)

   # Left motor on pins B0 (dir), C12 (PWM), C10 (nSLP), timer 3 channel 3
   mot_left = motor_driver(Pin.cpu.B0, Pin.cpu.C12, Pin.cpu.C10, Timer(3, freq=30_000), 3)

   # Right encoder on pins A8, A9 (timer 1)
   enc_right = Encoder(Timer(1, prescaler=0, period=0xFFFF), Pin.cpu.A8, Pin.cpu.A9)

   # Left encoder on pins A0, B3 (timer 2)
   enc_left = Encoder(Timer(2, prescaler=0, period=0xFFFF), Pin.cpu.A0, Pin.cpu.B3)

**IR Sensor Array:**

.. code-block:: python

   # 7 IR reflectance sensors for line detection
   ir_pins = [Pin.cpu.C4, Pin.cpu.B1, Pin.cpu.A7, Pin.cpu.C1,
              Pin.cpu.A4, Pin.cpu.A1, Pin.cpu.C3]
   ir_array = multiple_ir_readings(*ir_pins)

**Bump Sensor:**

.. code-block:: python

   # Collision detection sensor on pin PC11
   # Active LOW with pull-up resistor

**Robot Physical Parameters:**

- **Wheel Diameter**: 70mm (radius = 35mm)
- **Track Width**: 141mm
- **Encoder Resolution**: 1437.1 ticks/revolution
- **Wheel Circumference**: 220mm

Shared Variables
""""""""""""""""

The system uses 30+ shared variables for inter-task communication:

**Motor Control:**

- :code:`left_start_flg`, :code:`right_start_flg`: Motor start signals
- :code:`left_set_point`, :code:`right_set_point`: Control effort outputs
- :code:`left_desired_vel`, :code:`right_desired_vel`: Velocity setpoints
- :code:`left_end_flg`, :code:`right_end_flg`: Motor stop signals

**Encoder Feedback:**

- :code:`left_enc_pos`, :code:`right_enc_pos`: Position (ticks)
- :code:`left_enc_speed`, :code:`right_enc_speed`: Velocity (ticks/us)
- :code:`enc_heading_share`: Robot heading (degrees)
- :code:`enc_distance_share`: Distance traveled (mm)

**Navigation Control:**

- :code:`line_follow_flg`: Enable line following (0/1)
- :code:`force_straight_flg`: Force straight mode (0/1)
- :code:`nav_turn_flg`: Enable heading-based turning (0/1)
- :code:`desired_angle_share`: Target heading (degrees)
- :code:`bias_share`: Line following bias (-3 to +3)
- :code:`bump_detected_share`: Collision detection flag

**Calibration:**

- :code:`calibration_flg`: Line sensor calibration status (0-3)
- :code:`observer_calibration_flg`: Reserved for future use

**Controller Objects:**

- :code:`left_controller_share`: Left CL_control instance
- :code:`right_controller_share`: Right CL_control instance

Task Timing and Priorities
"""""""""""""""""""""""""""

The task scheduler executes tasks based on priority (higher number = higher priority) and period:

.. list-table::
   :header-rows: 1
   :widths: 35 15 15 35

   * - Task Name
     - Priority
     - Period (ms)
     - Function
   * - Navigation
     - 4
     - 50
     - Obstacle course FSM
   * - Serial
     - 3
     - 150
     - Bluetooth command interface
   * - Left Motor Control
     - 3
     - 12
     - Motor PWM and encoder
   * - Right Motor Control
     - 3
     - 12
     - Motor PWM and encoder
   * - Left CL Control
     - 3
     - 20
     - PI velocity control
   * - Right CL Control
     - 3
     - 20
     - PI velocity control
   * - Encoder Heading
     - 3
     - 50
     - Odometry calculation
   * - Bump Sensor
     - 3
     - 20
     - Collision detection

Startup Sequence
""""""""""""""""

The main program follows this initialization sequence:

1. **Hardware Initialization**:
   - Create motor driver objects with PWM timers
   - Initialize encoders with quadrature decoding
   - Set up IR sensor array with timer-triggered ADC
   - Configure bump sensor with pull-up resistor

2. **Object Creation**:
   - Create motor_control_task instances for both motors
   - Create CL_control instances for PI velocity control
   - Store controller objects in shared variables

3. **Shared Variable Creation**:
   - Create all 30+ shares for inter-task communication
   - Create queues for data logging (unused in final version)
   - Initialize calibration flags and control parameters

4. **Task Creation**:
   - Create 8 cooperative tasks with cotask.Task()
   - Assign priorities and periods for each task
   - Pass appropriate shares tuple to each task

5. **Scheduler Execution**:
   - Run garbage collector to free memory
   - Execute cotask.task_list.pri_sched() in infinite loop
   - Handle KeyboardInterrupt for graceful shutdown

Control Flow Example
""""""""""""""""""""

A typical control cycle showing task coordination:

.. code-block:: text

   Time 0ms:
   ├─ Left Motor Control (12ms task)
   │  ├─ Update encoder position and velocity
   │  ├─ Apply motor effort from set_point
   │  └─ Check time limits
   │
   ├─ Right Motor Control (12ms task)
   │  ├─ Update encoder position and velocity
   │  ├─ Apply motor effort from set_point
   │  └─ Check time limits

   Time 20ms:
   ├─ Left CL Control (20ms task)
   │  ├─ Read IR sensors for line position
   │  ├─ Calculate line following error
   │  ├─ Run PI velocity controller
   │  └─ Update left_set_point
   │
   ├─ Right CL Control (20ms task)
   │  ├─ Read IR sensors for line position
   │  ├─ Calculate line following error
   │  ├─ Run PI velocity controller
   │  └─ Update right_set_point
   │
   ├─ Bump Sensor (20ms task)
   │  ├─ Read bump sensor pin
   │  ├─ Debounce for 30ms if pressed
   │  └─ Set bump_detected_share if confirmed

   Time 50ms:
   ├─ Encoder Heading (50ms task)
   │  ├─ Read left and right encoder positions
   │  ├─ Calculate heading from differential
   │  ├─ Calculate distance from average
   │  └─ Update enc_heading_share, enc_distance_share
   │
   ├─ Navigation (50ms task - Priority 4)
   │  ├─ Read current distance and heading
   │  ├─ Check state transition conditions
   │  ├─ Update line_follow_flg, bias_share
   │  ├─ Set desired_angle_share for turns
   │  └─ Check bump_detected_share for recovery

   Time 150ms:
   ├─ Serial (150ms task)
   │  ├─ Check for Bluetooth commands
   │  ├─ Parse PARAMS, CALIBRATE_*, END_COMM
   │  └─ Update calibration_flg, velocity setpoints

Memory Management
"""""""""""""""""

The program includes memory management for MicroPython:

.. code-block:: python

   import gc
   gc.collect()  # Run before starting scheduler

This helps prevent memory fragmentation during long-running autonomous operation.

Main Program Notes
""""""""""""""""""

- **No IMU**: Uses pure encoder-based odometry (simplified from original approach)
- **Bump Recovery**: Automatic wall collision recovery in navigation FSM
- **Calibration**: Bluetooth-controlled line sensor calibration before each run
- **Modularity**: Each subsystem isolated in separate task with defined interfaces
- **Robustness**: Cooperative multitasking prevents blocking and missed deadlines
- **Debugging**: Serial output provides real-time visibility into robot state

--------
