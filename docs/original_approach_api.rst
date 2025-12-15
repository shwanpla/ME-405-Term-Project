Original Approach API Reference
================================

This page documents the API for the original state-space observer implementation preserved in ``src/`` with the ``defunct_`` prefix. This code was not used in the final system but represents the initial sensor fusion approach combining IMU, encoders, and RK4 numerical integration.

.. note::
   This implementation was replaced by the displacement-based odometry approach documented in :doc:`tasks`. See :doc:`analysis` for the technical rationale behind this decision, including IMU communication issues and the pivot to encoder-only estimation.

IMU Driver
----------

The BNO055 IMU driver provides 9-axis absolute orientation sensing with software I2C implementation. This driver includes automatic calibration management, bus recovery, and retry logic for robust operation.

.. automodule:: defunct_IMU_driver
   :members:
   :undoc-members:
   :show-inheritance:

SoftI2C Class
"""""""""""""

.. py:class:: SoftI2C(scl_pin, sda_pin, delay_us=50)

   Software I2C implementation with improved timeout handling and bus recovery.

   :param scl_pin: SCL pin name (e.g., 'B13')
   :type scl_pin: str
   :param sda_pin: SDA pin name (e.g., 'B14')
   :type sda_pin: str
   :param delay_us: Bit timing delay in microseconds (default: 50)
   :type delay_us: int

.. py:method:: mem_read(nbytes, addr, reg)

   Read multiple bytes from a device register with automatic bus recovery.

   :param nbytes: Number of bytes to read
   :type nbytes: int
   :param addr: I2C device address (7-bit)
   :type addr: int
   :param reg: Register address to read from
   :type reg: int
   :return: Data read from register
   :rtype: bytes
   :raises OSError: If communication fails after bus recovery attempt

.. py:method:: mem_write(data, addr, reg)

   Write multiple bytes to a device register with automatic bus recovery.

   :param data: Data to write
   :type data: bytes
   :param addr: I2C device address (7-bit)
   :type addr: int
   :param reg: Register address to write to
   :type reg: int
   :raises OSError: If communication fails after bus recovery attempt

BNO055 Class
""""""""""""

.. py:class:: BNO055(scl_pin, sda_pin, addr=0x28, autodetect=True, retries=10, retry_delay_ms=20)

   Driver for BNO055 9-axis absolute orientation sensor using software I2C.

   :param scl_pin: SCL pin name (e.g., 'B13')
   :type scl_pin: str
   :param sda_pin: SDA pin name (e.g., 'B14')
   :type sda_pin: str
   :param addr: I2C address (0x28 or 0x29, default: 0x28)
   :type addr: int
   :param autodetect: Try both addresses if chip ID doesn't match (default: True)
   :type autodetect: bool
   :param retries: Number of retry attempts for I2C operations (default: 10)
   :type retries: int
   :param retry_delay_ms: Delay between retries in milliseconds (default: 20)
   :type retry_delay_ms: int
   :raises RuntimeError: If BNO055 not detected or wrong chip ID

.. py:method:: set_mode(fusion_mode)

   Change the BNO055 operating mode.

   :param fusion_mode: Operating mode constant (MODE_NDOF, MODE_IMU, etc.)
   :type fusion_mode: int

.. py:method:: get_calibration_status()

   Get calibration status for all sensor subsystems.

   :return: Calibration status with keys 'sys', 'gyro', 'accel', 'mag'. Each value ranges from 0 (uncalibrated) to 3 (fully calibrated).
   :rtype: dict

   **Example**::

      >>> status = imu.get_calibration_status()
      >>> print(status)
      {'sys': 3, 'gyro': 3, 'accel': 3, 'mag': 3}

.. py:method:: read_calibration_blob()

   Read calibration coefficients from BNO055 as binary blob.

   Temporarily switches to CONFIG mode, reads 22-byte calibration data, then restores previous operating mode.

   :return: 22-byte calibration blob
   :rtype: bytes

.. py:method:: write_calibration_blob(blob)

   Write calibration coefficients to BNO055 from binary blob.

   Temporarily switches to CONFIG mode, writes 22-byte calibration data, then restores previous operating mode.

   :param blob: Exactly 22 bytes of calibration data
   :type blob: bytes
   :raises ValueError: If blob is not exactly 22 bytes

.. py:method:: read_euler()

   Read Euler angles from BNO055.

   :return: (heading, roll, pitch) in degrees
   :rtype: tuple

   - **heading**: 0-360° (yaw, compass heading)
   - **roll**: -180 to +180° (rotation about X-axis)
   - **pitch**: -90 to +90° (rotation about Y-axis)

.. py:method:: get_heading()

   Get heading (yaw) angle in degrees.

   Convenience function that extracts only the heading from Euler angles.

   :return: Heading in degrees (0-360°, 0 = North in NDOF mode)
   :rtype: float

.. py:method:: read_angular_velocity()

   Read angular velocity (gyro rates) from BNO055.

   :return: (gx, gy, gz) angular velocities in degrees per second
   :rtype: tuple

   - **gx**: Roll rate (rotation about X-axis)
   - **gy**: Pitch rate (rotation about Y-axis)
   - **gz**: Yaw rate (rotation about Z-axis)

.. py:method:: get_yaw_rate()

   Get yaw rate (angular velocity about Z-axis) in degrees per second.

   Convenience function that extracts only the yaw rate from angular velocities.

   :return: Yaw rate in degrees per second (positive = counter-clockwise)
   :rtype: float

IMU Hardware Notes
""""""""""""""""""

- **Sensor**: BNO055 9-axis absolute orientation sensor
- **Interface**: Software I2C on pins B13 (SCL) and B14 (SDA)
- **Reset Pin**: B15 (active low)
- **I2C Address**: 0x28 (default) or 0x29 (alternate)
- **Operating Mode**: NDOF (9-DOF fusion with magnetometer)
- **Heading Range**: 0-360 degrees (0 = North in NDOF mode)
- **Angular Rates**: Degrees per second
- **Calibration Status**: 0 (uncalibrated) to 3 (fully calibrated)
- **Software I2C Delay**: 150 µs (tuned for reliability)

Calibration Manager
-------------------

Manages BNO055 IMU calibration persistence with file-based storage. Handles saving and loading calibration coefficients to/from the filesystem for automatic calibration restoration on startup.

.. automodule:: defunct_CalibrationManager
   :members:
   :undoc-members:
   :show-inheritance:

CalibrationManager Class
"""""""""""""""""""""""""

.. py:class:: CalibrationManager(imu)

   Manages BNO055 IMU calibration persistence with file-based storage.

   :param imu: BNO055 IMU driver instance
   :type imu: BNO055

   **Class Attributes:**

   - ``CALIB_FILE = "calibration.txt"``: Filename for calibration storage
   - ``CALIB_BLOB_LEN = 22``: Expected calibration blob length in bytes

.. py:method:: calib_blob_to_hex(blob)

   Convert calibration blob (bytes) to hexadecimal string representation.

   :param blob: Raw calibration data bytes from IMU
   :type blob: bytes
   :return: Hexadecimal string representation of calibration data
   :rtype: str

.. py:method:: hex_to_calib_blob(hex_str)

   Convert hexadecimal string back to calibration blob (bytes).

   :param hex_str: Hexadecimal string from calibration file
   :type hex_str: str
   :return: Calibration data as bytes
   :rtype: bytes

.. py:method:: save_calibration()

   Read calibration coefficients from IMU and save to persistent storage.

   Reads the 22-byte calibration blob from the BNO055 sensor and writes it to ``calibration.txt`` as a hexadecimal string.

   :return: True if calibration saved successfully, False otherwise
   :rtype: bool

   **Example**::

      >>> calib_mgr.save_calibration()
      ✓ Calibration saved to calibration.txt
        Data: A1B2C3D4E5F6...
      True

.. py:method:: load_calibration()

   Load calibration coefficients from file and write to IMU sensor.

   Reads calibration data from ``calibration.txt``, validates the format and length, then writes the 22-byte calibration blob to the BNO055 sensor.

   :return: True if calibration loaded successfully, False otherwise
   :rtype: bool

   **Validation:**

   - Checks if file exists in current directory
   - Validates blob length equals 22 bytes
   - Handles conversion from hex string to bytes

.. py:method:: calibration_exists()

   Check if calibration file exists in the current directory.

   :return: True if ``calibration.txt`` exists, False otherwise
   :rtype: bool

.. py:method:: delete_calibration()

   Delete stored calibration file (for testing purposes).

   Removes ``calibration.txt`` file from the filesystem if it exists. Used to force recalibration during testing and development.

   :return: True if file deleted successfully, False if file not found
   :rtype: bool

Calibration Workflow
""""""""""""""""""""

**Typical usage pattern:**

1. **Initialization**: Create ``CalibrationManager`` with BNO055 instance
2. **Check for existing calibration**: Use ``calibration_exists()``
3. **Load or calibrate**:

   - If calibration exists: Call ``load_calibration()`` to restore coefficients
   - If no calibration: Perform manual calibration, then call ``save_calibration()``

4. **Verification**: Check IMU calibration status reaches ``3/3`` for all subsystems

**File Format:**

The calibration file stores 22 bytes as a hexadecimal string:

.. code-block:: text

   A1B2C3D4E5F607080910111213141516171819202122

**Calibration Notes:**

- Calibration blob contains coefficients for accelerometer, gyroscope, and magnetometer
- Must be performed in NDOF mode for full 9-axis calibration
- Calibration persists across power cycles when saved to file
- System calibration status should reach ``3/3`` before saving

IMU Handler Task
----------------

Provides cooperative multitasking functions for managing BNO055 IMU initialization, calibration persistence, and continuous sensor data reading. The calibration task implements a state machine for automatic calibration management, while the monitor task provides continuous heading and yaw rate updates with heading unwrapping.

.. automodule:: defunct_IMU_handler
   :members:
   :undoc-members:
   :show-inheritance:

Calibration Task Function
""""""""""""""""""""""""""

.. py:function:: calibration_task_fun(shares)

   Cooperative task function implementing IMU calibration state machine.

   Manages the complete IMU initialization and calibration workflow using a 6-state finite state machine. Automatically detects existing calibration files, loads calibration if available, or guides the user through manual calibration if needed.

   :param shares: Task shares (currently unused, reserved for future use)
   :type shares: tuple
   :yields: Current state number (0-5)
   :ytype: int

   **State Machine:**

   - **S_INIT (0)**: Initialize BNO055 hardware and reset pin
   - **S_CHECK_FILE (1)**: Check for existing calibration.txt file
   - **S_LOAD_CALIB (2)**: Load calibration from file
   - **S_MANUAL_CALIB (3)**: Manual calibration mode (rotate IMU)
   - **S_SAVE_CALIB (4)**: Save calibration to file
   - **S_DONE (5)**: Calibration complete, idle state

   **Hardware Setup:**

   - Reset pin B15: Pulsed low then high to reset BNO055
   - I2C pins B13/B14: Software I2C communication
   - I2C address: 0x28 (default)

   **Calibration Requirements:**

   - System calibration status must reach 3/3
   - IMU must be rotated in all directions during manual calibration
   - Calibration data saved to calibration.txt for persistence

   **Example Console Output**::

      ==================================================
      === BNO055 Initialization ===
      ==================================================
      [OK] BNO055 hardware initialized

      === Checking for Calibration File ===
      [INFO] calibration.txt not found
      Manual calibration required (rotate IMU in all directions)

      === Manual Calibration Mode ===
      Waiting for full calibration (Sys: 3/3)...
      Rotate the IMU slowly in all directions...
      [OK] Fully calibrated! (Sys: 3/3)

      === Saving Calibration ===
      ✓ Calibration saved to calibration.txt

      ==================================================
      === Calibration Complete - Starting Main Loop ===
      ==================================================

IMU Monitor Task Function
""""""""""""""""""""""""""

.. py:function:: imu_monitor_task_fun(shares)

   Cooperative task function for continuous IMU data monitoring with heading unwrapping.

   Alternately reads heading and yaw rate from the BNO055 IMU, updating shared variables for navigation and control tasks. Implements heading unwrapping to produce continuous heading values that do not wrap at 0/360 degrees.

   :param shares: Task shared variables
   :type shares: tuple

   **Shared Variables:**

   - ``shares[0]`` (Share): IMU object share (BNO055 instance)
   - ``shares[1]`` (Share): Continuous heading output (float, degrees)
   - ``shares[2]`` (Share): Yaw rate output (float, deg/s)

   :yields: None (yields control to scheduler after each sensor read)

   **Heading Unwrapping Algorithm:**

   The function maintains a continuous heading by detecting and correcting wraparound at the 0/360 degree boundary:

   1. Read raw heading from IMU (0-360 degrees)
   2. Calculate delta from previous reading
   3. Detect wraparound:

      - If delta > 180: Crossed from 359 to 0 (subtract 360)
      - If delta < -180: Crossed from 0 to 359 (add 360)

   4. Add corrected delta to continuous heading
   5. Update previous heading for next iteration

   **Example Unwrapping:**

   .. code-block:: python

      # Heading crosses from 359° to 1°
      # Raw IMU readings:    359, 360, 0,   1,   2
      # Continuous output:   359, 360, 360, 361, 362

   **Timing:**

   - Alternates between heading and yaw rate reads each yield
   - Typical period: 70ms (14.3 Hz update rate)

   **Notes:**

   - Continuous heading can exceed 360° or go negative
   - Initial heading matches first IMU reading
   - Yaw rate is read directly without processing
   - IMU object must be calibrated before this task starts

IMU Handler Workflow
"""""""""""""""""""""

**System Startup Sequence:**

1. **Hardware Reset**: Pin B15 pulsed low/high to reset BNO055
2. **I2C Initialization**: Software I2C established on B13/B14
3. **Calibration Check**: Look for calibration.txt file
4. **Load or Calibrate**:

   - If file exists: Load calibration coefficients automatically
   - If no file: Enter manual calibration mode

5. **Manual Calibration** (if needed):

   - User rotates IMU in all directions
   - System monitors calibration status (0-3 for each subsystem)
   - Waits for Sys: 3/3 before proceeding

6. **Save Calibration**: Write 22-byte blob to calibration.txt
7. **Start Monitoring**: Launch imu_monitor_task_fun for continuous data

**Task Integration:**

.. code-block:: python

   # Typical task setup in main program
   imu_share = task_share.Share('O', name='imu')
   heading_share = task_share.Share('f', name='heading')
   yaw_rate_share = task_share.Share('f', name='yaw_rate')

   monitor_task = cotask.Task(
       imu_monitor_task_fun,
       name="imu_monitor",
       priority=3,
       period=70,
       shares=(imu_share, heading_share, yaw_rate_share)
   )

Navigation (Original)
---------------------

Position-based navigation task for obstacle course using observer state estimation. This navigation implementation uses global X, Y coordinates from the state observer and IMU heading to navigate the course through a 5-state finite state machine with position-based transitions.

.. automodule:: defunct_navigation
   :members:
   :undoc-members:
   :show-inheritance:

Navigation Task Function
"""""""""""""""""""""""""

.. py:function:: navigation_task_fun(shares)

   Cooperative task function implementing position-based obstacle course navigation.

   Implements a 5-state finite state machine that uses observer-estimated X, Y coordinates and IMU heading to navigate the obstacle course. Coordinates line following, heading control, and straight-line movement based on position thresholds.

   :param shares: Task shared variables containing navigation state and control
   :type shares: tuple
   :yields: Current state number (0-4)
   :ytype: int

   **Shared Variables:**

   - ``line_follow_flg`` (Share): Enable/disable line following (0 or 1)
   - ``big_X_share`` (Share): Observer X position estimate (mm)
   - ``big_Y_share`` (Share): Observer Y position estimate (mm)
   - ``obs_heading_share`` (Share): Observer heading estimate (degrees)
   - ``obs_yaw_rate_share`` (Share): Observer yaw rate estimate (deg/s)
   - ``left_controller_share`` (Share): Left motor controller object
   - ``right_controller_share`` (Share): Right motor controller object
   - ``left_set_point`` (Share): Left motor control effort output
   - ``right_set_point`` (Share): Right motor control effort output
   - ``left_desired_vel`` (Share): Left motor velocity setpoint
   - ``right_desired_vel`` (Share): Right motor velocity setpoint
   - ``desired_angle_share`` (Share): Target heading for heading control (degrees)
   - ``nav_rest_flg`` (Share): Navigation rest flag
   - ``end_flg`` (Share): End flag to stop navigation
   - ``bias_share`` (Share): Line following bias (-3 to +3)
   - ``force_straight_flg`` (Share): Force straight mode (disable line following)
   - ``bias_timer_flg`` (Share): Bias timer flag
   - ``nav_stop_flg`` (Share): Navigation stop flag
   - ``calibration_flg`` (Share): Observer calibration status (0-3)
   - ``nav_turn_flg`` (Share): Enable heading-based turning

State Machine Details
""""""""""""""""""""""

**State 0: Line Following with Dynamic Bias**

- Initial line following mode
- Bias = 0.0 initially, switches to 1.55 when X > 550mm
- Transition: When Y < 685mm, move to State 1

**State 1: Heading Adjustment**

- Line following with simultaneous heading control
- Target heading: 90 degrees
- Bias reset to 0.0
- ``nav_turn_flg`` enabled for heading control
- Transition: When |heading_error| < 4°, move to State 2

**State 2: Straight Line at 90 Degrees**

- Disable line following, enable ``force_straight`` mode
- Maintain 90-degree heading using heading control
- Transition: When Y < 500mm, move to State 3

**State 3: Standard Line Following**

- Line following mode with zero bias
- Force straight disabled
- Final navigation state for course completion

**State 4: Placeholder**

- Reserved for future waypoint navigation
- Currently inactive

Configuration Parameters
"""""""""""""""""""""""""

**Position Thresholds:**

.. code-block:: python

   BIAS_SWITCH_X = 550.0     # Switch to left bias when X > 550mm
   STATE_2_Y = 685.0         # Transition to heading adjustment
   STATE_3_Y = 500.0         # Transition to final line following

**Heading Control:**

.. code-block:: python

   HEADING_ERROR_TOLERANCE = 4.0  # Degrees tolerance for heading achieved

**Bias Values:**

.. code-block:: python

   left_bias_value = 1.55     # Bias for left curve navigation
   center_bias_value = 0.0    # No bias (centered line following)

Observer Integration
""""""""""""""""""""

The navigation task waits for observer initialization before enabling state transitions:

1. **Calibration Check**: ``calibration_flg == 3`` indicates observer is calibrated
2. **Position Check**: ``Y > 700mm`` indicates valid initial position estimate
3. **Enable Transitions**: After both conditions met, state transitions become active

This prevents false transitions during system startup when position estimates are initializing.

Control Modes
"""""""""""""

**Line Following Mode** (``line_follow_flg = 1``)
   Uses IR sensor centroid with configurable bias for line tracking.

**Force Straight Mode** (``force_straight_flg = 1``)
   Disables line following, uses pure heading control for straight movement.

**Heading Turn Mode** (``nav_turn_flg = 1``)
   Enables heading-based PI control for turning to target angle.

**Bias Control** (``bias_share``)
   Adjusts line following behavior: -3 (left) to 0 (center) to +3 (right).

Example Console Output
"""""""""""""""""""""""

.. code-block:: text

   [NAV_TASK] STATE 0 - X: 234.5 Y: 756.2 Heading: 12.3° Bias: 0.000
   [NAV_TASK] X > 550 - Switching to bias 1.55
   [NAV_TASK] STATE 0 - X: 567.8 Y: 723.1 Heading: 45.6° Bias: 1.550
   [NAV_TASK] Y < 685 - Transitioning to STATE 1 (heading adjustment)
   [NAV_TASK] STATE 1 - X: 589.2 Y: 672.3 Heading: 78.9° (Target: 90.0°) Error: 11.1°
   [NAV_TASK] Heading reached 90° - Transitioning to STATE 2 (straight line)
   [NAV_TASK] STATE 2 - X: 612.4 Y: 543.7 Heading: 91.2°
   [NAV_TASK] Y < 500 - Transitioning to STATE 3 (line following)
   [NAV_TASK] STATE 3 - X: 645.8 Y: 456.9 Heading: 88.7°

Navigation Notes
""""""""""""""""

- **One-Way Transitions**: State transitions are irreversible (cannot return to previous states)
- **Observer Dependency**: Navigation requires observer to provide valid X, Y, heading estimates
- **Heading Wraparound**: Heading error calculation handles 360/0 degree boundary correctly
- **Debug Output**: Console messages print every 10th iteration to reduce output volume
- **Coordinate System**: X increases forward, Y increases to the right (course-specific)

State Observer
--------------

State observer task for robot localization using sensor fusion and RK4 integration. This module implements a cooperative multitasking observer function that fuses IMU heading measurements with encoder velocity data to estimate the robot's global position (X, Y), heading, and arc length traveled.

.. automodule:: defunct_observer_fcn
   :members:
   :undoc-members:
   :show-inheritance:

Observer Task Function
"""""""""""""""""""""""

.. py:function:: observer_task_fcn(shares)

   Cooperative task function implementing state observer with RK4 integration.

   Fuses IMU heading measurements with encoder velocity data to estimate the robot's global position, heading, and distance traveled. Uses 4th-order Runge-Kutta numerical integration to solve the robot's kinematic equations, providing accurate state estimation for navigation.

   :param shares: Task shared variables for sensor inputs and state outputs
   :type shares: tuple
   :yields: None (yields control to scheduler after each update cycle)

   **Shared Variables (14 inputs/outputs):**

   - ``meas_heading_share`` (Share): IMU heading measurement (degrees)
   - ``meas_yaw_rate_share`` (Share): IMU yaw rate measurement (deg/s)
   - ``left_enc_pos`` (Share): Left encoder position (ticks)
   - ``right_enc_pos`` (Share): Right encoder position (ticks)
   - ``left_enc_speed`` (Share): Left encoder velocity (ticks/us)
   - ``right_enc_speed`` (Share): Right encoder velocity (ticks/us)
   - ``obs_heading_share`` (Share): Observer heading output (degrees)
   - ``obs_yaw_rate_share`` (Share): Observer yaw rate output (deg/s)
   - ``left_set_point`` (Share): Left motor control effort
   - ``right_set_point`` (Share): Right motor control effort
   - ``observer_calibration_flg`` (Share): Calibration trigger flag (0 or 1)
   - ``big_X_share`` (Share): Observer X position output (mm)
   - ``big_Y_share`` (Share): Observer Y position output (mm)
   - ``initial_heading_share`` (Share): Initial heading reference (degrees)
   - ``end_flg`` (Share): End flag for debugging output

State Vector
""""""""""""

The observer maintains a 6-state vector:

.. math::

   \mathbf{x} = \begin{bmatrix} X \\ Y \\ \Theta \\ s \\ \Omega_L \\ \Omega_R \end{bmatrix}

where:

- **X**: Global X position (meters, converted to mm for output)
- **Y**: Global Y position (meters, converted to mm for output)
- **Theta**: Heading angle (radians, converted to degrees for output)
- **s**: Arc length traveled (meters)
- **Omega_L**: Left wheel velocity (m/s)
- **Omega_R**: Right wheel velocity (m/s)

Kinematic Model
"""""""""""""""

The observer uses differential drive kinematics with IMU heading feedback:

.. math::

   v_{center} = \frac{v_L + v_R}{2}

.. math::

   \omega_{body} = \frac{v_R - v_L}{w}

.. math::

   \dot{X} = v_{center} \cos(\theta_{IMU})

.. math::

   \dot{Y} = -v_{center} \sin(\theta_{IMU})

.. math::

   \dot{\theta} = \omega_{body}

.. math::

   \dot{s} = v_{center}

where:

- :math:`v_L, v_R`: Left and right wheel linear velocities (m/s)
- :math:`w`: Track width (0.141 m)
- :math:`\theta_{IMU}`: IMU heading measurement (radians)

Calibration Sequence
""""""""""""""""""""

When ``observer_calibration_flg == 1``, the observer performs initialization:

1. **Read Initial Heading**: Get IMU heading at startup
2. **Calculate Offset**: Compute correction to align IMU with robot frame (+X = 0°)
3. **Initialize Position**: Set starting position to (100, 800) mm
4. **Enable Observer**: Set calibration flag to 0 and begin estimation

Example::

   IMU heading at startup: -79.63°
   IMU heading offset (to correct to +X = 0°): 79.63° (1.3895 rad)
   [FIRST_RUN] Observer initialized at (100, 800) mm pointing +X (0°)

IMU Heading Filtering
""""""""""""""""""""""

Low-pass filter applied to reduce heading jitter:

.. code-block:: python

   heading_filtered = alpha * heading_new + (1 - alpha) * heading_prev
   # alpha = 0.3 (30% new, 70% previous)

This helps smooth out rapid heading oscillations from IMU noise.

RK4 Integration Parameters
"""""""""""""""""""""""""""

- **Integration Horizon**: 0 to 100ms
- **Step Size**: 10ms
- **Solver**: 4th-order Runge-Kutta (RK4_solver)
- **Update Rate**: 100ms (observer task period)

Tuning Parameters
"""""""""""""""""

.. py:data:: VELOCITY_SCALE
   :type: float
   :value: 2.0

   Velocity calibration factor to match physical measurements.

   - **Increase** if observer underestimates distance traveled
   - **Decrease** if observer overestimates distance traveled

.. py:data:: imu_alpha
   :type: float
   :value: 0.3

   IMU heading low-pass filter coefficient.

   - **Lower** (e.g., 0.1) = smoother but slower response
   - **Higher** (e.g., 0.5) = faster response but more noise

Hardware Configuration
""""""""""""""""""""""

- **IMU**: BNO055 9-DOF sensor for heading measurements
- **Encoders**: Quadrature encoders on both wheels (1437.1 ticks/rev)
- **Wheel Diameter**: 70mm (35mm radius)
- **Track Width**: 141mm
- **Wheel Circumference**: 0.2199 m

Coordinate Frame
""""""""""""""""

- **X Axis**: Forward direction of robot at initialization
- **Y Axis**: Right direction of robot at initialization
- **Theta**: Counter-clockwise positive from +X axis
- **Origin**: Initial position (100, 800) mm at startup

Example Console Output
"""""""""""""""""""""""

.. code-block:: text

   IMU heading at startup: -79.63°
   IMU heading offset (to correct to +X = 0°): 79.63° (1.3895 rad)
   [FIRST_RUN] Observer initialized at (100, 800) mm pointing +X (0°)

   X = 567.8 mm, Y = 723.1 mm, Heading = 45.62° (corrected), s = 523.4 mm | v=12.3cm/s

Observer Notes
""""""""""""""

- **Calibration Required**: Observer must be calibrated before navigation begins
- **Heading Source**: IMU heading is used directly for X, Y position calculation
- **Integration Source**: Encoder velocities drive heading integration via RK4
- **Output Format**: Position in mm, heading in degrees
- **Update Period**: 100ms observer task period ensures real-time performance

Angle Normalization Function
"""""""""""""""""""""""""""""

.. py:function:: normalize_angle_deg(angle_deg)

   Normalize angle to [-180, 180] degree range.

   :param angle_deg: Angle in degrees (can be any value)
   :type angle_deg: float
   :return: Normalized angle in degrees within [-180, 180]
   :rtype: float

   **Example**::

      >>> normalize_angle_deg(270.0)
      -90.0
      >>> normalize_angle_deg(-190.0)
      170.0

RK4 Solver
----------

Lightweight 4th-order Runge-Kutta (RK4) ODE solver for MicroPython + ulab. This module provides numerical integration for systems of ordinary differential equations, designed for embedded control and estimation tasks.

.. automodule:: defunct_rk4_solver
   :members:
   :undoc-members:
   :show-inheritance:

RK4 Solver Function
"""""""""""""""""""

.. py:function:: RK4_solver(fcn, x_0, tspan, tstep)

   Integrate a system of ODEs using 4th-order Runge-Kutta method.

   The ODE function must have the signature:

   .. code-block:: python

      x_dot, y = fcn(t, x)

   where:

   - ``x_dot`` is the state derivative (same shape as x)
   - ``y`` is the output vector at time t

   :param fcn: ODE function handle f(t, x) -> (x_dot, y)
   :type fcn: callable
   :param x_0: Initial state vector (column array, shape (n_states, 1))
   :type x_0: ulab.numpy.ndarray
   :param tspan: Time span [t0, tf] for integration
   :type tspan: list | tuple
   :param tstep: Integration step size
   :type tstep: float
   :return: (tout, yout) where tout is a 1D array of time samples and yout is a 2D array of outputs, shape (len(tout), n_outputs)
   :rtype: tuple(ulab.numpy.ndarray, ulab.numpy.ndarray)

RK4 Algorithm
"""""""""""""

The classic 4th-order Runge-Kutta method computes four intermediate slopes per step:

.. math::

   k_1 = f(t_n, x_n)

.. math::

   k_2 = f(t_n + \frac{h}{2}, x_n + \frac{h}{2} k_1)

.. math::

   k_3 = f(t_n + \frac{h}{2}, x_n + \frac{h}{2} k_2)

.. math::

   k_4 = f(t_n + h, x_n + h k_3)

.. math::

   x_{n+1} = x_n + \frac{h}{6}(k_1 + 2k_2 + 2k_3 + k_4)

where :math:`h` is the step size (``tstep``).

Usage Example
"""""""""""""

.. code-block:: python

   from ulab import numpy as np
   from defunct_rk4_solver import RK4_solver

   # Define system: x_dot = -0.5 * x (exponential decay)
   def system_fcn(t, x):
       x_dot = np.array([[-0.5 * x[0, 0]]])
       y = x  # Output = state
       return x_dot, y

   # Initial condition: x(0) = 1.0
   x_0 = np.array([[1.0]])

   # Integrate from t=0 to t=10 with step size 0.1
   tout, yout = RK4_solver(system_fcn, x_0, [0, 10], 0.1)

   # yout contains the solution at each time step
   print(f"Final state: {yout[-1, 0]:.4f}")  # Should be ≈ 0.0067

Observer Integration Example
"""""""""""""""""""""""""""""

The observer task uses RK4_solver to integrate robot kinematics:

.. code-block:: python

   # Wrapper function for RK4 solver
   def kinematics_wrapper(t, x_col):
       '''Wrapper to convert column vector to/from kinematics_fun format'''
       # Use corrected heading for position integration
       xd, y = kinematics_fun(t, x_col, omega_L_ms, omega_R_ms, heading_corrected_rad)
       return xd, y

   # Run RK4 integration for 100ms with 10ms steps
   tout, yout = RK4_solver(kinematics_wrapper, x, [0, 0.1], 0.01)

   # Extract final state from the last row
   x = np.array([[yout[-1, i]] for i in range(6)])

RK4 Performance Notes
"""""""""""""""""""""

- **Memory Efficient**: Designed for ulab's subset of NumPy (MicroPython)
- **Fixed Step Size**: Uses constant time step (no adaptive step sizing)
- **Accuracy**: 4th-order method, local truncation error :math:`O(h^5)`
- **Speed**: Optimized for embedded systems with limited resources
- **Typical Usage**: 10ms steps over 100ms horizon for observer task

Main (Original)
---------------

Main program for the original IMU-based obstacle course navigation robot. This implementation uses BNO055 IMU for heading estimation combined with encoder-based state observation, implementing cooperative multitasking with IMU calibration, observer-based state estimation, and navigation control.

.. automodule:: defunct_main
   :members:
   :undoc-members:
   :show-inheritance:

System Architecture
"""""""""""""""""""

The main program coordinates seven cooperative tasks using the cotask scheduler:

1. **IMU Calibration Task** (Priority 7, One-time)
   - Runs at startup to initialize and calibrate BNO055 IMU
   - Loads calibration from file if available
   - Performs manual calibration if needed
   - Period: 50ms (runs until calibration complete)

2. **IMU Monitor Task** (Priority 3, Period 70ms)
   - Continuously reads heading and yaw rate from IMU
   - Implements heading unwrapping for continuous values
   - Provides sensor data to observer and navigation tasks

3. **Observer Task** (Priority 5, Period 100ms)
   - Fuses IMU heading with encoder velocity data
   - Uses RK4 numerical integration for state estimation
   - Outputs global X, Y position and heading estimates

4. **Velocity Control Task** (Priority 2, Period 12ms)
   - Updates motor PWM based on control effort
   - Fastest task for responsive motor control
   - Applies setpoints from closed-loop controllers

5. **Closed Loop Control Task** (Priority 4, Period 20ms)
   - PI velocity control for both motors
   - Implements line following with IR sensor array
   - Coordinates heading control and bias adjustments

6. **Navigation Task** (Priority 6, Period 100ms)
   - 5-state finite state machine for course navigation
   - Position-based state transitions using observer estimates
   - Coordinates line following, turns, and straight segments

7. **Serial Communication Task** (Priority 1, Period 200ms)
   - Handles terminal commands and status output
   - Lowest priority for non-critical communication

Hardware Configuration
""""""""""""""""""""""

**Motors and Encoders:**

.. code-block:: python

   # Left motor on pins PC1 (direction), PA0 (PWM), timer 5 channel 1
   left_motor = motor_driver(Pin.cpu.C1, Pin.cpu.A0, Timer(5, freq=20_000), 1)

   # Right motor on pins PA10 (direction), PC7 (PWM), timer 3 channel 2
   right_motor = motor_driver(Pin.cpu.A10, Pin.cpu.C7, Timer(3, freq=20_000), 2)

   # Left encoder on pins PB6, PB7 (timer 4)
   left_encoder = Encoder(Pin.cpu.B6, Pin.cpu.B7, 4)

   # Right encoder on pins PC6, PC7 (timer 8)
   right_encoder = Encoder(Pin.cpu.C6, Pin.cpu.C7, 8)

**IR Sensor Array:**

.. code-block:: python

   # 7 IR reflectance sensors for line detection
   ir_pins = [Pin.cpu.A4, Pin.cpu.A5, Pin.cpu.A6, Pin.cpu.A7,
              Pin.cpu.B0, Pin.cpu.B1, Pin.cpu.C4]
   ir_sensors = [pyb.ADC(pin) for pin in ir_pins]

**IMU Sensor:**

.. code-block:: python

   # BNO055 9-DOF IMU on software I2C
   # SCL: Pin B13, SDA: Pin B14, Reset: Pin B15
   # I2C Address: 0x28 (default)

**Robot Physical Parameters:**

- **Wheel Diameter**: 70mm (radius = 35mm)
- **Track Width**: 141mm
- **Encoder Resolution**: 1437.1 ticks/revolution
- **Wheel Circumference**: 0.2199 m

Task Timing and Priorities
"""""""""""""""""""""""""""

The task scheduler executes tasks based on priority (higher number = higher priority) and period:

.. list-table::
   :header-rows: 1
   :widths: 30 15 15 40

   * - Task Name
     - Priority
     - Period (ms)
     - Function
   * - IMU Calibration
     - 7
     - 50
     - One-time startup calibration
   * - Navigation
     - 6
     - 100
     - Position-based state machine
   * - Observer
     - 5
     - 100
     - State estimation with RK4
   * - Closed Loop Control
     - 4
     - 20
     - PI velocity control
   * - IMU Monitor
     - 3
     - 70
     - Heading/yaw rate reading
   * - Velocity Control
     - 2
     - 12
     - Motor PWM update
   * - Serial Communication
     - 1
     - 200
     - Terminal I/O

Shared Variables
""""""""""""""""

The system uses 28 shared variables for inter-task communication:

**Sensor Measurements:**

- ``meas_heading_share``: IMU heading (degrees, continuous)
- ``meas_yaw_rate_share``: IMU yaw rate (deg/s)
- ``left_enc_pos``: Left encoder position (ticks)
- ``right_enc_pos``: Right encoder position (ticks)
- ``left_enc_speed``: Left encoder velocity (ticks/us)
- ``right_enc_speed``: Right encoder velocity (ticks/us)

**Observer Outputs:**

- ``big_X_share``: Global X position estimate (mm)
- ``big_Y_share``: Global Y position estimate (mm)
- ``obs_heading_share``: Observer heading estimate (degrees)
- ``obs_yaw_rate_share``: Observer yaw rate estimate (deg/s)
- ``initial_heading_share``: Initial heading reference (degrees)

**Motor Control:**

- ``left_set_point``: Left motor control effort
- ``right_set_point``: Right motor control effort
- ``left_desired_vel``: Left motor velocity setpoint
- ``right_desired_vel``: Right motor velocity setpoint
- ``left_controller_share``: Left motor PI controller object
- ``right_controller_share``: Right motor PI controller object

**Navigation Flags:**

- ``line_follow_flg``: Enable line following (0/1)
- ``force_straight_flg``: Force straight mode (0/1)
- ``nav_turn_flg``: Enable heading-based turning (0/1)
- ``nav_stop_flg``: Stop navigation (0/1)
- ``nav_rest_flg``: Navigation rest flag
- ``bias_share``: Line following bias (-3 to +3)
- ``bias_timer_flg``: Bias timer flag

**System Control:**

- ``observer_calibration_flg``: Observer calibration trigger (0/1)
- ``calibration_flg``: System calibration status (0-3)
- ``end_flg``: End flag for debugging output
- ``imu_share``: IMU object share (BNO055 instance)

Startup Sequence
""""""""""""""""

The main program follows this initialization sequence:

1. **Hardware Initialization**:
   - Configure motor drivers with PWM timers
   - Initialize encoders with position and velocity tracking
   - Set up IR sensor array for line detection
   - Configure IMU reset pin (B15)

2. **Shared Variable Creation**:
   - Create all 28 shared variables for inter-task communication
   - Initialize controller objects (PI for velocity control)

3. **Task Creation**:
   - Create 7 cooperative tasks with cotask.Task()
   - Assign priorities and periods for each task
   - Add all tasks to cotask.task_list

4. **Scheduler Execution**:
   - Run cotask.task_list.pri_sched() in infinite loop
   - Tasks yield control based on period and priority
   - Handle KeyboardInterrupt for graceful shutdown

Control Flow Example
""""""""""""""""""""

A typical control cycle showing task coordination:

.. code-block:: text

   Time 0ms:
   ├─ Velocity Control (12ms task)
   │  └─ Apply motor PWM based on left/right_set_point

   Time 20ms:
   ├─ Closed Loop Control (20ms task)
   │  ├─ Read IR sensors for line position
   │  ├─ Calculate line following error with bias
   │  ├─ Run PI velocity controllers
   │  └─ Update left/right_set_point

   Time 70ms:
   ├─ IMU Monitor (70ms task)
   │  ├─ Read heading from BNO055
   │  ├─ Apply heading unwrapping
   │  └─ Update meas_heading_share

   Time 100ms:
   ├─ Observer (100ms task)
   │  ├─ Read IMU heading and encoder velocities
   │  ├─ Run RK4 integration (10ms steps over 100ms)
   │  └─ Update big_X_share, big_Y_share, obs_heading_share
   │
   ├─ Navigation (100ms task)
   │  ├─ Read X, Y position from observer
   │  ├─ Check position thresholds for state transitions
   │  ├─ Update line_follow_flg, bias_share based on state
   │  └─ Set desired_angle_share for heading control

Terminal Commands
"""""""""""""""""

**Connecting to Board:**

.. code-block:: bash

   # List available USB serial devices (macOS/Linux)
   ls /dev/tty.*

   # Connect using screen (115200 baud)
   screen /dev/tty.usbmodem205F379C39472 115200

   # Exit screen session
   # Press: Ctrl-A, then K, then Y

**Board Commands:**

.. code-block:: python

   # Soft reset the board
   # Press: Ctrl-D

   # Hard interrupt (KeyboardInterrupt)
   # Press: Ctrl-C

Memory Management
"""""""""""""""""

The program includes memory management for MicroPython:

.. code-block:: python

   import gc
   gc.collect()  # Run garbage collection periodically

This helps prevent memory fragmentation during long-running operation.

Differences from Final Implementation
""""""""""""""""""""""""""""""""""""""

This original approach was replaced by the displacement-based odometry system. Key differences:

**Original (defunct) Approach:**

- Uses BNO055 IMU for absolute heading measurement
- Observer fuses IMU heading with encoder data via RK4 integration
- Global X, Y position estimation from sensor fusion
- Position-based navigation with coordinate thresholds
- More complex state estimation with 6-state observer

**Final Approach:**

- Pure encoder-based odometry (no IMU)
- Displacement calculation from encoder ticks
- Simpler state estimation without sensor fusion
- Distance-based navigation (no global coordinates)
- Reduced complexity and improved reliability

See :doc:`analysis` for the technical rationale behind this architectural change, including IMU communication issues that motivated the switch to encoder-only estimation.
