Alternative Approaches
======================

Abandoned Approaches
--------------------

IMU-Based Heading
~~~~~~~~~~~~~~~~~

**Tried**: Using accelerometer + gyroscope for heading calculation

**Why Abandoned**:

- Gyro drift accumulates over long runs (5-10° error by end)
- Accelerometer affected by vibration and acceleration
- Calibration required for each course variant
- Higher computational cost

**Replaced By**: Encoder-based odometry (subtracting encoder deltas)

Result: Heading stays within ±5° consistently, no drift

Dynamic Bias Updates During Straights
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Tried**: Adjusting line_follow bias every cycle based on heading error

**Problem**:

- Conflicted with force_straight_flg
- Created asymmetric motor commands
- One motor got correction, one fought it
- Severe heading drift (10-15°) in tight sections

**Solution**: Static bias = 0.0 during force_straight_flg=1

- Motor PID proportional gain handles fine corrections
- No conflicting signals

Fixed Heading Targets After Turns
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Tried**: STATE 3 targets HEADING_NEG_90, STATE 6 targets HEADING_90

**Problem**:

- Motors were still adjusting heading when STATE entered
- Additional 50-100ms delay between turn and straight
- Overshoot and oscillation

**Solution**: Instant heading capture (use current heading as target)

- On entry to STATE 3/6, capture enc_heading_share
- Use that as desired_angle_share
- Eliminates delay, smooth transition

Fixed Recovery Distances
~~~~~~~~~~~~~~~~~~~~~~~~

**Tried**: STATE 16 backup = 50mm, STATE 18 = 500mm, STATE 20 = 400mm

**Problem**:

- Robot would often get stuck during second turn
- Didn't leave enough room for final exit

**Solution**: Optimized distances

- Backup: 50mm (unchanged)
- STATE 18: 175mm (reduced)
- STATE 20: 200mm (reduced)
- STATE 22: 200mm line follow (allows clean exit)

File Logging
~~~~~~~~~~~~

**Tried**: Open file at task start, write every cycle, close at end

**Problem**:

- File handle stays open, blocks REPL
- MicroPython doesn't flush automatically
- Hundreds of writes killed SD card responsiveness

**Solution**: Queue-based streaming to Bluetooth

- Navigation writes to queue
- Serial task drains queue and sends via BT
- No file I/O, no blocking
- Terminal output captures everything

Alternative Technologies Considered
------------------------------------

Kalman Filter for State Estimation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Reason Rejected**: 

- Too computationally expensive for MicroPython
- Encoder odometry already sufficient for short runs
- Added complexity not justified by accuracy gains

Image Processing for Line Detection
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Reason Rejected**:

- NUCLEO doesn't have camera interface
- IR sensors adequate for line following
- Vision processing would require external module

Gyro Heading with Encoder Correction
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Reason Rejected**:

- Hybrid approaches more error-prone than pure encoder
- No advantage over pure encoder odometry for closed loops
- Additional sensor noise

Machine Learning for Obstacle Prediction
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Reason Rejected**:

- Obstacle positions fixed and known
- State machine deterministic and sufficient
- ML overkill for predefined course

Edge Cases Not Pursued
~~~~~~~~~~~~~~~~~~~~~~

**Soft Start for Motors**: Would reduce current spike but adds complexity
  - Quick PWM ramp sufficient for this application

**Adaptive PID Gains**: Could tune gains per state
  - Fixed gains work well enough; adds maintenance burden

**Graceful Degradation on Sensor Failure**: Would require redundancy
  - Single sensor failure ends run anyway (course design)

**Battery Voltage Compensation**: Would account for power drain
  - 50mm/s speed low enough that voltage variance minimal

Proven Solutions
----------------

This implementation uses these proven approaches:

- **Encoder odometry** for heading (reliable, no drift)
- **Static force_straight_flg** (prevents motor conflicts)
- **Queue-based logging** (non-blocking, reliable)
- **PI velocity control** (standard motor regulation)
- **State machine navigation** (predictable, easy to debug)
- **Instant heading capture** (smooth transitions)

All of these were chosen through testing and iteration on the actual course.
