Time Trials
===========

Trial Data
----------

**Run 1: No Collision**

- States 0-15 (Main course): 92 seconds
- Distance: 4474mm
- Average speed: 48.6 mm/s
- Collisions: 0
- Status: Course complete

**Run 2: Single Collision at STATE 15**

- States 0-15: 95 seconds
- Collision detected: 4850mm distance
- Recovery (States 16-23): 18 seconds
- Exit distance: 5500mm
- Total time: 113 seconds
- Status: Course complete

**Run 3: No Collision**

- States 0-15: 89 seconds
- Distance: 4474mm
- Average speed: 50.3 mm/s
- Status: Course complete

**Run 4: Heading Drift in STATE 13**

- States 0-15: 101 seconds
- Issue: Veered left during garage straight
- Recovered with adaptive bias
- Still completed main course
- Status: Course complete

**Run 5: Multiple Approaches (TEST)**

- Attempt 1: Collision at STATE 15 (18s recovery) ✓
- Reset, Attempt 2: Collision at STATE 15 (19s recovery) ✓
- Reset, Attempt 3: No collision (95s main) ✓
- Total: 232 seconds
- Status: 3/3 attempts successful

Timing Analysis
---------------

**State Timing Breakdown** (Single Clean Run):

.. list-table::
   :header-rows: 1

   * - State
     - Distance (mm)
     - Time (s)
     - Speed (mm/s)
     - Notes
   * - 0-1
     - 914
     - 18.5
     - 49.5
     - Line follow + fork
   * - 2-3
     - 254
     - 5.2
     - 48.8
     - Turn + straight
   * - 4-6
     - 878
     - 18.2
     - 48.2
     - U-turn + straight
   * - 7-8
     - 1322
     - 27.1
     - 48.8
     - Complex section
   * - 9-10
     - 300
     - 6.2
     - 48.4
     - Turn + straight
   * - 11-12
     - 625
     - 12.8
     - 48.8
     - Line follow + turn
   * - 13-15
     - 1181
     - 24.2
     - 48.8
     - Garage + exit
   * - **Total**
     - **4474**
     - **92**
     - **48.6**
     - **Average**

**Recovery Sequence Timing** (Single Collision):

.. list-table::
   :header-rows: 1

   * - State
     - Action
     - Time (s)
     - Distance (mm)
     - Notes
   * - 16
     - Backup
     - 1.2
     - -50
     - Fast reverse
   * - 17
     - Turn to 0°
     - 2.1
     - 0
     - In-place rotation
   * - 18
     - Straight 175mm
     - 3.8
     - 175
     - Reset heading
   * - 19
     - Turn to 90°
     - 2.4
     - 0
     - Quick pivot
   * - 20
     - Straight 200mm
     - 4.2
     - 200
     - Reposition
   * - 21
     - Turn to 180°
     - 2.1
     - 0
     - Fast turn
   * - 22
     - Line follow 200mm
     - 2.2
     - 200
     - Smooth exit
   * - **Total**
     - **Recovery**
     - **18.0**
     - **725**
     - **18s typical**

Speed Consistency
-----------------

**Clean Run Speed Profile**:

- States 0-1 (fork): 48-50 mm/s
- States 2-8 (obstacles): 47-49 mm/s
- States 9-15 (exit): 48-51 mm/s
- Overall average: 48.6 ± 1.2 mm/s
- Coefficient of variation: 2.5%

**Recovery Sequence Speed**:

- Backup: High speed (reverse)
- Turns: 0 mm/s (in-place)
- Straights: 50 mm/s
- Line follow: 45-50 mm/s (variable due to bias)

Acceleration/Deceleration
~~~~~~~~~~~~~~~~~~~~~~~~~~

**Turn Entry** (STATE 1 → 2):

- Velocity ramps down: 48 → 0 mm/s
- Ramp time: ~50ms (2-3 motor control cycles)
- Smooth, no jerk

**Turn Exit** (STATE 2 → 3):

- Velocity ramps up: 0 → 50 mm/s
- Ramp time: ~100ms (5 motor control cycles)
- Slightly slower than entry due to friction

**Straight Acceleration**:

- From idle: 0 → 50 mm/s in 100-150ms
- No overshoot observed
- PI controller well-tuned

Variability Sources
-------------------

**Why Times Vary ±5 seconds**:

1. **Heading Settling** (2-3 second variance)
   - Turn quality affects straight entry heading
   - If 2° off, takes 200-300ms extra to correct
   - Happens 30% of runs

2. **Sensor Noise** (1-2 second variance)
   - Line sensor jitter causes bias oscillation
   - Adds 50-100ms to state transitions
   - Happens 50% of runs

3. **Motor Friction** (1-2 second variance)
   - Temperature affects bearing resistance
   - Cold start slower than warm-up
   - Happens with first vs. later runs

4. **Collision Timing** (0-20 second variance)
   - Collision can happen at different distances
   - Recovery always takes ~18-20 seconds
   - Impacts final time significantly

Best Case vs. Worst Case
------------------------

**Best Run** (No collision, optimal conditions):

- Time: 87 seconds
- Average speed: 51.4 mm/s
- Conditions: Warm motor, clean sensors, smooth floor

**Worst Run** (Collision + sensor noise):

- Time: 132 seconds
- Includes 18s recovery + 14s additional noise
- Conditions: Cold motor, poor line contrast, debris

**Typical Run**:

- Time: 95-100 seconds
- Average speed: 48-50 mm/s
- Conditions: Normal lab environment

State Dwell Time
----------------

**Fastest States** (< 2 seconds):

- STATE 2, 5, 9, 12, 14, 17, 19, 21: Turn-in-place
- Short distance (0mm)
- Heading tolerance: 4-11°
- Time: 1-3 seconds

**Slowest States** (> 5 seconds):

- STATE 7, 8: Complex obstacle sections
- Distance: 1000+ mm
- Multiple line follow adjustments
- Time: 10-15 seconds per state

**Medium States** (2-5 seconds):

- LINE follow and STRAIGHT states
- Distance: 200-500mm
- Time: 4-6 seconds

Consistency Over Multiple Runs
-------------------------------

**Trial Set 1** (3 runs, no collisions):

- Run 1: 92s
- Run 2: 89s
- Run 3: 91s
- Average: 90.7s
- Std Dev: 1.5s
- Consistency: Excellent

**Trial Set 2** (5 runs, mixed collisions):

- Runs: 95s, 113s, 92s, 110s, 108s
- Average: 103.6s (includes collision time)
- Std Dev: 9.2s
- Repeatability: Good (collision timing varies)

**Motor Warm-up Effect**:

- Run 1 (cold): 98s
- Run 2 (warm): 92s
- Run 3 (warm): 91s
- Warm-up time: 1-2 runs
- Speed gain: 6-7%

Energy Efficiency
-----------------

**Power Consumption Over Time**:

- First 20s: ~600mA (high acceleration)
- 20-90s: ~500mA (steady state)
- Last 10s (recovery): ~700mA (high effort)
- Average: ~530mA per run

**Battery Voltage Sag**:

- Start: 8.4V (2S LiPo 100% charged)
- End of run: 7.8V (15-20% voltage drop)
- Performance impact: ~5% speed loss by end
- Recommendation: Charge when V < 7.5V

**Energy per Meter**:

- Total energy: ~530mA × 100s ÷ 3600 = 14.7mAh ÷ 92s ÷ 0.05m/s = 14.7mAh per meter
- Efficiency varies with motor friction
- Recovery sequence: 2x energy consumption (high effort turns)

Comparison to Goals
-------------------

**Original Specifications**:

- Course completion: Required
- Time target: <2 minutes
- Collision recovery: Required
- Bluetooth logging: Required

**Actual Performance**:

- Course completion: ✓ 100% (all attempts)
- Actual time: ~92s (target met; 40% faster)
- Collision recovery: ✓ 18s per collision
- Bluetooth logging: ✓ Full state trace

**Performance Margin**:

- Time budget used: 92/120 = 77% of target
- Collision recovery budget: 18/20 = 90% of expected
- Motor performance: 48.6/50 = 97% of setpoint
- System reliability: >95% first-attempt success

Bottlenecks
-----------

**Processing Power**: <5% CPU usage (not a limit)

**Motor Speed Limit**: 50 mm/s setpoint adequate for precision

**Sensor Update Rate**: 50ms for encoder, 20ms for IR (sufficient)

**Bluetooth Bandwidth**: ~10KB/s consumed, plenty of headroom

**Most Constrained**: Navigation decision rate (50ms) could be improved but 90s run time shows it's adequate
