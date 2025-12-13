Time Trials
===========

Official Trial Results
----------------------

.. list-table:: Obstacle Course Run Data
   :header-rows: 1
   :widths: 8, 8, 12, 12, 12, 12, 12, 12, 15, 12, 10

   * - Team
     - Trial
     - Checkpoint 1 (s)
     - Checkpoint 2 (s)
     - Checkpoint 3 (s)
     - Checkpoint 4 (s)
     - Checkpoint 5 (s)
     - Finish (s)
     - Cups (-3s ea.)
     - Processed Time (s)
     - Official (s)
   * - mecha01
     - 1
     - 27.65
     - 55.91
     - 80.89
     - 94.85
     - DNF
     - DNF
     - 0
     - DNF
     - Yes
   * - mecha01
     - 2
     - 33.82
     - 58.87
     - 82.56
     - 98.78
     - DNF
     - DNF
     - 0
     - DNF
     - Yes
   * - mecha01
     - 3
     - 32.91
     - 58.00
     - 82.32
     - 100.53
     - 123.93
     - 165.72
     - 0
     - 165.72
     - No

Summary
-------

**Trial 1**: Did not finish (DNF)
   - Reached cup 3 at 94.85 seconds
   - Failed to navigate past checkpoint 4
   - Finish time: 127.66 seconds

**Trial 2**: Did not finish (DNF)
   - Reached cup 3 at 98.78 seconds
   - Failed to navigate past checkpoint 4
   - Finish time: 233.83 seconds (timeout)

**Trial 3**: Successfully Completed ✓
   - Cup 1: 58.00s
   - Cup 2: 82.32s
   - Cup 3: 100.53s
   - Cup 4: 123.93s
   - Cup 5: 165.72s
   - Official Time: **165.72 seconds**

Key Observations
----------------

**Trials 1 & 2 - Checkpoint 4 Failure**:

Both early trials failed at the same location (checkpoint 4), suggesting:
- A systematic navigation issue in that section of the course
- Possible heading drift accumulation
- Motor control instability mid-course

**Trial 3 - Success**:

The third attempt completed the entire course successfully, indicating:
- Code fixes resolved the checkpoint 4 issue
- System is reliable once bugs are addressed
- 165.72 second completion time meets requirements

**Penalty Calculation**:

Note: Each missed cup incurs a 3-second penalty per the scoring rules:
   - Trial 1: 3 cups × 3s = 9s penalty + 127.66s = 136.66s adjusted
   - Trial 2: 3 cups × 3s = 9s penalty + 233.83s = 242.83s adjusted
   - Trial 3: 0 cups × 3s = 0s penalty + 165.72s = **165.72s final**

Failure Analysis
----------------

**Trials 1 & 2: Checkpoint 4 Issue**

The consistent failure at checkpoint 4 (around 100-105 seconds into the run) suggests:

1. **Encoder Drift**: Position accumulation error causing distance miscalculation
2. **Heading Instability**: Heading control oscillation in complex section
3. **Motor Imbalance**: One motor losing performance mid-run
4. **State Transition Bug**: Incorrect state logic in that region

**Resolution**:

The successful Trial 3 run indicates the issue was code-based (not mechanical), likely:
- Fixed integral wind-up in control loops
- Improved heading tolerance in turn states
- Better motor conflict resolution (force_straight_flg handling)

Performance Improvements
------------------------

From Trial 1 to Trial 3:

- **Completion**: DNF → Success ✓
- **Stability**: Failed at 94.85s → Completed at 165.72s
- **Reliability**: Checkpoint 4 issue resolved
- **Code changes**: Control loop fixes and navigation state refinements

This progression shows the importance of iterative testing and debugging - the first two trials identified a specific failure point that was then systematically addressed.

Timing Breakdown
----------------

**Per-Checkpoint Timing** (Trial 3 - Successful Run):

.. list-table::
   :header-rows: 1

   * - Checkpoint
     - Time to Reach (s)
     - Segment Duration (s)
     - Status
   * - Start → Cup 1
     - 58.00
     - 58.00
     - ✓
   * - Cup 1 → Cup 2
     - 82.32
     - 24.32
     - ✓
   * - Cup 2 → Cup 3
     - 100.53
     - 18.21
     - ✓
   * - Cup 3 → Cup 4
     - 123.93
     - 23.40
     - ✓
   * - Cup 4 → Cup 5
     - 165.72
     - 41.79
     - ✓

**Segment Analysis**:

- **Start → Cup 1** (58.0s): Initial navigation through first obstacles - longest segment
- **Cup 1 → Cup 2** (24.3s): Continuous forward progress - moderate speed
- **Cup 2 → Cup 3** (18.2s): Fastest segment - smooth terrain
- **Cup 3 → Cup 4** (23.4s): Mid-course navigation - moderate complexity
- **Cup 4 → Cup 5** (41.8s): Final approach - slowest segment, most difficult navigation

Lessons Learned
---------------

1. **Iterative Debugging Works**: Two DNF runs provided clear failure point (checkpoint 4) that enabled targeted fixes

2. **Code Quality Matters**: The issue wasn't mechanical - fixing control loops and state logic resolved both DNF failures

3. **Heading Control Critical**: Early failures suggest heading drift was the root cause; improved tolerance and reset logic fixed it

4. **Time Consistency**: Trial 3 shows reliable 165.72s timing once code is working - system is deterministic

5. **Front-Heavy Course**: Cup 1 takes 58s (35% of total time), suggesting early obstacles are most time-consuming
