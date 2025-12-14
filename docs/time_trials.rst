Time Trials
==========

The time trials were a live demonstration of Romi’s full system autonomy under official scoring rules, where performance depended on completing the obstacle course reliably across multiple attempts rather than producing a single ideal run. The evaluation emphasized repeatable navigation through constrained track features and decision points, with timed checkpoint progression and optional time improvement opportunities used only when they did not compromise full course completion.

Official Trial Results
----------------------

The table below reports our official checkpoint timestamps recorded during the live runs, where each checkpoint time represents elapsed time from the start to that checkpoint. DNF indicates a run that terminated before reaching the remaining checkpoints, and these results are used to identify repeatability limits and isolate the course segment that most strongly constrained performance.


.. list-table:: Obstacle Course Run Data
   :header-rows: 1
   :widths: 8, 12, 12, 12, 12, 12, 12, 15, 12, 10

   * - Trial
     - Checkpoint 1 (s)
     - Checkpoint 2 (s)
     - Checkpoint 3 (s)
     - Checkpoint 4 (s)
     - Checkpoint 5 (s)
     - Finish (s)
     - Cups (-3s ea.)
     - Processed Time (s)
     - Official (s)
   * - 1
     - 27.65
     - 55.91
     - 80.89
     - 94.85
     - DNF
     - DNF
     - 0
     - DNF
     - Yes
   * - 2
     - 33.82
     - 58.87
     - 82.56
     - 98.78
     - DNF
     - DNF
     - 0
     - DNF
     - Yes
   * - 3
     - 32.91
     - 58.00
     - 82.32
     - 100.53
     - 123.93
     - 165.72
     - 0
     - 165.72
     - No

.. raw:: html

   <div style="text-align:center; margin-top: 10px; margin-bottom: 10px;">
     <video controls style="max-width: 900px; width: 100%; height: auto;">
       <source src="_static/successful_run.mp4" type="video/mp4">
       Your browser does not support the video tag.
     </video>
   </div>

.. centered::
   *Successful full course run recorded during the live time trials demonstration.*

Summary
-------

**Trial 1**: Did not finish (DNF)
   - Reached checkpoint 3 at 94.85 seconds
   - Failed entering parking garage (checkpoint 4)
   - Heading control lost in narrow corridor
   - Finish time: DNF

**Trial 2**: Did not finish (DNF)
   - Reached checkpoint 3 at 98.78 seconds
   - Failed entering parking garage (checkpoint 4)
   - Same heading instability as Trial 1
   - Finish time: DNF

**Trial 3**: Successfully Completed ✓
   - Checkpoint 1: 32.91s
   - Checkpoint 2: 58.00s
   - Checkpoint 3: 82.32s
   - Checkpoint 4: 100.53s
   - Checkpoint 5: 123.93s
   - Checkpoint 6: 165.72s

Key Observations
----------------

**Trials 1 & 2 - Checkpoint 4 Failure**:

Both early trials failed at checkpoint 4 (garage entry):
- Heading control unstable in corridor
- Robot drifted into wall or overcorrected into opposite wall
- Never reached checkpoint 5
- Consistent failure point indicates systematic control issue at this location

**Trial 3 - Same Code, Different Outcome**:

Trial 3 **also uses the same code** but managed to:
- Pass checkpoint 4 (barely)
- Reach checkpoint 5 (garage exit)
- Complete checkpoint 6 (wall interaction)
- **But this is NOT due to code fixes - the problem persists**

**Why Trial 3 Succeeded When 1 & 2 Failed**:

With identical code, the difference must be:
- **Luck/Timing**: Random initial conditions (heading, motor state) happened to work
- **Environmental Factors**: Different floor friction, temperature, or sensor calibration
- **Margin of Safety**: The code barely works - Trials 1 & 2 hit the wall, Trial 3 threaded the needle
- **Non-Deterministic Behavior**: The same code sometimes works, sometimes fails

**Critical Issue**:

The checkpoint 4 problem is **NOT FIXED**. The code is **unreliable** because:
- Same code fails 2/3 times (Trials 1 & 2)
- Same code passes 1/3 times (Trial 3)
- Success appears to depend on chance, not code quality
- Future runs will likely fail again at checkpoint 4

**Cups (Bonuses)**:

No cups were collected in any trial:
- Cups provide -3 second bonus each if collected
- Trial 3 could have scored lower with cup collection
- Focus was on completing course safely, not optimization

Failure Analysis
----------------

**The Garage Sequence: The Unresolved Problem**

Trials 1 & 2 failed between checkpoint 4 and 5 (garage). Trial 3 used the **same code** but didn't fail - this means:

1. **The Problem Wasn't Fixed**: No code changes were made between trials
2. **The Code is Unreliable**: It fails sometimes, passes sometimes, with identical logic
3. **Chance, Not Quality**: Trial 3 likely succeeded by luck - slightly different initial conditions or timing
4. **Will Fail Again**: Future runs using this code will probably fail at checkpoint 4 again

**What's Actually Happening at Checkpoint 4**:

The garage entry (CP 4) requires precise heading control in a corridor. The code's heading control strategy:
- Either drifts too much (hits wall)
- Or corrects too aggressively (oscillates and hits opposite wall)
- Or sometimes finds the balance purely by chance

**Why Trial 3 Succeeded**:

Possible explanations (all non-code-related):
- Robot started with slightly better heading alignment
- Motor friction was different that attempt
- Floor conditions were different
- Timing of heading corrections happened to work out
- Pure chance - the robot threaded the needle

**Root Cause**: The heading control algorithm fundamentally can't reliably navigate the tight garage corridor. The code needs redesign, not just tuning.

Performance Problems
-------------------

From Trial 1 to Trial 3:

- **Trial 1**: Failed after CP 4 (heading control lost)
- **Trial 2**: Failed after CP 4 (same heading control issue)
- **Trial 3**: Passed through the garage, completed entire course

**No Actual Improvements**:

- No code changes between trials
- Same heading control algorithm
- Same motor balance issues
- Trial 3 success is luck, not improvement

**The Real Issue**:

The checkpoint 4 garage problem is not solved. The code needs:
- Fundamentally different heading control strategy
- Better motor symmetry handling
- Wider tolerance or different approach to narrow corridors
- Redesign, not tuning

Timing Breakdown
----------------

**Segment Timing** (Trial 3 - Successful Run):

.. list-table::
   :header-rows: 1

   * - Segment
     - Time to Reach (s)
     - Segment Duration (s)
     - Notes
   * - Start → CP 1
     - 32.91
     - 32.91
     - Straight line, right turn fork (Radius = 200mm),
   * - CP 1 → CP 2
     - 58.00
     - 25.09
     - Open navigation
   * - CP 2 → CP 3
     - 82.32
     - 24.32
     - Mid-course
   * - CP 3 → CP 4 (Garage Entry)
     - 100.53
     - 18.21
     - **Tight corridor - heading critical**
   * - CP 4 → CP 5 (Garage Interior)
     - 123.93
     - 23.40
     - **Parking garage main segment**
   * - CP 5 → CP 6 (Wall Interaction)
     - 165.72
     - 41.79
     - **Final wall approach and recovery**

**Garage Segment Analysis** (CP 4 → CP 5):

The parking garage (18.21s + 23.40s = 41.61s total) accounts for 41.61/165.72 = 25% of total time:
- Most technically difficult section
- Narrow corridor with pillars on both sides
- Requires constant heading maintenance at 180°
- Motor balance critical - any asymmetry causes wall contact
- **Trials 1 & 2 never made it past CP 4 entry**
- Trial 3 passed cleanly through entire garage

**Wall Interaction Segment** (CP 5 → CP 6):

The final approach (41.79s) is the longest segment:
- Robot approaches wall and must interact (bump/stop)
- Requires proper heading control to align with wall
- Possible recovery sequence if bump detected
- Trial 3 completed this segment successfully after solving garage issues

Lessons Learned
---------------

The trial runs highlighted critical issues with code reliability and project management. The same code failed 2/3 times but passed 1/3 times, revealing fundamental non-deterministic behavior rather than robust control design. This inconsistency points to the need for better version management practices - we should have used version control (git) more rigorously to track exactly what code changes were made between trials and to maintain stable, tested releases. Additionally, our code and share organization became chaotic during rapid iteration, making it difficult to identify which modules were causing failures. The scheduler implementation, in particular, suffered from poorly documented task timing relationships and unclear dependencies between cooperative tasks, making debugging timing-sensitive failures extremely difficult. For future projects, establishing clear file organization, documented interfaces between tasks/shares, well-commented scheduler configuration with explicit timing constraints, and a disciplined commit workflow would help isolate problems faster and ensure reproducible results across multiple runs.

Recommendations for Future Runs
--------------------------------

- **Tighten Pre-Garage Setup**: Ensure motors are perfectly synchronized before CP 3→4 transition
- **Collect Cups**: With garage navigation proven, future runs could collect cups for -3s bonuses each
- **Monitor Motor Health**: Check bearing friction - impacts garage performance significantly
- **Heading Tolerance Tuning**: May be able to tighten tolerance in open sections while keeping loose tolerance in garage
- **Bump Recovery Optimization**: Once garage is reliable, could optimize CP 5→6 wall interaction for bonus points

Future Speed Optimization Plan
-------------------------------

To achieve faster trial times, the following major improvements are needed:

**1. Increase Overall Speed**
   - Higher motor duty cycles throughout the course
   - Requires redesigning the scheduler task timing and tuples
   - Need to ensure control loop runs fast enough for increased speeds
   - At the current speed (50mm/s), detouring to cups takes >3 seconds (negating the 3-second bonus). Higher speeds will make cup collection worthwhile.

**2. Optimize IR Sensor Readings**
   - Current IR sensor polling may be limiting responsiveness
   - Reduce sensor read latency
   - Improve sensor data processing efficiency
   - Consider faster sampling rates or interrupt-driven updates

**3. Retune Closed-Loop Control**
   - Current control gains optimized for slower speeds
   - Higher speeds will require re-tuning PID/heading control parameters
   - May need increased gains to handle faster dynamics
   - Test stability margins at higher speeds

**4. Scheduler and Timing Restructure**
   - Revisit task scheduling periods (tuples)
   - Ensure all critical tasks (motor control, sensor reads, heading control) run fast enough
   - Balance computation load across tasks
   - May need to reduce lower-priority task frequencies

**Implementation Priority**:
   1. Profile current scheduler performance and identify bottlenecks
   2. Optimize IR sensor read cycle
   3. Increase speed incrementally while monitoring control stability
   4. Retune closed-loop controllers at each speed increment
   5. Final full-speed validation run
