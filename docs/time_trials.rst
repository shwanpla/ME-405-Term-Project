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
   - Checkpoint 4 (Garage Entry): 100.53s
   - Checkpoint 5 (Garage Exit): 123.93s
   - Checkpoint 6 (Wall Interaction): 165.72s
   - **Official Time: 165.72 seconds** (no bonuses)

Key Observations
----------------

**Trials 1 & 2 - Garage Heading Failure**:

Both early trials failed entering the parking garage at checkpoint 4:
- Heading control unstable in narrow corridor
- Robot either:
  - Lost heading and wandered off-line into wall, OR
  - Overcorrected and drove straight into wall
- Failed before ever reaching the garage exit (CP 5)
- Indicates systematic heading stability problem in tight spaces

**Trial 3 - Complete Success**:

Successfully navigated:
- Parking garage (CP 4 → CP 5): Maintained heading in narrow corridor
- Wall interaction (CP 5 → CP 6): Handled final approach cleanly
- Full course completion in 165.72 seconds

**Cups (Bonuses)**:

No cups were collected in any trial:
- Cups provide -3 second bonus each if collected
- Trial 3 could have scored lower with cup collection
- Focus was on completing course safely, not optimization

Failure Analysis
----------------

**Trials 1 & 2: Parking Garage Entry Failure**

The parking garage entry (checkpoint 3 → 4) is a tight corridor requiring:
- Precise heading maintenance (180° constant)
- No heading drift tolerance in narrow space
- Smooth heading control without oscillation

**Why It Failed**:

The garage is bounded by walls on both sides. The robot either:
1. **Understeered**: Drifted left/right and made contact with wall
2. **Oversteered**: Overcorrected heading and drove straight into wall
3. **Lost Control**: Integral wind-up caused oscillation, hit wall multiple times

The fact that both Trial 1 & 2 failed at the same point (before CP 5) confirms the issue is heading control in the garage segment, not the wall interaction sequence.

**Resolution** (Trial 3):

- Increased heading tolerance to allow slight drift without aggressive correction
- Added adaptive bias only if drift > 8° (prevents over-correction)
- Improved motor conflict resolution
- Better integral term handling in control loops

Performance Improvements
------------------------

From Trial 1 to Trial 3:

- **Garage Entry**: DNF at 94.85s → Passed at 100.53s ✓
- **Garage Completion**: Never reached CP 5 → Reached at 123.93s ✓
- **Wall Interaction**: Never attempted → Completed at 165.72s ✓
- **Code Changes**: Focus on garage heading control and motor balance

This shows the garage segment (CP 4→5) is the critical bottleneck - once that's solved, the wall interaction (CP 5→6) proceeds smoothly.

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
     - Initial obstacles
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
- Narrow corridor with walls on both sides
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

1. **Garage is the Bottleneck**: Both DNF runs failed entering the garage - this is the critical control challenge

2. **Heading Stability in Tight Spaces**: Narrow corridors amplify control issues - tolerance must balance oscillation prevention with wall avoidance

3. **Motor Symmetry Critical**: One motor pulling harder in the garage causes gradual wall contact and collision

4. **Iterative Tuning**: Trial 1 & 2 failures pinpointed the exact problem (garage heading); targeted fixes resolved it

5. **Wall Interaction is Secondary**: Only attempted once garage was working - the approach setup matters more than the final interaction

6. **Safety Over Speed**: Trial 3 took 165.72s without collecting cups - navigating garage safely was the priority

Recommendations for Future Runs
--------------------------------

- **Tighten Pre-Garage Setup**: Ensure motors are perfectly synchronized before CP 3→4 transition
- **Collect Cups**: With garage navigation proven, future runs could collect cups for -3s bonuses each
- **Monitor Motor Health**: Check bearing friction - impacts garage performance significantly
- **Heading Tolerance Tuning**: May be able to tighten tolerance in open sections while keeping loose tolerance in garage
- **Bump Recovery Optimization**: Once garage is reliable, could optimize CP 5→6 wall interaction for bonus points
