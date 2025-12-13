Tasks
=====

The system uses 8 concurrent tasks scheduled at different priorities and periods:

Serial Task (150ms)
===================

**Purpose**: Bluetooth communication and configuration

**Responsibilities**:

- Receives calibration commands (BLACK, WHITE, COMPLETE)
- Parses velocity parameters
- Relays navigation logs via Bluetooth queue

**Flow Diagram**::

    [BT Input] → [Parse Command] → [Set Flags/Velocities] → [Drain Nav Queue to BT]
         ↓
    [CALIBRATION_COMPLETE] → [Start Navigation] → [Monitor for END_COMM]

Motor Control Tasks (12ms period)
==================================

**Left/Right Motor Control** (2 tasks)

**Purpose**: Low-level motor velocity regulation

**Responsibilities**:

- Read desired velocity from navigation task
- Apply PWM to motor via H-bridge
- Track actual motor speed via encoder
- Update effort level

**Flow Diagram**::

    [Desired Velocity] → [PWM Driver] → [Motor] → [Encoder] → [Actual Speed]
                            ↑                                        ↓
                            └────────────[Feedback]─────────────────┘

Closed-Loop Control Tasks (20ms period)
========================================

**Left/Right CL Control** (2 tasks)

**Purpose**: PI velocity and heading control

**Responsibilities**:

- Calculate velocity error (desired - actual)
- Apply PI gains (Kp, Ki) for velocity
- Handle line following bias adjustment
- Implement heading correction when force_straight_flg=0
- Prevent bias updates when force_straight_flg=1

**Flow Diagram**::

    [Desired Vel] ─→ [PI Velocity Controller] ─→ [Motor Effort]
    [Actual Vel]  ←─                           ↑
                                               │
    [Heading Error] ─→ [Heading Correction] ──┤
    [Line Bias] ────→ [Line Following]  ──────┘
    [force_straight_flg] ─→ [Disable Line Bias]

Encoder Heading Task (50ms period)
===================================

**Purpose**: Calculate robot heading and distance from encoders

**Responsibilities**:

- Read left/right encoder positions
- Calculate delta positions (odometry)
- Compute heading angle using encoder difference
- Update enc_heading_share and enc_distance_share

**Flow Diagram**::

    [Left Encoder] ─┐
                   ├─→ [Odometry Calc] ─→ [Heading = atan2(ΔL-ΔR)]
    [Right Encoder]─┘
                   ├─→ [Distance Accumulation] ─→ [Total Distance]

Navigation Task (50ms period)
=============================

**Purpose**: 24-state obstacle course state machine

**Responsibilities**:

- Monitor calibration and bump sensor
- Execute state transitions based on distance/heading
- Send velocity and heading targets to controllers
- Manage line following bias and straight motion
- Handle wall collision recovery (7-state sequence)
- Log all state changes via queue

**Flow Diagram**::

    [Calibration Flag] ─→ [STATE 0: Line Follow] ─→ [Distance Check]
                              ↓
                         [STATE 1-8: Obstacles] ─→ [Turns + Straights]
                              ↓
                         [STATE 9-15: Exit Sequence] ─→ [Bump Detected?]
                              ├─→ [STATE 16-22: Recovery] ─→ [STATE 23: Stop]
                              └─→ [Continue Course]

**State Categories**:

- **Line Following**: 0, 1, 4, 7, 8, 11, 22 (bias varies)
- **Straight Sections**: 3, 6, 10, 13, 15, 18, 20 (heading control only)
- **Turning States**: 2, 5, 9, 12, 14, 17, 19, 21 (rotate in place)
- **Instant Capture**: 3, 6 (capture current heading on entry)
- **Recovery Sequence**: 16-23 (7 states for wall escape)

Bump Sensor Task (20ms period)
==============================

**Purpose**: Monitor collision sensor

**Responsibilities**:

- Poll bump sensor pin
- Set bump_detected_share when collision occurs
- Trigger navigation recovery sequence

**Flow Diagram**::

    [Bump Sensor] ─→ [Read Pin] ─→ [bump_detected_share = 1]
                        ↓
                   [Navigation detects] ─→ [Enters STATE 16]

Task Interactions
=================

The tasks communicate via shared memory (task_share):

.. list-table::
   :header-rows: 1

   * - Share
     - Source
     - Destination
     - Purpose
   * - left_desired_vel
     - Navigation
     - Motor + CL Control
     - Target velocity
   * - enc_heading_share
     - Encoder Task
     - Navigation + CL Control
     - Current heading
   * - enc_distance_share
     - Encoder Task
     - Navigation
     - Total distance
   * - bump_detected_share
     - Bump Sensor
     - Navigation
     - Collision flag
   * - line_follow_flg
     - Navigation
     - CL Control
     - Enable/disable line follow
   * - force_straight_flg
     - Navigation
     - CL Control
     - Disable line bias
   * - bias_share
     - Navigation
     - CL Control
     - Line following adjustment
   * - desired_angle_share
     - Navigation
     - CL Control
     - Target heading

Task Timing
===========

The scheduler runs at ~1kHz. Task periods:

- Serial Task: 150ms (low priority, communication)
- Motor Tasks: 12ms (high priority, direct control)
- CL Control Tasks: 20ms (high priority, feedback)
- Encoder Task: 50ms (medium priority, odometry)
- Navigation Task: 50ms (medium priority, state machine)
- Bump Sensor Task: 20ms (high priority, safety)
