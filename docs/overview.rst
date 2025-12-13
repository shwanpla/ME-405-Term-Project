Overview
========

.. image:: /images/romi_isometric.png
   :width: 400px
   :align: center

Project Summary
---------------

This documentation covers the complete design and implementation of an **autonomous obstacle course navigation system** for a Romi robot using the NUCLEO-L476RG microcontroller.

The robot autonomously navigates a 24-state obstacle course using:

- **Encoder-based odometry** for distance and heading tracking
- **IR line sensors** for obstacle detection and steering
- **Bump sensor** for collision recovery
- **Dual closed-loop motor controllers** for precise speed and heading control
- **Multi-task scheduler** coordinating 8 concurrent tasks

**Key Achievement**: Successfully completes the obstacle course in ~90-100 seconds with >95% reliability, including automated recovery from wall collisions.

What's Inside
-------------

**Overview** - High-level project goals and system architecture

**Hardware** - Robot platform, motors, encoders, and sensor specifications

**Wiring** - Complete pinout and electrical connections for all components

**Tasks** - Detailed breakdown of 8 concurrent tasks with flow diagrams showing:
   - Motor control and regulation
   - Closed-loop steering and heading stabilization
   - Encoder odometry calculations
   - Navigation state machine (24 states)
   - Bump sensor collision handling
   - Bluetooth telemetry

**Source Code** - File organization, key design patterns, and code examples

**Alternative Approaches** - Design decisions: what was tried, what failed, and why current solutions were chosen

**Analysis** - Performance metrics, error analysis, sensor performance, and optimization opportunities

**Time Trials** - Actual run data with timing breakdowns, variability analysis, and energy consumption

Project Goal
-----------

Develop a fully autonomous robot capable of navigating a complex obstacle course without human intervention, recovering from unexpected collisions, and completing the course reliably within a reasonable time frame.

System Architecture
------------------

The system uses a **multi-task real-time scheduler** with 8 concurrent tasks:

- **Motor Tasks** (12ms): Direct PWM control and velocity feedback
- **Control Tasks** (20ms): PI velocity and heading regulation
- **Encoder Task** (50ms): Odometry calculation from wheel encoders
- **Navigation Task** (50ms): 24-state obstacle course state machine
- **Serial Task** (150ms): Bluetooth communication and telemetry
- **Bump Sensor Task** (20ms): Collision detection

The navigation task sends high-level commands (velocity, heading) to the motor controllers while continuously monitoring sensors for obstacles and state transitions. This separation allows smooth, responsive control without blocking on decision logic.

**Key Innovation**: Instant heading capture on turn-to-straight transitions eliminates overshoot and provides instantaneous state changes without waiting for motor adjustment.
