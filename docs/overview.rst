Overview
========

Project Goal
-----------

Develop a fully autonomous obstacle course navigation system for a Romi robot using encoder-based odometry and IR line-following sensors.

Key Features
-----------

- **24-state obstacle course navigation** with encoder distance tracking
- **Bump sensor recovery sequence** for wall collision handling
- **Dual-motor closed-loop control** with heading stabilization
- **Real-time state logging** streamed via Bluetooth
- **Instant heading capture** for smooth turn-to-straight transitions

System Architecture
------------------

The system uses a multi-task scheduler coordinating:

- Motor control (velocity regulation)
- Closed-loop steering (heading + line following)
- Encoder odometry (distance + heading calculation)
- Navigation state machine (24 states)
- Bump sensor monitoring
- Bluetooth telemetry

The navigation task sends commands to motor controllers while monitoring sensors in parallel, enabling smooth obstacle avoidance and recovery from collisions.
