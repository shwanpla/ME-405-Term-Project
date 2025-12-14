Tasks
=====
This page documents the cooperative multitasking architecture used on Romi and the finite state machines that govern each subsystem task. The diagrams are intended to make the runtime behavior of the robot explicit by showing how sensing, estimation, control, and navigation execute concurrently under the cotask scheduler while exchanging information through task_share shares and queues. The task diagram describes system level signal flow, and the FSM diagrams describe how each task progresses through its internal operating modes during calibration, trial execution, navigation, and shutdown.

.. contents:: Table of Contents
   :depth: 2
   :local:

--------

=============================
System Architecture Overview
=============================

Task Diagram
------------

The task diagram represents the complete software architecture executed on the microcontroller during a run. Each box corresponds to a cooperative task implemented as a generator, and arrows represent the real time variables passed through task_share shares and queues. Low level tasks handle hardware interface and telemetry generation, mid level tasks implement control and estimation, and the top level navigation task coordinates the entire system by setting mode flags and setpoints rather than calling hardware directly. This separation is what allows Romi to run multiple subsystems simultaneously with deterministic timing while keeping the navigation logic readable and modular.

.. image:: /images/task_diagram.png
   :width: 90%
   :align: center

.. centered::
   *System task diagram showing the interconnection of all cooperative tasks and the data flow through shared variables and queues.*

In operation, the serial task acts as the external command and calibration interface, the motor control tasks apply effort and produce encoder feedback, the closed loop control tasks compute effort commands for velocity tracking and line following, and the heading task converts encoder motion into distance and heading state estimates. Navigation consumes these estimates and sensor results to decide what behavior should be active at each point in the course. Event style tasks such as the bump sensor provide discrete triggers that allow navigation to enter recovery sequences, while sensing tasks such as the IMU handler provide inertial data used for heading stabilization and turn accuracy when enabled.

--------

==============================
Individual Task State Machines
==============================

Serial Communication Task
-------------------------

The serial communication task represents the system’s external interface during calibration and timed trials. It listens for Bluetooth commands at high baud rate, parses parameter messages, and sets the shared variables that arm the run and configure control targets. This task also gates the IR calibration sequence by transitioning through explicit modes for black reference, white reference, and calibration completion, ensuring the line following controller receives normalized sensor data before the robot begins autonomous motion.

.. image:: /images/serial_task_fsm.png
   :width: 80%
   :align: center

.. centered::
   *Serial task FSM managing Bluetooth command parsing, calibration sequencing, and trial start and stop control.*


In its waiting mode, the task continuously checks for ASCII commands and updates shared variables such as desired velocity, trial time, and calibration flags. After parameters and calibration are complete, it asserts start flags that enable the drivetrain tasks and control tasks to begin execution. During an active trial, the task monitors for stop commands and end conditions, then drives a clean termination by asserting end flags and clearing any buffered logging queues so the next run begins from a consistent state.

Motor Control Task
------------------

The motor control task represents the lowest level actuation and wheel telemetry subsystem. Two identical instances run simultaneously, one for the left drivetrain and one for the right drivetrain. Each task is responsible for enabling the motor driver, applying the commanded PWM effort setpoint, updating encoder position and velocity at high rate, and publishing telemetry through shared variables so that closed loop control and estimation remain synchronized with the drivetrain.

.. image:: /images/motor_ctrl_task_V3_fsm.png
   :width: 80%
   :align: center

.. centered::
   *Motor control FSM enabling the driver, zeroing the encoder, applying PWM effort, and publishing encoder telemetry during a trial.*

The start state holds the motor disabled until a shared start flag is asserted, then enables the driver and zeroes the encoder to establish a consistent reference. During the data collection state, the task repeatedly calls the encoder update routine, computes a filtered velocity estimate, updates shared values for position, speed, and elapsed time, and applies the latest effort command using the motor driver interface. The task monitors time limits and end flags to disable the motor cleanly and return to the start state for the next trial.

Closed-Loop Control Task
-------------------------

The closed loop control task represents the velocity, line tracking, and turning control layer that generates drivetrain effort commands. Two instances run simultaneously, producing independent effort outputs for the left and right motor control tasks. The controller supports multiple operating modes so that navigation can switch between line following segments, forced straight segments, and heading based turns while preserving stable PI velocity regulation underneath.

.. image:: /images/CL_control_task_V5_fsm.png
   :width: 80%
   :align: center

.. centered::
   *Closed loop control FSM implementing PI velocity regulation with line following correction, rest mode, and heading based turning mode.*

In its main run state, the controller tracks a desired velocity using encoder speed feedback with PI regulation and saturation handling. When line following is enabled, it adds a reflectance based centroid error term that biases left and right effort commands in opposite directions to steer back to the tape, and it manages integral accumulation to prevent windup during saturation. When force straight mode is enabled, line correction is suppressed and the controller prioritizes heading stability. In the turn state, the controller interprets a target heading from navigation, wraps heading error into a bounded range, and applies a dedicated PI turn law to drive the robot into the desired angular window before returning to the run state.

Encoder Heading Task
--------------------

The encoder heading task represents the real time odometry estimator based on differential drive kinematics. It runs as a continuous periodic loop rather than a multi state FSM, converting left and right encoder motion into forward distance traveled and heading change. This task provides the primary state variables used by navigation to trigger distance based transitions and to evaluate heading error during forced straight and turn segments.

.. image:: /images/encoder_heading.png
   :width: 85%
   :align: center

.. centered::
   *Encoder based odometry computation converting left and right wheel motion into forward distance and heading used by navigation and turning control.*

Each update converts encoder ticks into wheel displacement, integrates the average displacement into distance, and integrates the wheel displacement difference into heading using the known track width. To keep the heading estimate usable for control, the algorithm handles wraparound at plus or minus 180 degrees by unwrapping discontinuities and re normalizing the final heading into a consistent range. The output shares provide a continuously updated distance and heading estimate that all higher level logic can consume without directly reading encoder hardware.

Navigation Task
---------------

The navigation task represents the top level supervisory finite state machine that executes the obstacle course strategy. It does not perform low level control directly. Instead, it commands system behavior by setting shared mode flags, desired velocities, target headings, and line bias values that the closed loop control layer interprets. This design keeps navigation focused on sequencing and decision making while the lower level tasks remain responsible for timing critical sensing and actuation.

.. image:: /images/navigation_fsm.png
   :width: 95%
   :align: center

.. centered::
   *Navigation finite state machine coordinating line following, force straight segments, heading based turns, and bump triggered recovery behavior.*

The navigation FSM evaluates distance, heading, and event flags at a fixed period and transitions when thresholds are reached or when heading error falls within a specified tolerance. During line following segments it enables line tracking and may apply bias to prefer a branch at a fork. During non line regions it enables force straight mode and uses heading feedback to maintain a commanded orientation. For precise reorientation, navigation asserts a turning mode and updates the desired heading share until the control layer reports convergence. When a bump event is detected, navigation switches into a recovery sequence that backs away, reorients, and returns to the line so the run can continue or conclude cleanly.

Bump Sensor Task
----------------

The bump sensor task represents a discrete event detector used for wall interaction handling. It continuously monitors a digital input and sets a shared bump flag when contact is detected, providing a clean trigger for navigation to transition into a recovery routine. The task is designed to avoid noisy repeated triggers by enforcing a single confirmed event per run using debounce logic and a latched detected state.

.. image:: /images/bump_task.png
   :width: 70%
   :align: center

.. centered::
   *Bump sensor task debouncing contact input and publishing a single collision event flag to navigation.*

Because this task publishes an event rather than a continuous measurement, its output is treated as an interrupt style condition within the FSM logic. When asserted, navigation can reliably assume a collision occurred and execute a deterministic recovery sequence without ambiguity due to switch chatter or vibration.


.. IMU Handler Task
.. ----------------
..
.. The IMU handler task represents inertial sensor management over I2C, including initialization, calibration management, and data readiness. It is responsible for bringing the IMU into a known operating mode and ensuring that usable heading or yaw rate information is available to other tasks. Calibration persistence is handled by loading a saved calibration blob when present, and saving new calibration data after a successful manual procedure so repeated runs can start quickly with consistent sensor behavior.
..
.. .. image:: /images/imu_handler_fsm.png
..    :width: 90%
..    :align: center
..
.. .. centered::
..    *IMU handler FSM performing BNO055 reset, calibration load or manual calibration, calibration persistence, and steady state data readiness over I2C.*
..
.. During initialization the task performs a reset sequence and brings up the I2C interface. It then checks for stored calibration data and loads it to avoid repeated manual calibration. If no valid calibration exists, it enters a manual calibration mode that monitors calibration status until completion, then saves the calibration blob for future boots. Once complete, the task remains in a steady state where inertial measurements can be sampled consistently and provided to the rest of the system through shared variables.
..
.. Observer Task
.. -------------
..
.. The observer task represents model based state estimation implemented in state space form. It runs concurrently with the control stack, using measured outputs and applied inputs to estimate internal states that are not directly measured or that benefit from filtering. The observer gain is selected using pole placement to achieve stable convergence behavior, and RK4 integration is used to propagate the state estimate forward in time under the discrete task period.
..
.. .. image:: /images/observer_block.png
..    :width: 85%
..    :align: center
..
.. .. centered::
..    *Observer structure showing state space estimation with Luenberger gain injection and RK4 integration under a discrete task period.*
..
.. This task formalizes estimation as part of the architecture rather than ad hoc filtering. By publishing estimated states through shares, the observer output can be consumed by other tasks without changing their hardware interfaces, allowing estimation improvements to be integrated cleanly into navigation or control when required by the course segment or sensing constraints.
