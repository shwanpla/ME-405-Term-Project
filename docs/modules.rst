Code Reference
==============

.. toctree::
   :maxdepth: 1

   modules/main
   modules/navigation
   modules/control
   modules/motor
   modules/encoder
   modules/sensors
   modules/communication

Main Entry Point
~~~~~~~~~~~~~~~~

**main.py** - System initialization and task scheduler

Sets up all hardware (motors, encoders, sensors) and creates the task scheduler with 8 concurrent tasks:

- Serial communication task
- Left/right motor control tasks
- Left/right closed-loop control tasks
- Encoder heading calculation task
- Navigation state machine task
- Bump sensor monitoring task

Navigation
~~~~~~~~~~

**navigation_v2_compact_encoder.py** - 24-state obstacle course FSM

Handles high-level course navigation with encoder odometry, bump sensor detection, and wall recovery sequences.

Control
~~~~~~~

**CL_control_task_V5.py** - Closed-loop PI velocity and heading control

Implements dual PI controllers (velocity and heading) with line-following integration and battery compensation.

Motor & Encoder
~~~~~~~~~~~~~~~

**motor.py** - Motor driver interface

**motor_ctrl_task_V3.py** - Low-level motor control task

**encoder.py** - Quadrature encoder with velocity averaging

**encoder_heading_task.py** - Odometry calculation (heading & distance)

Sensors
~~~~~~~

**multi_sensor_read.py** - 7-sensor IR array interface

**line_follow_V1.py** - Line error calculation with bias adjustment

**bump_sensor_task.py** - Bump sensor debouncing and detection

**battery_adc.py** - Battery voltage monitoring

Communication
~~~~~~~~~~~~~~

**serial_task_V6.py** - Bluetooth serial interface and command handling

**UI_Line_Follow.py** - Python PC application for trials
