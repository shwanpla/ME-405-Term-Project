Romi Autonomous Navigation
==========================

Overview
--------
Autonomous obstacle course navigation system using encoder-based odometry, closed-loop motor control, and adaptive line-following. The robot autonomously navigates a multi-state obstacle course with bump sensor wall recovery and real-time multi-task scheduling.

.. toctree::
   :maxdepth: 2
   :caption: Documentation

   design
   navigation_algorithm
   control_scheme
   hardware

.. toctree::
   :maxdepth: 1
   :caption: Code Reference

   modules

Key Features
============

- **24-State Navigation FSM** - Encoder-based odometry with no IMU
- **Bump Sensor Recovery** - Autonomous wall detection and escape sequence
- **Closed-Loop Motor Control** - PI velocity and heading regulation
- **Adaptive Line-Following** - Dynamic bias with integral wind-up prevention
- **Real-Time Task Scheduling** - 8 concurrent tasks with priority execution
- **Bluetooth Telemetry** - Real-time diagnostics and logging

Project Structure
=================

::

    romi-navigation/
    ├── src/                    # Python source code
    │   ├── main.py
    │   ├── navigation_v2_compact_encoder.py
    │   ├── CL_control_task_V5.py
    │   └── ...
    ├── docs/                   # Sphinx documentation
    ├── hardware/               # Wiring diagrams, mechanical designs
    └── README.md

Quick Start
===========

1. Clone the repository
2. Upload source files to NUCLEO-L476RG board
3. Connect via Bluetooth at 460800 baud
4. Send ``PARAMS,0,50`` to start obstacle course
5. Monitor via ``UI_Line_Follow.py``

Technologies
============

- **Microcontroller:** STM32L476RG
- **Language:** MicroPython
- **Sensors:** 7-sensor IR array, quadrature encoders, bump sensor
- **Architecture:** Task-based scheduler with shared memory communication

.. image:: images/system_diagram.png
   :alt: System Architecture
   :width: 100%

Performance
===========

- **Course Completion:** Full obstacle course with wall recovery
- **Reliability:** Robust state transitions and error handling
- **Speed:** Optimized for smooth line-following and turning

See Also
========

- :doc:`design` - Mechanical and electrical design details
- :doc:`navigation_algorithm` - 24-state FSM explanation
- :doc:`control_scheme` - PI control implementation
- :doc:`hardware` - Hardware specifications and wiring
