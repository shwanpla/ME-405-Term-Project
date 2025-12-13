# ME-405-Term-Project
This repository contains firmware and Python/MicroPython code for a Romi (differential-drive) robot controlled by an STM32 microcontroller for ME 405 – Mechatronics. It brings together motor control, encoder feedback, BNO055 IMU integration, line and auxiliary sensor drivers, calibration routines, and a set of high-level tasks for navigation, data logging, and course time-trial experiments.

The goal of this project is to develop a reliable, modular codebase that can:
- Drive the Romi robot along a prescribed track,
- Maintain robust state estimation using onboard sensors, and  
- Provide a platform for experimenting with control strategies and embedded software design patterns.

---

## Project Objectives

- Implement **real-time motor control** for a differential-drive platform using PWM and encoder feedback.
- Integrate a **BNO055 IMU** for heading/orientation estimation and calibration.
- Design **task-based firmware** using cooperative multitasking (e.g., `cotask`, `task_share`) to structure robot behavior.
- Develop and test **navigation routines** for line following, straight-line motion, and turning.
- Log relevant data (position, heading, velocities, sensor readings) for offline analysis.
- Support **ME 405 lab experiments** and final time-trial challenges with a flexible and extensible codebase.

---

## Features

- **Motor & Encoder Control**
  - PWM-based speed control for both left and right motors.
  - Encoder drivers for each wheel for odometry and closed-loop speed/position control.
  - Support for basic velocity controllers and open-loop testing.

- **BNO055 IMU Integration**
  - Initialization and configuration of the BNO055 over I²C.
  - Orientation and angular velocity readings (e.g., heading/yaw).
  - Calibration routines with persistent storage of calibration offsets.

- **Sensor Drivers**
  - Line sensor support (for line-following behavior).
  - Hooks for additional sensors (e.g., distance sensors, auxiliary digital inputs).

- **Task-Based Architecture**
  - Cooperative multitasking using ME 405’s `cotask` and `task_share` framework.
  - Separate tasks for IMU reading, motor control, line-following logic, and data logging.
  - Clear separation of responsibilities between high-level behavior and low-level drivers.

- **Calibration & Utilities**
  - IMU calibration sequence with instructions and/or guided prompts.
  - Encoder/velocity checks and basic tuning support.
  - Utility functions for unit conversions, timing, and safety checks.

- **Data Logging and Analysis**
  - Optional logging of runtime data (e.g., encoder counts, heading, motor commands).
  - Designed to export data for plotting and analysis in tools like Python/Matplotlib or MATLAB.

---

## Hardware Overview

This project targets a Romi chassis with:

- **Chassis:** Pololu Romi (or equivalent) two-wheel differential-drive platform.
- **Microcontroller:** STM32-based development board (e.g., Nucleo) mounted on the Romi.
- **Actuation:**
  - Two DC gear motors with quadrature encoders.
- **Sensors:**
  - BNO055 IMU (orientation/accel/gyro).
  - Line sensor array (for track-following).
  - Additional GPIO pins available for extra sensors if needed.
- **Power:**
  - Battery pack mounted on the chassis.
  - Regulated 5 V / 3.3 V supplies for logic and sensors as required.

> Exact wiring diagrams, pin mappings, and part numbers should be described in the hardware documentation section of this repo (see `docs/` or `elec/` if present).

---

## Software Architecture

The firmware is organized into layers:

1. **Low-Level Drivers**
   - Motor driver interface (PWM channels, direction control).
   - Encoder driver for each wheel (interrupt or timer-based).
   - BNO055 IMU driver (I²C transaction layer + register interface).
   - Line sensor driver and digital/analog input configuration.

2. **Middleware / Utilities**
   - Timing utilities (e.g., millisecond counters, delays).
   - Shared variables using `task_share`.
   - Generic math/helpers (e.g., angle normalization, unit conversions).

3. **Task Layer**
   - Individual tasks for:
     - Reading IMU and updating heading.
     - Reading line sensors and computing line error.
     - Running motor control loops (speed/position).
     - High-level navigation/state machine (e.g., idle, follow line, stop).
     - Data logging tasks for offline analysis.

4. **Application / Experiment Code**
   - Scripts or task configurations specific to:
     - Lab exercises.
     - Time-trial experiments.
     - Tuning and debugging sessions.

This layered approach is intended to make it easier to reuse low-level drivers while modifying high-level behavior for different labs or experiments.

---
