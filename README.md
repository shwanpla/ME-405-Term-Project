# ME-405-Term-Project - Romi

In **ME 405 Mechatronics**, our three-person team delivered an autonomous two-wheeled differential-drive robot named **Romi**, engineered for robust, repeatable navigation of a timed obstacle course. The system combines embedded hardware integration, real-time **MicroPython** firmware on an STM32 Nucleo, cooperative multitasking, multi-sensor instrumentation, and closed-loop control with state-space methods to produce reliable behavior across multiple trials.

**Documentation:** https://me-405-term-project.readthedocs.io/en/latest/index.html

**Repository:** https://github.com/shwanpla/ME-405-Term-Project

## What this project demonstrates
Romi’s autonomy is built from modular subsystems (drivers and generator-based tasks) coordinated by a top-level navigation task. This architecture supports deterministic timing, clean separation between sensing, control, and decision logic, and a repeatable tuning workflow using live telemetry and physical trials.

Key capabilities implemented in this project include:
- Cooperative scheduling with generator tasks and shared-data interfaces
- Motor actuation with PWM and encoder feedback for closed-loop velocity control
- Centroid-based QTR line tracking with calibration and steering injection
- Encoder odometry for distance and heading estimation
- IMU integration over **I2C** for heading/yaw feedback and calibration management
- Event-driven bump detection and recovery behaviors
- Bluetooth serial telemetry for tuning, diagnostics, and data capture

## Live demonstration videos
- **Successful full course run:** https://youtu.be/gsIIfVX_UOE  
- **Line-following breakthrough:** https://youtu.be/3hr3dCyPbDM  

## Hardware stack summary
Romi integrates a drivetrain and embedded electronics stack designed for repeatable sensing and stable control:
- Pololu Romi differential-drive chassis with DC gearmotors and quadrature encoders  
- STM32 Nucleo (MicroPython target) + Shoe of Brian interface board  
- Pololu QTR reflectance sensor array (7-channel) with controlled standoff mounting  
- BNO055 IMU over I2C  
- Bump sensor input for collision detection  
- Battery monitoring via ADC using a custom voltage-divider circuit  
- Bluetooth serial module for live telemetry and parameter updates  

Detailed wiring tables, pin maps, and constraints are documented here:  
https://me-405-term-project.readthedocs.io/en/latest/wiring.html :contentReference[oaicite:1]{index=1}

## Code architecture
Firmware is organized into reusable drivers (classes) and cooperative tasks (generators). The scheduler runs each task at a defined period, while shared variables/queues provide clean, low-overhead data exchange between subsystems.

Primary sections of the documentation:
- **Hardware:** components and technical roles :contentReference[oaicite:2]{index=2}  
- **Wiring:** pin constraints, interfaces, and full assembly :contentReference[oaicite:3]{index=3}  
- **Tasks:** system task diagram + FSMs and responsibilities :contentReference[oaicite:4]{index=4}  
- **Repository source reference:** module-level breakdowns :contentReference[oaicite:5]{index=5}  
- **Analysis & tuning narrative:** telemetry, control equations, calibration, and validation :contentReference[oaicite:6]{index=6}  
- **Time trials:** official results and performance discussion :contentReference[oaicite:7]{index=7}  

## Repository layout
- `src/` – MicroPython firmware (drivers, tasks, and top-level `main.py`)
- `docs/` – Sphinx documentation source
- `docs/images/` – figures used across the documentation (hardware, wiring, FSMs, course maps)

## Build and run
This repository is structured for a MicroPython workflow on the STM32 Nucleo:
1. Flash MicroPython onto the Nucleo target.
2. Wire the robot using the documented pin maps and constraints.
3. Copy the `src/` files onto the board storage (or deploy via your preferred MicroPython toolchain).
4. Run `main.py` to start the scheduler and enable Bluetooth command/telemetry.

The documentation provides the authoritative wiring and subsystem descriptions needed to reproduce the build. :contentReference[oaicite:8]{index=8}
