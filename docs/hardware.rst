Hardware
========

Platform
--------

**Romi Robot** with NUCLEO-L476RG microcontroller

Motors
------

- **Left/Right DC Motors** with 30 kHz PWM (Timer 3)
- Motor driver control via GPIO pins (H-bridge)
- Bidirectional speed control (-100 to +100 mm/s)

Encoders
--------

- **Left Encoder** (Timer 2): PA0 (A), PB3 (B)
- **Right Encoder** (Timer 1): PA8 (A), PA9 (B)
- Resolution: 16-bit counters for position tracking
- Used for distance and heading calculation

Sensors
-------

**IR Line Sensors (7 channels)**:

- PC4, PB1, PA7, PC1, PA4, PA1, PC3
- Returns analog values (0-4095)
- Calibrated for black/white thresholds
- Used for line following bias calculation

**Bump Sensor**:

- Triggers wall collision detection
- Initiates 7-state recovery sequence
- State 15 monitors continuously

Communication
-------------

**Bluetooth Serial (UART 1)**:

- Baud rate: 460800
- Used for configuration and telemetry
- Serial task relays navigation logs
- UI_Line_Follow.py for manual control
