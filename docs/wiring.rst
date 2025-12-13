Wiring
======

Motor Control
-------------

**Left Motor**:

- Enable: PB0
- PWM A: PC12
- PWM B: PC10
- Timer 3, Channel 3

**Right Motor**:

- Enable: PC9
- PWM A: PH1
- PWM B: PH0
- Timer 3, Channel 4

Encoder Connections
-------------------

**Left Encoder (Timer 2)**:

- Channel A: PA0
- Channel B: PB3
- 16-bit counter

**Right Encoder (Timer 1)**:

- Channel A: PA8
- Channel B: PA9
- 16-bit counter

IR Sensor Array
---------------

.. list-table::
   :header-rows: 1

   * - Channel
     - Pin
     - Position
   * - 1
     - PC4
     - Left far
   * - 2
     - PB1
     - Left mid
   * - 3
     - PA7
     - Left near
   * - 4
     - PC1
     - Center
   * - 5
     - PA4
     - Right near
   * - 6
     - PA1
     - Right mid
   * - 7
     - PC3
     - Right far

Bump Sensor
-----------

- Momentary switch (active high)
- Wired to external interrupt pin
- Triggers on collision during STATE 15

Serial Communication
--------------------

- UART 1 at 460800 baud
- TX/RX pins (standard NUCLEO config)
- Connected to Bluetooth module or USB-to-serial adapter
