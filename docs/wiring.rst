Wiring
======

This page defines the electrical interconnects used on Romi, including motor driver control, encoder feedback, reflectance sensing, inertial sensing, bump event inputs, battery monitoring, and Bluetooth serial communication. The goal is to provide a single reference for connector level wiring, Nucleo pin assignments, and the signals required by each subsystem task.

.. image:: /images/nucleo_pinout.png
   :width: 900px
   :align: center

.. centered::
   *STM32 Nucleo pinout reference used to assign timer, ADC, I2C, and UART signals for the Romi wiring harness.*

Nucleo Pin Constraints
----------------------

Several STM32 Nucleo pins are reserved for onboard functions such as ST-Link communication, USB, SWD programming, the user LED and button, and the 32 kHz oscillator. These pins were treated as unavailable during integration to preserve reliable REPL access, firmware flashing, and debugging support throughout development and testing.

.. centered::
   *Reserved Nucleo pins that were avoided to prevent conflicts with ST-Link, USB, SWD, onboard LED and button, and RTC oscillator functions.*

Reserved pins not used in this project

.. list-table::
   :header-rows: 1
   :widths: 25 25 50

   * - Nucleo Pin
     - Arduino Label
     - Reserved Function
   * - PA2
     - A2
     - UART2 link to ST-Link interface
   * - PA3
     - A3
     - UART2 link to ST-Link interface
   * - PA5
     - A5
     - Onboard user LED
   * - PA11
     - A11
     - USB interface signals through the shoe
   * - PA12
     - A12
     - USB interface signals through the shoe
   * - PA13
     - A13
     - SWD programming interface
   * - PA14
     - A14
     - SWD programming interface
   * - PC13
     - C13
     - User button input
   * - PC14
     - C14
     - 32 kHz oscillator for RTC
   * - PC15
     - C15
     - 32 kHz oscillator for RTC
Motor Control
-------------

Motor driver pin mapping

.. raw:: html

   <div style="overflow-x:auto;">
   <table style="border-collapse:collapse; width:100%; font-size:14px;">
     <tr>
       <th style="border:1px solid #999; padding:8px; text-align:left;">Motor</th>
       <th style="border:1px solid #999; padding:8px; text-align:left;">Signal</th>
       <th style="border:1px solid #999; padding:8px; text-align:left;">NUCLEO Pin</th>
       <th style="border:1px solid #999; padding:8px; text-align:left;">Timer / Channel</th>
     </tr>

     <tr>
       <td style="border:1px solid #999; padding:8px;">Left</td>
       <td style="border:1px solid #999; padding:8px;">Enable</td>
       <td style="border:1px solid #999; padding:8px; background:#cfe8ff;">PB0</td>
       <td style="border:1px solid #999; padding:8px;">Digital output</td>
     </tr>
     <tr>
       <td style="border:1px solid #999; padding:8px;">Left</td>
       <td style="border:1px solid #999; padding:8px;">PWM A</td>
       <td style="border:1px solid #999; padding:8px; background:#fff2cc;">PC12</td>
       <td style="border:1px solid #999; padding:8px;">TIM3 CH3</td>
     </tr>
     <tr>
       <td style="border:1px solid #999; padding:8px;">Left</td>
       <td style="border:1px solid #999; padding:8px;">PWM B</td>
       <td style="border:1px solid #999; padding:8px; background:#fff2cc;">PC10</td>
       <td style="border:1px solid #999; padding:8px;">TIM3 CH3</td>
     </tr>

     <tr>
       <td style="border:1px solid #999; padding:8px;">Right</td>
       <td style="border:1px solid #999; padding:8px;">Enable</td>
       <td style="border:1px solid #999; padding:8px; background:#d9ead3;">PC9</td>
       <td style="border:1px solid #999; padding:8px;">Digital output</td>
     </tr>
     <tr>
       <td style="border:1px solid #999; padding:8px;">Right</td>
       <td style="border:1px solid #999; padding:8px;">PWM A</td>
       <td style="border:1px solid #999; padding:8px; background:#cfe8ff;">PH1</td>
       <td style="border:1px solid #999; padding:8px;">TIM3 CH4</td>
     </tr>
     <tr>
       <td style="border:1px solid #999; padding:8px;">Right</td>
       <td style="border:1px solid #999; padding:8px;">PWM B</td>
       <td style="border:1px solid #999; padding:8px; background:#cfe8ff;">PH0</td>
       <td style="border:1px solid #999; padding:8px;">TIM3 CH4</td>
     </tr>
   </table>
   </div>

.. centered::
   *Motor control wiring using TIM3 PWM channels for effort commands and dedicated enable lines for each motor driver.*


Encoder Connections
-------------------

Quadrature encoder mapping

.. raw:: html

   <div style="overflow-x:auto;">
   <table style="border-collapse:collapse; width:100%; font-size:14px;">
     <tr>
       <th style="border:1px solid #999; padding:8px; text-align:left;">Encoder</th>
       <th style="border:1px solid #999; padding:8px; text-align:left;">Signal</th>
       <th style="border:1px solid #999; padding:8px; text-align:left;">NUCLEO Pin</th>
       <th style="border:1px solid #999; padding:8px; text-align:left;">Timer / Channel</th>
     </tr>
     <tr>
       <td style="border:1px solid #999; padding:8px;">Left</td>
       <td style="border:1px solid #999; padding:8px;">Channel A</td>
       <td style="border:1px solid #999; padding:8px; background:#cfe8ff;">PA0</td>
       <td style="border:1px solid #999; padding:8px;">TIM2 CH1</td>
     </tr>
     <tr>
       <td style="border:1px solid #999; padding:8px;">Left</td>
       <td style="border:1px solid #999; padding:8px;">Channel B</td>
       <td style="border:1px solid #999; padding:8px; background:#fff2cc;">PB3</td>
       <td style="border:1px solid #999; padding:8px;">TIM2 CH2</td>
     </tr>
     <tr>
       <td style="border:1px solid #999; padding:8px;">Right</td>
       <td style="border:1px solid #999; padding:8px;">Channel A</td>
       <td style="border:1px solid #999; padding:8px; background:#cfe8ff;">PA8</td>
       <td style="border:1px solid #999; padding:8px;">TIM1 CH1</td>
     </tr>
     <tr>
       <td style="border:1px solid #999; padding:8px;">Right</td>
       <td style="border:1px solid #999; padding:8px;">Channel B</td>
       <td style="border:1px solid #999; padding:8px; background:#fff2cc;">PA9</td>
       <td style="border:1px solid #999; padding:8px;">TIM1 CH2</td>
     </tr>
   </table>
   </div>

.. centered::
   *Encoder wiring using hardware timer encoder mode to produce wheel position and velocity feedback for control, odometry, and navigation.*


Battery Monitoring
------------------

Battery ADC input

.. raw:: html

   <div style="overflow-x:auto;">
   <table style="border-collapse:collapse; width:60%; font-size:14px;">
     <tr>
       <th style="border:1px solid #999; padding:8px; text-align:left;">Signal</th>
       <th style="border:1px solid #999; padding:8px; text-align:left;">NUCLEO Pin</th>
       <th style="border:1px solid #999; padding:8px; text-align:left;">Notes</th>
     </tr>
     <tr>
       <td style="border:1px solid #999; padding:8px;">Battery ADC</td>
       <td style="border:1px solid #999; padding:8px; background:#cfe8ff;">PC0</td>
       <td style="border:1px solid #999; padding:8px;">ADC input from voltage divider output</td>
     </tr>
   </table>
   </div>

.. centered::
   *Battery voltage measurement routed to an ADC channel for runtime monitoring and diagnostics.*


IR Sensor Array
---------------

Reflectance sensor channel mapping

.. raw:: html

   <div style="overflow-x:auto;">
   <table style="border-collapse:collapse; width:70%; font-size:14px;">
     <tr>
       <th style="border:1px solid #999; padding:8px; text-align:left;">Channel</th>
       <th style="border:1px solid #999; padding:8px; text-align:left;">NUCLEO Pin</th>
       <th style="border:1px solid #999; padding:8px; text-align:left;">Position</th>
     </tr>
     <tr><td style="border:1px solid #999; padding:8px;">1</td><td style="border:1px solid #999; padding:8px; background:#cfe8ff;">PC4</td><td style="border:1px solid #999; padding:8px;">Left far</td></tr>
     <tr><td style="border:1px solid #999; padding:8px;">2</td><td style="border:1px solid #999; padding:8px; background:#fff2cc;">PB1</td><td style="border:1px solid #999; padding:8px;">Left mid</td></tr>
     <tr><td style="border:1px solid #999; padding:8px;">3</td><td style="border:1px solid #999; padding:8px; background:#cfe8ff;">PA7</td><td style="border:1px solid #999; padding:8px;">Left near</td></tr>
     <tr><td style="border:1px solid #999; padding:8px;">4</td><td style="border:1px solid #999; padding:8px; background:#d9ead3;">PC1</td><td style="border:1px solid #999; padding:8px;">Center</td></tr>
     <tr><td style="border:1px solid #999; padding:8px;">5</td><td style="border:1px solid #999; padding:8px; background:#fff2cc;">PA4</td><td style="border:1px solid #999; padding:8px;">Right near</td></tr>
     <tr><td style="border:1px solid #999; padding:8px;">6</td><td style="border:1px solid #999; padding:8px; background:#cfe8ff;">PA1</td><td style="border:1px solid #999; padding:8px;">Right mid</td></tr>
     <tr><td style="border:1px solid #999; padding:8px;">7</td><td style="border:1px solid #999; padding:8px; background:#d9ead3;">PC3</td><td style="border:1px solid #999; padding:8px;">Right far</td></tr>
   </table>
   </div>

.. centered::
   *Seven channel reflectance array wiring used for centroid based line error computation in the line following controller.*


IMU Connections
---------------

BNO055 I2C and reset mapping

.. raw:: html

   <div style="overflow-x:auto;">
   <table style="border-collapse:collapse; width:70%; font-size:14px;">
     <tr>
       <th style="border:1px solid #999; padding:8px; text-align:left;">BNO055 Pin</th>
       <th style="border:1px solid #999; padding:8px; text-align:left;">Wire Color</th>
       <th style="border:1px solid #999; padding:8px; text-align:left;">NUCLEO Pin</th>
       <th style="border:1px solid #999; padding:8px; text-align:left;">Notes</th>
     </tr>
     <tr>
       <td style="border:1px solid #999; padding:8px;">SDA</td>
       <td style="border:1px solid #999; padding:8px;">Purple</td>
       <td style="border:1px solid #999; padding:8px; background:#d9b3ff;">B14</td>
       <td style="border:1px solid #999; padding:8px;">I2C data</td>
     </tr>
     <tr>
       <td style="border:1px solid #999; padding:8px;">SCL</td>
       <td style="border:1px solid #999; padding:8px;">Grey</td>
       <td style="border:1px solid #999; padding:8px; background:#d9d9d9;">B13</td>
       <td style="border:1px solid #999; padding:8px;">I2C clock</td>
     </tr>
     <tr>
       <td style="border:1px solid #999; padding:8px;">RST</td>
       <td style="border:1px solid #999; padding:8px;">White</td>
       <td style="border:1px solid #999; padding:8px;">B15</td>
       <td style="border:1px solid #999; padding:8px;">Hardware reset line</td>
     </tr>
   </table>
   </div>

.. centered::
   *BNO055 wiring over I2C with a dedicated reset pin to support deterministic startup and calibration handling.*


Bump Sensors
------------

Bump sensor wiring and jumpers

.. raw:: html

   <div style="overflow-x:auto;">
   <table style="border-collapse:collapse; width:75%; font-size:14px;">
     <tr>
       <th style="border:1px solid #999; padding:8px; text-align:left;">Side</th>
       <th style="border:1px solid #999; padding:8px; text-align:left;">Board Pin</th>
       <th style="border:1px solid #999; padding:8px; text-align:left;">Wire Color</th>
       <th style="border:1px solid #999; padding:8px; text-align:left;">NUCLEO Pin</th>
     </tr>
     <tr>
       <td style="border:1px solid #999; padding:8px;">Right</td>
       <td style="border:1px solid #999; padding:8px;">BMP 0</td>
       <td style="border:1px solid #999; padding:8px;">Purple</td>
       <td style="border:1px solid #999; padding:8px; background:#d9b3ff;">C11</td>
     </tr>
     <tr>
       <td style="border:1px solid #999; padding:8px;">Right</td>
       <td style="border:1px solid #999; padding:8px;">BMP 1</td>
       <td style="border:1px solid #999; padding:8px;">-</td>
       <td style="border:1px solid #999; padding:8px;">Jumper to 0</td>
     </tr>
     <tr>
       <td style="border:1px solid #999; padding:8px;">Right</td>
       <td style="border:1px solid #999; padding:8px;">BMP 2</td>
       <td style="border:1px solid #999; padding:8px;">-</td>
       <td style="border:1px solid #999; padding:8px;">Jumper to 0</td>
     </tr>
     <tr>
       <td style="border:1px solid #999; padding:8px;">Left</td>
       <td style="border:1px solid #999; padding:8px;">BMP 3</td>
       <td style="border:1px solid #999; padding:8px;">Teal</td>
       <td style="border:1px solid #999; padding:8px; background:#b7f0f0;">Jumper to 2</td>
     </tr>
     <tr>
       <td style="border:1px solid #999; padding:8px;">Left</td>
       <td style="border:1px solid #999; padding:8px;">BMP 4</td>
       <td style="border:1px solid #999; padding:8px;">-</td>
       <td style="border:1px solid #999; padding:8px;">Jumper to 3</td>
     </tr>
     <tr>
       <td style="border:1px solid #999; padding:8px;">Left</td>
       <td style="border:1px solid #999; padding:8px;">BMP 5</td>
       <td style="border:1px solid #999; padding:8px;">-</td>
       <td style="border:1px solid #999; padding:8px;">Jumper to 3</td>
     </tr>
   </table>
   </div>

.. centered::
   *Bump sensor wiring consolidated into a single event input used by navigation for collision detection and recovery sequencing.*


Bluetooth Serial
----------------

UART1 Bluetooth pin mapping

.. raw:: html

   <div style="overflow-x:auto;">
   <table style="border-collapse:collapse; width:60%; font-size:14px;">
     <tr>
       <th style="border:1px solid #999; padding:8px; text-align:left;">Signal</th>
       <th style="border:1px solid #999; padding:8px; text-align:left;">NUCLEO Pin</th>
       <th style="border:1px solid #999; padding:8px; text-align:left;">Notes</th>
     </tr>
     <tr>
       <td style="border:1px solid #999; padding:8px;">TX</td>
       <td style="border:1px solid #999; padding:8px; background:#cfe8ff;">B6</td>
       <td style="border:1px solid #999; padding:8px;">UART1 transmit to Bluetooth RX</td>
     </tr>
     <tr>
       <td style="border:1px solid #999; padding:8px;">RX</td>
       <td style="border:1px solid #999; padding:8px; background:#fff2cc;">B7</td>
       <td style="border:1px solid #999; padding:8px;">UART1 receive from Bluetooth TX</td>
     </tr>
   </table>
   </div>

.. centered::
   *Bluetooth serial link on UART1 used for calibration commands, trial parameters, and runtime telemetry at 460800 baud.*


Full Assembly
-------------

.. image:: /images/romi_full_assembly.jpg
   :width: 900px
   :align: center

.. centered::
   *Fully assembled Romi with integrated wiring harness, controller stack, and sensor mounts configured for obstacle course trials.*
