Overview
========
In ME 405 Mechatronics, our term project culminated in a fully integrated autonomous two wheeled differential drive robot named Romi, engineered for robust, repeatable obstacle course navigation. Our three person team delivered a complete mechatronic system that tightly couples embedded hardware, real time MicroPython firmware, task based scheduling, multi sensor instrumentation, and closed loop control with state estimation methods rooted in state space modeling and pole placement. The result is an autonomous robot system that demonstrates end to end capability in designing, integrating, and validating an embedded robotic solution, spanning electrical fabrication and mechanical sensor mounting through real time software architecture and navigation level decision logic.

.. image:: /images/romi_isometric.png
   :width: 600px
   :align: center

.. centered::
   *Romi isometric view highlighting the integrated chassis layout and sensor mounting geometry used for autonomous navigation.*

Our development process emphasized modularity and verification at every layer. We implemented reusable drivers for motor actuation, encoder measurement, timed ADC acquisition, I2C based IMU communication, and serial and Bluetooth connectivity, then composed higher level behaviors using generator based tasks and shared data interfaces under a cooperative scheduler. This structure enabled deterministic update rates, clean separation between sensing, control, and decision making, and an efficient tuning workflow based on repeatable trials and telemetry.

Our software stack was designed to be modular, readable, and scalable from early bring up through final navigation. We implemented structured MicroPython code using functions for reusable logic, classes for hardware drivers and interfaces, and generator based tasks to enforce periodic execution of each subsystem. Each task exposed a clear input output boundary through shared variables, allowing sensing, estimation, control, and navigation to run concurrently with deterministic timing under a cooperative scheduler. This architecture supported incremental integration and debugging, made it straightforward to tune control gains and thresholds using telemetry, and allowed the navigation layer to remain clean by commanding behavior through well defined subsystem tasks rather than direct low level hardware calls.

Our hardware work focused on building a reliable, serviceable robot suitable for repeated trials. We produced wiring harnesses with custom crimps and headers, maintained clean routing, and completed soldered circuitry to support stable analog measurement and battery monitoring. Mechanical integration was reinforced with CAD and 3D printed fixtures that controlled sensor geometry, including mounts that set the IR sensor array height relative to the ground to improve reflectance contrast and measurement consistency.

The culminating deliverable of the term was a navigation task built specifically for a unique obstacle course with distinct features and decision points. The final live demonstration was traversing the entire course across multiple trials, which required Romi to seamlessly switch between behaviors such as line following, straight segments, distance based motion, controlled turns, and bump response while maintaining stable closed loop motor control underneath. By combining calibrated sensing and state information with structured decision logic, our navigation layer executed repeatable maneuver commands and made consistent path choices through the course obstacles rather than relying on a one off run. Together, these pages document the final robot and codebase and provide evidence of our capability to build a multi skilled autonomous system through disciplined embedded software, controls, and full stack integration.

.. image:: /images/Obstacle_Course.png
   :width: 900px
   :align: center

.. centered::
   *Obstacle course top view used for time trials. The run begins at CP1 and proceeds through the numbered checkpoints in order. Dashed circles indicate optional bonus checkpoints worth minus 3 seconds each on the final runtime.*
