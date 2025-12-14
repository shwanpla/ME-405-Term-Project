Analysis
========

Bluetooth Telemetry and Line Following Breakthrough
===================================================

A key capability that accelerated our progress was real time introspection of the running firmware. After integrating the Bluetooth module, we streamed structured REPL output to a PC using PuTTY while Romi executed its cooperative tasks. This gave us continuous visibility into internal variables such as encoder velocity, PI error, integrator state, line centroid, saturation status, and scheduler state flags. With that visibility, each trial became a measurable experiment. We could confirm timing behavior, detect mode transitions, validate that calibration was applied correctly, and save full text logs for offline plotting and direct comparison across runs.

From there, we focused on the most control intensive milestone of the term: stable line following with the 7 channel Pololu QTR array. The program treats line tracking as a closed loop regulation problem driven by a centroid measurement computed from the reflectance profile. Each sample produces a normalized set of sensor intensities, a centroid location across the array, and a signed lateral error relative to the center sensor. That error is then converted into a steering correction which is injected into the motor effort commands while the underlying velocity loop maintains repeatable forward speed.

Control and sensing were implemented explicitly in the code using the following sequence of computations.

IR calibration and normalization
-------------------------------

Calibration establishes two reference measurements for each channel, one for white background and one for black tape. During runtime, each raw reading is mapped into a normalized intensity so the centroid calculation remains consistent across lighting variation and across repeated trials.

.. math::

   I_i = \mathrm{clip}\!\left(\frac{R_i - W_i}{B_i - W_i},\,0,\,1\right)

where :math:`R_i` is the raw sensor reading for channel :math:`i`, :math:`W_i` is the white reference, :math:`B_i` is the black reference, and :math:`I_i` is the normalized intensity used for line tracking.

Centroid and line error
-----------------------

The centroid represents the weighted location of the detected line across the array. With seven sensors indexed left to right, the centroid is computed as:

.. math::

   c = \frac{\sum_{i=1}^{7} p_i I_i}{\sum_{i=1}^{7} I_i}

where :math:`p_i` are the sensor position weights. The signed line error is then formed relative to the desired center position :math:`c_0`:

.. math::

   e_\mathrm{line} = c - c_0

This single scalar error gives the controller a continuous measure of how far the line is from the center of the array, with sign indicating which direction the robot must steer to re center.

Velocity PI control and steering injection
------------------------------------------

Each wheel is regulated using a PI velocity controller driven by encoder speed feedback. The core loop follows:

.. math::

   e_v(t) = v_\mathrm{ref}(t) - v(t)

.. math::

   u_\mathrm{PI}(t) = K_p e_v(t) + K_i \int e_v(t)\,dt

The line following correction is implemented as an additive differential term that biases left and right effort in opposite directions:

.. math::

   \Delta u_\mathrm{line}(t) = K_\ell e_\mathrm{line}(t) + K_{\ell i}\int e_\mathrm{line}(t)\,dt

.. math::

   u_L(t) = \mathrm{sat}\!\left(u_\mathrm{PI}(t) - \Delta u_\mathrm{line}(t)\right), \quad
   u_R(t) = \mathrm{sat}\!\left(u_\mathrm{PI}(t) + \Delta u_\mathrm{line}(t)\right)

where :math:`u_L` and :math:`u_R` are the left and right motor effort commands and :math:`\mathrm{sat}(\cdot)` enforces the allowable effort range. This structure makes the behavior interpretable during tuning because the forward speed is controlled by the velocity loop while lateral correction is controlled by the centroid error terms.

Integrator management and repeatability
---------------------------------------

To maintain stability across aggressive maneuvers, the program tracks saturation and constrains integral accumulation when the output is pinned at its limit. This prevents integrator growth from dominating the response after the robot reacquires the line. The same philosophy is applied to the line integral term so that line correction remains smooth rather than accumulating a large bias during temporary loss of contrast.

Tuning workflow and physical validation
---------------------------------------

At this phase of the term, most iteration time was spent converting these computations into reliable physical performance. We tuned :math:`K_p` and :math:`K_i` for stable speed tracking, then tuned :math:`K_\ell` and :math:`K_{\ell i}` so the robot continuously re centered on the tape without oscillation, overshoot, or drift. Each trial around the black circle was evaluated using the telemetry stream and direct observation. We looked for the same signatures in the data every run: bounded line error, stable encoder speed, limited saturation time, and consistent centroid convergence after disturbances. Once the robot could follow the circle repeatedly, we had a validated sensing and control foundation that supported the later navigation logic used on the obstacle course.

.. image:: /images/team_line_follow_tuning.png
   :width: 900px
   :align: center

.. centered::
   *Our team during the line following tuning phase, using live PuTTY telemetry and repeated trials to refine calibration, gains, and stability margins.*

.. raw:: html

   <div style="text-align:center; margin-top: 10px; margin-bottom: 10px;">
     <iframe width="720" height="405"
             src="https://www.youtube.com/embed/3hr3dCyPbDM"
             title="Initial Line Following Breakthrough"
             frameborder="0"
             allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
             allowfullscreen>
     </iframe>
   </div>

.. centered::
   *First stable line following breakthrough demonstrating centroid based tracking with PI velocity regulation and continuous steering correction.*
