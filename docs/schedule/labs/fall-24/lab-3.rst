Lab 3: Inverse Kinematics and Trajectory Tracking
================================================

Goal
----
Implement inverse kinematics for a robot leg and create a trajectory tracking system using ROS2.

Please also fill out the `lab document <https://docs.google.com/document/d/1X1UOZr6DPuhhVHxnpaHo7VfXr0YNnqKPDF-i4rvzxN8/edit?usp=sharing>`_ .

.. raw:: html

    <div style="position: relative; padding-bottom: 56.25%; height: 0; overflow: hidden; max-width: 100%; height: auto;">
        <iframe src="https://www.youtube.com/embed/ygwWZw50yw0" frameborder="0" allowfullscreen style="position: absolute; top: 0; left: 0; width: 100%; height: 100%;"></iframe>
    </div>
|

Part 0: Setup
-------------

1. Clone the lab 3 code repository on your Raspberry Pi:

   .. code-block:: bash

      cd ~/
      git clone https://github.com/cs123-stanford/lab_3_2024.git lab_3

   Note: Ensure the folder name is ``lab_3``. If different, update the launch file accordingly.

2. Open the workspace in VSCode.

3. Examine ``lab_3.py`` to understand the structure of the ``InverseKinematics`` class and its methods.

4. Change all the 12 occurances of homing_kp values to 5.5 and homing_kd values to 0.2 in the ~/ros2_ws/src/pupper_v3_description/description/components.xacro file to undo the changes from lab 2.

Part 1: Forward Kinematics (already done in lab 2)
------------------------------------

1. Open ``lab_3.py`` and locate the ``forward_kinematics`` method in the ``InverseKinematics`` class.

- Paste code from lab 2
- Review the compositions of the transformation matrices T_0_1, T_1_2, T_2_3, and T_3_ee.
- Review the final transformation matrix T_0_ee.
- Review the end-effector position (the first 3 elements of the last column of T_0_ee).

Part 2: Implement Inverse Kinematics
------------------------------------

1. Find the ``inverse_kinematics`` method in the ``InverseKinematics`` class.

**TODO 1:** Implement the cost function for inverse kinematics.

- Use the ``forward_kinematics`` method to get the current end-effector position.
- Calculate the L1 distance between the current and target end-effector positions.
- Return the sum of squared L1 distances as the cost (AKA the squared 2-norm of the error vector).

**TODO 2:** Implement the gradient function for inverse kinematics.

- Calculate the numerical gradient of the cost function with respect to each joint angle.
- Use a small epsilon value (e.g., 1e-3) for the finite difference approximation.

**TODO 3:** Implement the gradient descent algorithm for inverse kinematics.

- Use the provided learning rate and maximum iterations.
- Update the joint angles using the calculated gradient.
- Stop the iteration if the mean L1 distance is below the tolerance.
- Bonus: Implement the (quasi-)Newton's method for faster convergence.

Part 3: Implement Trajectory Generation
---------------------------------------

1. Locate the ``interpolate_triangle`` method in the ``InverseKinematics`` class.

**TODO 4:** Implement the interpolation for the triangular trajectory.

- Use the provided ``ee_triangle_positions``, which define the 3 vertices of the triangle trajectory.
- Implement linear interpolation between the triangle vertices based on the input time ``t``.
- Ensure the trajectory loops every ~2 seconds approximately.

Part 4: Implement PD Control (already done in lab 1)
----------------------------

1. Find the ``pd_timer_callback`` method in the ``InverseKinematics`` class.

- Paste code from lab 1
- Review the position error (difference between target and current joint positions).
- Review the velocity error (assuming target velocity is zero).
- Review computation of the torque command using the PD control law: τ = Kp * position_error - Kd * velocity_error.

Part 5: Run and Test Your Implementation
----------------------------------------

1. Run the launch file using the following command:

   .. code-block:: bash

      ros2 launch lab_3 lab_3.launch.py

2. Observe the robot leg's movement and the terminal output.

3. Experiment with different trajectory shapes by modifying the ``ee_triangle_positions`` in the ``__init__`` method.

**DELIVERABLE:** Take a video of the robot leg tracking the triangular trajectory and submit it with your submission.

Part 6: Analyze and Improve Performance
---------------------------------------

1. Experiment with different values for ``Kp`` and ``Kd`` in the PD controller.

2. Modify the ``ik_timer_period`` and ``pd_timer_period`` to see how they affect the system's performance.

3. Try different initial guesses for the inverse kinematics algorithm and observe the convergence behavior.

**DELIVERABLE:** In your lab document, report on:

- The effects of changing ``Kp`` and ``Kd`` values
- How different timer periods affect the system's behavior
- The impact of initial guesses on the inverse kinematics convergence

Additional Notes
----------------

- The ``inverse_kinematics`` method uses gradient descent. Ensure you understand how the cost function and gradient are calculated.
- The ``interpolate_triangle`` method should create a continuous trajectory between the defined triangle points.

Congratulations on completing Lab 3! This hands-on experience with inverse kinematics and trajectory control will be crucial for more advanced robot control tasks in future labs.
