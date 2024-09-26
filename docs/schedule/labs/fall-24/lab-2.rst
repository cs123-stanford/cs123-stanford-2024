Lab 2: Forward Kinematics
=========================

Goal
----
Implement forward kinematics for the right front leg of the Pupper robot using ROS2 and Python.

Fill out the `lab document <https://docs.google.com/document/d/1uAoTIHvAqEqXTPVWyHrLkuw0ZJ24BPCPn_Q6XIztvR0/edit?usp=sharing>`_ as you go.

Part 0: Setup
-------------

1. Make sure you have completed Lab 1 and are familiar with the ROS2 environment on your Raspberry Pi 5.

2. Clone the lab 2 code repository on the Raspberry Pi:

   .. code-block:: bash

      cd ~/
      git clone https://github.com/cs123-stanford/lab_2_2024.git lab_2

3. Open the workspace in VSCode and examine the ``lab_2.py`` file.

Part 1: Understanding the Code Structure
----------------------------------------

Before we start implementing the TODOs, let's understand the structure of the ``lab_2.py`` file:

1. The code defines a ``ForwardKinematics`` class that inherits from ``rclpy.node.Node``.
2. It subscribes to the ``joint_states`` topic and publishes to the ``leg_front_r_end_effector_position`` topic.
3. The ``forward_kinematics`` method is where we'll implement the forward kinematics calculations.
4. The code uses NumPy for matrix operations.
5. Note that it is convention to orient the coordinate frame so that the rotation about each motor is the z axis.

Part 2: Implementing Forward Kinematics
---------------------------------------

Step 1: Implement Rotation Matrices
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1. Open ``lab_2.py`` and locate the ``forward_kinematics`` method.

2. Implement the rotation matrix about the y-axis:

   .. code-block:: python

      def rotation_y(angle):
          return np.array([
              [x, x, x, 0],
              [x, x, x, 0],
              [x, x, x, 0],
              [0, 0, 0, 1]
          ])

3. Implement the rotation matrix about the z-axis:

   .. code-block:: python

      def rotation_z(angle):
          return np.array([
              [x, x, x, 0],
              [x, x, x, 0],
              [x, x, x, 0],
              [0, 0, 0, 1]
          ])

Step 2: Implement Transformation Matrices
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Note that theta is the motor angle

1. Implement the transformation matrix from leg_front_r_1 to leg_front_r_2. This involves a rotation about y, then another rotation. What axis will you need to rotate about?

   .. code-block:: python

      T_1_2 = rotation_y(-1.57080) @ TODO

2. Implement the transformation matrix from leg_front_r_2 to leg_front_r_3. This will invole a translation (to move to coordinate of leg_front_r_3), then two rotations. What axes will you rotate about (and in what order / how much)?

   .. code-block:: python

      T_2_3 = translation(TODO) @ TODO @ TODO

   Note: The translation values may need to be adjusted based on the actual dimensions of your robot. Make sure to verify these values with your robot's specifications.

Part 3: Testing Your Implementation
-----------------------------------

1. Save your changes to ``lab_2.py``.

2. Run the ROS2 node:

   .. code-block:: bash

      ros2 launch lab_2.launch.py

3. In another terminal, use the following command to run the main code:

   .. code-block:: bash

      python lab_2.py

4. Move the right front leg of your robot and observe the changes in the published positions.

To test your code in simulation to make sure that the code works as expected, you can use RVIZ. RVIZ will show the Pupper model as well as a marker that shows the output from the forward kinematics.

   .. code-block:: bash

      rviz2 -d lab_2.rviz

The above command will load the RVIZ config file. If you just run ``rviz``, you can manually add the configuration. After running `rviz`, click the "Add" button, and then select a Robot Model type. Select the \robot_description topic. Next, add the marker by selecting "Add" again, and select a Marker type. Select the joint

Part 4: Analyzing the Results
-----------------------------

1. Record the end-effector positions for at the front right leg configurations.

2. Compare these positions with the expected positions based on the physical dimensions of your robot.

3. If there are discrepancies, try to identify the source of the errors. It could be due to:
   
   - Incorrect transformation matrices
   - Inaccurate joint angle readings
   - Errors in the physical measurements of the robot

Deliverables
------------

1. Submit your completed ``lab_2.py`` file.

2. Provide a brief report (maximum 1 page) that includes:
   
   - A description of your implementation process
   - The recorded end-effector positions for the front right leg configurations
   - An analysis of the accuracy of your forward kinematics implementation
   - Any challenges you faced and how you overcame them

Additional Challenges (Optional)
--------------------------------

If you finish early or want to explore further:

1. Extend your implementation to calculate forward kinematics for all four legs of the Pupper robot.
2. Create a visualization of the leg's end-effector position using RViz or another visualization tool.

Remember, understanding forward kinematics is crucial for robot control and motion planning. Take your time to ensure you understand each step of the process.
