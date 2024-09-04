Lab 3: Romeo and Juliet
=======================================================

**Goals:**
        1. Learn how to compute inverse kinematics 
        2. Use FK+IK to create a mirroring setup

.. figure:: ../../../_static/lab3.jpg
    :align: center
    :width: 50%

Step 1. Code inverse kinematics
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
#. Clone the starter code:

   .. code-block:: bash

      cd ~/
      git clone https://github.com/cs123-stanford/lab_3_2024.git lab_3

#. Implement ``cost`` in ``lab_3.py`` as the squared-norm of the error between the position returned by ``FK(guess)`` and ``target_location``. 
#. Implement ``calculate_gradient`` in ``lab_3.py``
#. Implement ``inverse_kinematics`` in ``lab_3.py``

Step 2. Test the consistency between forward kinematics and inverse kinematics
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
#. Look at the tests written for you in ``test_ik`` within ``test_inv_kinematics.h``
#. The test works by taking some reachable (x,y,z) point in space, and calling your IK function to get the corresponding joint angles, then passing them to your FK function to retrieve the original (x,yz).
#. Make sure the test passes before moving on

**Deliverable: Write about why we are doing an IK -> FK consistency test rather than an FK -> IK test (2-3 sentences). Hint: Think about the robot leg configuration(s)**

**Deliverable: Why is it important that the point we are testing is reachable? Describe what you expect IK to return for this case?**

Step 4. Put it together! Make your two robot arms match each other's end-effector positions
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
1. TODO: Implement walking by tracing a triangle

**Deliverable: Send a video of the arms roughly matching each other when you move them**

2. Try more iterations of IK in ``lab_3.py``, and observe the behavior **(Careful may be unstable)**

**Deliverable: Why does more iterations of IK cause instability?**
