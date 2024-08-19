Lab 5: How to Train Your Dog
========================

*Goal: Train Pupper to walk using reinforcement learning!*

Step 1. Colab setup
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
#. Make a copy of the Colab notebook `HERE <https://colab.research.google.com/drive/1sZGUI-ivHQdxdR9PosIWi54TKFU3KokG?authuser=1#scrollTo=IbZxYDxzoz5R>`_
#. Create a wandb account (https://wandb.ai/), navigate to User Settings, and generate an API key
#. Set up your wandb key by running cell 2 and inserting your key
#. Run all cells up until Config to install dependencies


Step 2. Pupper stand-up in sim
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
#. Complete the #TODO in the Reward Functions cell to write a reward function to make Pupper stand up
#. Run the ENVIRONMENT and TRAIN cells to load in the Pupper flat environment and train Pupper to stand up
#. Pupper should take around 5-10 minutes to train. 

DELIVERABLE: Visualize Pupper's progress every ~100 episodes. How does Pupper look 100 episodes in? How does this relate to the reward you coded?
DELIVERABLE: Screen recording of stand-up in simulation

Step 3. Deploy Stand High Policy
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. raw:: html

    <div style="position: relative; padding-bottom: 56.25%; height: 0; overflow: hidden; max-width: 100%; height: auto;">
        <iframe src="https://www.youtube.com/embed/Xhj-rCPxm6o" frameborder="0" allowfullscreen style="position: absolute; top: 0; left: 0; width: 100%; height: 100%;"></iframe>
    </div>

|
Transfer policy from local machine to pupper
#. When you train a policy that can standup and stay stable, you are ready to transfer the policy to the physical Pupper robot
#. Navigate to archive in wandb, and download the .json from the policy you trained
#. ‘scp [path_to_model_name.json] pi@pupper.local:’ (note the colon at the end)

#. In local neural_controller repo, change the policy called in policy.py (ros2_ws folder) to your policy name (your .json file)
#. On the pi, run the launch.py script


DELIVERABLE: Take video of stand-up


.. figure:: ../_static/motor_ids.png
    :align: center
    
    Motor ID diagram

Step 4. Walking Policy
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. raw:: html

    <div style="position: relative; padding-bottom: 56.25%; height: 0; overflow: hidden; max-width: 100%; height: auto;">
        <iframe src="https://www.youtube.com/embed/vv3jABUq3ng" frameborder="0" allowfullscreen style="position: absolute; top: 0; left: 0; width: 100%; height: 100%;"></iframe>
    </div>

|

#. Write a reward function that makes Pupper move forward in the PUPPER WALK cell
#. Reload the environment, and train Pupper to walk in sim
#. Pupper should take around 10-15 minutes to train. 

DELIVERABLE: What terms are included in your reward functions? What coefficients did you use? How did you come up with these terms and what was their desired effect? Why might this policy perform poorly on the physical robot?
DELIVERABLE: Visualize Pupper's progress every ~100 episodes. How does Pupper look 100 episodes in? How does this relate to the reward you coded?
DELIVERABLE: Screen recording of stand-up in simulation

Step 5. Reward tuning
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. raw:: html

    <div style="position: relative; padding-bottom: 56.25%; height: 0; overflow: hidden; max-width: 100%; height: auto;">
        <iframe src="https://www.youtube.com/embed/OArwzrKzQdM" frameborder="0" allowfullscreen style="position: absolute; top: 0; left: 0; width: 100%; height: 100%;"></iframe>
    </div>

|

#. Tune the reward so 
#. Deploy on real, works


DELIVERABLE: What terms are included in your reward functions? What coefficeints did you use? How did you come up with these terms and what was their desired effect? Why might this policy perform poorly on the physical robot?
DELIVERABLE: What other terms could you randomize?



.. figure:: ../_static/djipupper_photos/startup-position.png
    :align: center
    
    Startup position.


Step 6. Domain randomization
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. raw:: html

    <div style="position: relative; padding-bottom: 56.25%; height: 0; overflow: hidden; max-width: 100%; height: auto;">
        <iframe src="https://www.youtube.com/embed/OArwzrKzQdM" frameborder="0" allowfullscreen style="position: absolute; top: 0; left: 0; width: 100%; height: 100%;"></iframe>
    </div>

Okay, so Pupper looks pretty good in sim, but the policy doesn't look so great in the real world...

You will need to add randomization to the sim environment so your policy successfully transfers

#. Edit the environment config to adequately represent all the situations Pupper might encounter in the real world


DELIVERABLE: Test your policy during office hours

Resources
-----------

Wiring diagram
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. figure:: ../_static/wiring-diagram.png
    :align: center
    
    Wiring diagram.
