This code balances the Cassie biped robot on one leg in MuJoCo using a Linear Quadratic Regulator (LQR) controller. The implementation addresses the challenge of achieving single-leg balance for autonomous robots. The controller's Q and R cost matrices have been fine-tuned within a specific operating region using the MuJoCo simulator, providing insights into the feasibility of LQR control for robotic balance and locomotion.

The model used can be found [here](https://github.com/google-deepmind/mujoco_menagerie/blob/main/agility_cassie/README.md): 

You can download prebuilt binaries for MuJoCo from the GitHub [releases page](https://github.com/google-deepmind/mujoco/releases/). Alternatively, if you are working with Python, as I have, you can install the native bindings from PyPI via pip install MuJoCo. For alternative installation instructions, see [here](https://github.com/google-deepmind/mujoco#installation).

I have modified the XML file to include the perfect positioning of the robot on one-leg (and one more keyframe wherein a ~1.5cm height transition works).

<img src="https://github.com/user-attachments/assets/0035d3b9-217a-40bf-9487-f7a29fa71cc9" alt="00_1leg_photo" style="width:40%;">
