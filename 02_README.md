This code balances the Cassie biped robot on one leg in MuJoCo using a Linear Quadratic Regulator (LQR) controller. The implementation addresses the challenge of achieving single-leg balance for autonomous robots, similar to this [code](https://colab.research.google.com/github/deepmind/mujoco/blob/main/python/LQR.ipynb) developed to balance a 27 DoF humanoid on one leg. The model used can be found [here](https://github.com/google-deepmind/mujoco_menagerie/blob/main/agility_cassie/README.md): 

<div align="center">
    <img src="https://github.com/user-attachments/assets/0035d3b9-217a-40bf-9487-f7a29fa71cc9" alt="00_1leg_photo" style="width:40%;">
</div>

You can download prebuilt binaries for MuJoCo from the GitHub [releases page](https://github.com/google-deepmind/mujoco/releases/). Alternatively, if you are working with Python, as I have, you can install the native bindings from PyPI as demonstrated below.

### Python (>= 3.9)

The native Python bindings, which come pre-packaged with a copy of MuJoCo, can be installed from [PyPI](https://pypi.org/project/mujoco/) via:

```bash
pip install mujoco
```bash


For alternative installation instructions, see [here](https://github.com/google-deepmind/mujoco#installation).

The controller's Q and R cost matrices have been fine-tuned within a specific operating region using the MuJoCo simulator (see photos below for reference).

I have modified the XML file to include the perfect positioning of the robot on one-leg (and one more keyframe wherein a ~1.5cm height transition works). 

