## One-leg LQR Controller Implementation for the Cassie Robot

This code implements a Linear Quadratic Regulator (LQR) controller to balance the Cassie robot on one leg in MuJoCo. The original MJCF model and assets were provided by [Agility Robotics](https://agilityrobotics.com/) under an [MIT License](https://github.com/google-deepmind/mujoco_menagerie/blob/main/agility_cassie/LICENSE). The implementation is inspired by [this tutorial](https://colab.research.google.com/github/deepmind/mujoco/blob/main/python/LQR.ipynb), which balances a 27 DoF humanoid on one leg. The Cassie model used in this code can be found [here](https://github.com/google-deepmind/mujoco_menagerie/blob/main/agility_cassie/README.md).


<div align="center">
    <img src="https://github.com/user-attachments/assets/0035d3b9-217a-40bf-9487-f7a29fa71cc9" alt="00_1leg_photo" style="width:30%;">
</div>

---

### MuJoCo Bindings

You can download prebuilt binaries for MuJoCo from the GitHub [releases page](https://github.com/google-deepmind/mujoco/releases/). Alternatively, if you are working with Python, as I have, you can install the native bindings from PyPI as demonstrated below.


#### Python (>= 3.9)

The native Python bindings, which come pre-packaged with a copy of MuJoCo, can be installed from [PyPI](https://pypi.org/project/mujoco/) via:

```
pip install mujoco
```

For alternative installation instructions, see [here](https://github.com/google-deepmind/mujoco#installation).

---

### Running Script

To run the script, you will first need to make sure `numpy` and `matplotlib` have been installed:

```
pip install numpy
pip install matplotlib
```

You will also need to ensure the Python script is in the desired directory, alongside the XML file. To execute the code, run:

```
mjpython DIRECTORY/LQR_1leg.py
```

### Keyframe Transition

I have modified the XML file to include the precise positioning of the robot on one leg, along with an additional keyframe that incorporates a ~1.5 cm height transition. To enable or disable this keyframe transition feature, simply adjust the toggle at the top of the Python script. Additionally, you can toggle the application of forces in any direction to the pelvis on or off as needed (line ~613).

---

### Operating Region

The controller's Q and R cost matrices have been fine-tuned to enable the robot to balance effectively across a wider operating region in the MuJoCo simulator. This region is visualised in the photos below. To explore this operating region yourself, refer to the data indexing section for guidance on which elements to modify in the keyframe embedded within the XML file.

<div align="center">
    <img src="./07_Visualisation%20of%20Maximum%20Operating%20Margin%20in%20the%20X%20and%20Z%20Directions.png" alt="Visualization" style="width:80%;">
</div>


<div align="center">
    <img src="./08_Visualisation%20of%20Maximum%20Operating%20Margin%20in%20the%20Y%20Direction.png" alt="Visualization" style="width:60%;">
</div>

---

### Data Indexing

To map the elements in the keyframe within the XML file to their respective position and velocity indices, the data indexing is provided below.

As per `cassiemujoco.h`, the order of states in `d.qpos` is as follows:

    [ 0] Pelvis x
    [ 1] Pelvis y
    [ 2] Pelvis z
    [ 3] Pelvis orientation qw
    [ 4] Pelvis orientation qx
    [ 5] Pelvis orientation qy
    [ 6] Pelvis orientation qz
    [ 7] Left hip roll         (Motor [0])
    [ 8] Left hip yaw          (Motor [1])
    [ 9] Left hip pitch        (Motor [2])
    [10] Left achilles rod qw
    [11] Left achilles rod qx
    [12] Left achilles rod qy
    [13] Left achilles rod qz
    [14] Left knee             (Motor [3])
    [15] Left shin                        (Joint [0])
    [16] Left tarsus                      (Joint [1])
    [17] Left heel spring
    [18] Left foot crank
    [19] Left plantar rod
    [20] Left foot             (Motor [4], Joint [2])
    [21] Right hip roll        (Motor [5])
    [22] Right hip yaw         (Motor [6])
    [23] Right hip pitch       (Motor [7])
    [24] Right achilles rod qw
    [25] Right achilles rod qx
    [26] Right achilles rod qy
    [27] Right achilles rod qz
    [28] Right knee            (Motor [8])          
    [29] Right shin                       (Joint [3])
    [30] Right tarsus                     (Joint [4])
    [31] Right heel spring
    [32] Right foot crank
    [33] Right plantar rod
    [34] Right foot            (Motor [9], Joint [5])

As for the velocities in `d.qvel`, the order is:

    [ 0] Pelvis x
    [ 1] Pelvis y
    [ 2] Pelvis z
    [ 3] Pelvis orientation wx
    [ 4] Pelvis orientation wy
    [ 5] Pelvis orientation wz
    [ 6] Left hip roll         (Motor [0])
    [ 7] Left hip yaw          (Motor [1])
    [ 8] Left hip pitch        (Motor [2])
    [ 9] Left achilles rod wx
    [10] Left achilles rod wy
    [11] Left achilles rod wz
    [12] Left knee             (Motor [3])
    [13] Left shin                        (Joint [0])
    [14] Left tarsus                      (Joint [1])
    [15] Left heel spring
    [16] Left foot crank
    [17] Left plantar rod
    [18] Left foot             (Motor [4], Joint [2])
    [19] Right hip roll        (Motor [5])
    [20] Right hip yaw         (Motor [6])
    [21] Right hip pitch       (Motor [7])
    [22] Right achilles rod wx
    [23] Right achilles rod wy
    [24] Right achilles rod wz
    [25] Right knee            (Motor [8])
    [26] Right shin                       (Joint [3])
    [27] Right tarsus                     (Joint [4])
    [28] Right heel spring
    [29] Right foot crank
    [30] Right plantar rod
    [31] Right foot            (Motor [9], Joint [5])


### The Joints

The configuration vector `q` includes all the joint angles (rad) in the order below. Nominal joint values and joint limits are included (in deg, from `cassie.xml` file). **NOTE:** Agility documentation and `cassie.xml` files have different values for joint limits. Check later for consistency! 
- hip abduction         ()  [-15, 22.5]
- hip yaw               ()  [-22.5, 22.5]
- hip pitch             ()  [-50, 80]
- knee joint            ()  [-164, -37]
- tarsus (passive)      ()  [50, 170]
- toe joint             ()  [-140, -30]

---

**Disclaimer:** All analysis and code detailed above was developed by William Lavender as part of an engineering honours thesis, completed in 2024 at The University of Sydney.
