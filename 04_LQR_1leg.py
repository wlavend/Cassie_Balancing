#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Oct 28 22:02:13 2024

@author: williamlavender
"""


import os
import mujoco
import imageio
import numpy as np
import scipy 
import time
from scipy.io import savemat


DURATION = 2            # seconds
FRAMERATE = 60          # Hz (in XML, timestep is set to 0.0005 because the 
                        # standard Cassie controller runs at 2 kHz)

toggle = 0              # Toggle to run LQR only or run full script 
                        # (default = 0) i.e. LQR only

keyframe = 0            # Choose keyframe from XML file (default = 0)
transition_toggle = 0   # Transition between positions (default = 0) i.e off
key_target = 0

                        # To analyse the impact of a force turn force toggle 
                        # on (line ~613) and change force array

class Cassie_LQR:

    def __init__(Cassie):
        
        # Define the path to your cassie.xml file
        Cassie.model_path = '/Users/williamlavender/Desktop/Cassie/Cassie/scene.xml'
        
        # Load the MuJoCo model from XML file
        Cassie.model = mujoco.MjModel.from_xml_path(Cassie.model_path)
        Cassie.data = mujoco.MjData(Cassie.model)  # State and quantities
        
        # Renderer for visualization
        Cassie.renderer = mujoco.Renderer(Cassie.model, width=1920, height=1088)
        
        # Initialise to desired pose
        mujoco.mj_resetDataKeyframe(Cassie.model, Cassie.data, keyframe)
        mujoco.mj_forward(Cassie.model, Cassie.data)
        
        # Specify output directory
        Cassie.output_dir = os.path.expanduser('~/Desktop/THESIS/Code/One_Leg_LQR')
        Cassie.output_dir1 = os.path.expanduser('~/Desktop/THESIS/Code/One_Leg_LQR/1.1- No Control')
        Cassie.output_dir2 = os.path.expanduser('~/Desktop/THESIS/Code/One_Leg_LQR/1.2- Initial Control')
        Cassie.output_dir3 = os.path.expanduser('~/Desktop/THESIS/Code/One_Leg_LQR/1.3- Full Control')

        # Make new camera and set distance for visualization
        Cassie.camera = mujoco.MjvCamera()
        mujoco.mjv_defaultFreeCamera(Cassie.model, Cassie.camera)
        Cassie.camera.distance = 2.3
        
        # Enable visualization of contact forces
        Cassie.scene_option = mujoco.MjvOption()
        Cassie.scene_option.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = True
        
    def print_info(Cassie):
        
        print("\n\n------------------------KEY INFORMATION------------------------\n\n")

        # Define the width for labels to ensure consistent alignment
        label_width = 25  # Adjust this value to fit your longest label

        # Print the model file you are using
        print(f"{'Model File:'.ljust(label_width)} {Cassie.model_path}\n")

        # Print mass and weight (using Cassie.model)
        mass = round(Cassie.model.body_subtreemass[1], 2)
        print(f"{'Mass:'.ljust(label_width)} {mass}  KG")
        
        weight = mass * np.linalg.norm(Cassie.model.opt.gravity)
        print(f"{'Weight:'.ljust(label_width)} {round(weight, 2)} N\n")

        # Print the number of body parts and their names
        body_names = [Cassie.model.names[i] for i in range(Cassie.model.nbody)]

        print(f"{'Number of Body Parts:'.ljust(label_width)} {len(body_names)}")
        print(f"{'Number of Joints:'.ljust(label_width)} {Cassie.model.njnt}")

        # Number of Degrees of Freedom (DoF)
        nv = Cassie.model.nv  # Number of degrees of freedom
        print(f"{'Number of DoF (nv):'.ljust(label_width)} {nv}")

        # Number of Actuators (nu)
        nu = Cassie.model.nu  # Number of actuators
        print(f"{'Number of Actuators (nu):'.ljust(label_width)} {nu}\n")
        
        # Print the body parts
        print(f"Body Parts:")
        for idx, name in enumerate(body_names):
            print(f"    {idx:02d}: {name}")  # Format the index as a 2-digit number

    def setup_no_control(Cassie):
    
        for filename in os.listdir(Cassie.output_dir1): # Clear out images 
            file_path = os.path.join(Cassie.output_dir1, filename)
            try:
                if os.path.isfile(file_path):
                    os.unlink(file_path)
            except Exception as e:
                print(f'Error deleting file {file_path}: {e}')
    
        # Create the directory if it doesn't exist
        if not os.path.exists(Cassie.output_dir1):
            os.makedirs(Cassie.output_dir1)
            
    def run_no_control(Cassie):
        # Initialize list to store frames
        frames1 = []
    
        # Run the simulation loop until the time reaches the defined duration
        while Cassie.data.time < DURATION:
            # Step the simulation.
            mujoco.mj_step(Cassie.model, Cassie.data)
    
            # Render and save frames.
            if len(frames1) < Cassie.data.time * FRAMERATE:
                # Set the lookat point to the humanoid's center of mass.
                Cassie.camera.lookat = Cassie.data.body('cassie-pelvis').subtree_com
                Cassie.renderer.update_scene(Cassie.data, Cassie.camera)
    
                # Render the current frame
                frame = Cassie.renderer.render()
    
                # Save each frame to a file in the specified directory
                frame_filename = os.path.join(Cassie.output_dir1, f'{len(frames1):04d}.png')
                imageio.imwrite(frame_filename, frame)
    
                frames1.append(frame)
                
            
        # Reset model to original key frame position
        mujoco.mj_resetDataKeyframe(Cassie.model, Cassie.data, keyframe)
    
        return frames1    
    
    def forces_initial_control(Cassie):
        print("\n\n------------------------INITIAL CONTROL------------------------\n\n")
        
        # Use control setpoint from seperate script
        ctrl0 = np.array([0.2, 0.0109, 0.1526, -0.2822, -0.0000, -1.8239, -0.2733, -0.8596, 6.1414, -0.4978])
        
        Cassie.data.ctrl = ctrl0
        mujoco.mj_forward(Cassie.model, Cassie.data)
    
        qpos0 = Cassie.data.qpos.copy()
    
        # Set the state and controls to their setpoints
        mujoco.mj_resetData(Cassie.model, Cassie.data)
        Cassie.data.qpos = qpos0
        Cassie.data.ctrl = ctrl0
        
        return qpos0, ctrl0

    def height_offset(Cassie):
        # Look at difference in the 'Forces Required' (third force). We are
        # searching for the height at which the robot is in perfect balance— 
        # where the vertical force required is minimal, meaning that the robot
        # can be held up mostly by its own structure, rather than needing 
        # external help.
        
        height_offsets = np.linspace(-0.01, 0.01, 20001)  
        vertical_forces = []
        
        for offset in height_offsets:
            mujoco.mj_resetDataKeyframe(Cassie.model, Cassie.data, keyframe)
            mujoco.mj_forward(Cassie.model, Cassie.data)
            Cassie.data.qacc = 0
            
            # Offset the height by `offset`.
            Cassie.data.qpos[2] += offset
            mujoco.mj_inverse(Cassie.model, Cassie.data)
            
            # Store the vertical force (third component of qfrc_inverse)
            vertical_forces.append(Cassie.data.qfrc_inverse[2])
            
        # Find the height-offset at which the vertical force is smallest.
        idx = np.argmin(np.abs(vertical_forces))
        best_offset = height_offsets[idx]
                
        return best_offset, vertical_forces, height_offsets

    def new_control(Cassie, best_offset):
        # Reset the model to the keyframe
        mujoco.mj_resetDataKeyframe(Cassie.model, Cassie.data, keyframe)
        
        # Apply the height offset
        Cassie.data.qpos[2] += best_offset
        
        # Zero out qacc for all joints except the pelvis (indices 0-5)
        Cassie.data.qacc = 0
        
        # Save the position setpoint
        qpos0 = Cassie.data.qpos.copy()
    
        # Perform inverse dynamics
        mujoco.mj_inverse(Cassie.model, Cassie.data)
        qfrc0 = Cassie.data.qfrc_inverse.copy()      
        
        # Use control setpoint from seperate script
        ctrl0 = np.array([0.2, 0.0109, 0.1526, -0.2822, -0.0000, -1.8239, -0.2733, -0.8596, 6.1414, -0.4978])

        # Apply the control setpoint and move forward the simulation
        Cassie.data.ctrl = ctrl0
        mujoco.mj_forward(Cassie.model, Cassie.data)
    
        # Return the setpoints
        return qpos0, ctrl0, qfrc0

    def setup_initial_control(Cassie):
        
        # Clear out the images in the folder before adding new ones
        for filename in os.listdir(Cassie.output_dir2):  
            file_path = os.path.join(Cassie.output_dir2, filename)
            try:
                if os.path.isfile(file_path):
                    os.unlink(file_path)
            except Exception as e:
                print(f'Error deleting file {file_path}: {e}')
        
        # Create the directory if it doesn't exist
        if not os.path.exists(Cassie.output_dir2):  
            os.makedirs(Cassie.output_dir2)

    def run_initial_control(Cassie, qpos0, ctrl0):
        
        frames2 = []
        
        # Edit this one line
        mujoco.mj_resetData(Cassie.model, Cassie.data)
        Cassie.data.qpos = qpos0
        Cassie.data.ctrl = ctrl0
        
        qpos1_data = []
        
        y=1
        
        # Run the simulation while the time is less than the specified duration
        while Cassie.data.time < DURATION:
            
            # Step the simulation
            mujoco.mj_step(Cassie.model, Cassie.data)
            y += 1
    
            # Render and save frames
            if len(frames2) < Cassie.data.time * FRAMERATE:
                
                # Set the lookat point to Cassies COM
                Cassie.camera.lookat = Cassie.data.body('cassie-pelvis').subtree_com
                Cassie.renderer.update_scene(Cassie.data, Cassie.camera)
    
                # Render the current frame
                frame = Cassie.renderer.render()
    
                # Save each frame to a file in the specified directory with 
                # 'y' in the filename
                frame_filename = os.path.join(Cassie.output_dir2, f'{len(frames2):04d}_y{y}.png')
                imageio.imwrite(frame_filename, frame)
    
                frames2.append(frame)
                
                qpos1_data.append(Cassie.data.qpos.copy())  
                
        return frames2, qpos1_data

    def R(Cassie):

        # Q impacts state, R impacts control input. Choosing R is simple
        
        # Use brysons rule (using max inputs)
        R_brysons = np.array([
                     1 / 4.5**2,
                     1 / 4.5**2,
                     1 / 12.2**2,
                     1 / 12.2**2,
                     1 / 0.9**2,
                     
                     1 / 4.5**2,
                     1 / 4.5**2,
                     1 / 12.2**2,
                     1 / 12.2**2,
                     1 / 0.9**2])
                
        R = np.diag(R_brysons)*0.1
                
        return R

    def Q(Cassie):
        
        # Shortcut for the number of DoFs.
        nv = Cassie.model.nv  
        qpos0 = Cassie.data.qpos.copy() 
        
        # Reset the model data before computing Jacobians
        mujoco.mj_resetData(Cassie.model, Cassie.data)
        Cassie.data.qpos = qpos0
        mujoco.mj_forward(Cassie.model, Cassie.data)
        
        # Get the Jacobian for the root body (torso) CoM.
        jac_com = np.zeros((3, nv))
        mujoco.mj_jacSubtreeCom(Cassie.model, Cassie.data, jac_com, Cassie.model.body('cassie-pelvis').id)
        
        # Get the Jacobian for the left foot.
        jac_foot = np.zeros((3, nv))
        mujoco.mj_jacBodyCom(Cassie.model, Cassie.data, jac_foot, None, Cassie.model.body('right-foot').id)
        
        # Compute the difference in Jacobians
        jac_diff = jac_com - jac_foot
        Qbalance = jac_diff.T @ jac_diff
        
        
        # FINAL Q
        pos = np.array([0,         # Pelvis x (in line with foot)
                        0,         # Pelvis y
                        0,         # Pelvis z
                        
                        0,         # Pelvis orientation wx
                        0,         # Pelvis orientation wy 
                        0,         # Pelvis orientation wz
                        
                        # Left Leg
                        3,         # Hip Roll
                        1,         # Hip Yaw
                        2,         # Hip Pitch
                        0.00,      # Achilles Rod
                        0.00,      # Achilles Rod
                        0.00,      # Achilles Rod
                        4,         # Knee
                        0.00,      # Shin
                        0.00,      # Tarsus
                        0.00,      # Heel Spring
                        0.00,      # Foot Crank
                        0.00,      # Plantar Rod
                        1.5,       # Foot
                        
                        # Right Leg
                        1,         # Hip Roll
                        10,        # Hip Yaw
                        20,        # Hip Pitch
                        0.00,      # Achilles Rod
                        0.00,      # Achilles Rod
                        0.00,      # Achilles Rod
                        40,        # Knee
                        0,         # Shin
                        0.00,      # Tarsus
                        0.00,      # Heel Spring
                        0.00,      # Foot Crank
                        0.00,      # Plantar Rod
                        15         # Foot
                       ])
        
        Qpos = np.diag(pos)
        
        vel = np.array([0,         # Pelvis x
                        0,         # Pelvis y
                        0,         # Pelvis z
                        0,         # Pelvis orientation qx
                        0,         # Pelvis orientation qy
                        0.00,      # Pelvis orientation qz
                        
                        # Left Leg
                        1,         # Hip Roll
                        1,         # Hip Yaw
                        2,         # Hip Pitch
                        0.00,      # Achilles Rod
                        0.00,      # Achilles Rod
                        0.00,      # Achilles Rod
                        4,         # Knee
                        0.00,      # Shin
                        0.00,      # Tarsus
                        0.00,      # Heel Spring
                        0.00,      # Foot Crank
                        0.00,      # Plantar Rod
                        1.5,       # Foot
                        
                        # Right Leg
                        10,         # Hip Roll
                        10,         # Hip Yaw
                        20,         # Hip Pitch
                        0.00,       # Achilles Rod
                        0.00,       # Achilles Rod
                        0.00,       # Achilles Rod
                        40,         # Knee
                        0,          # Shin
                        0.00,       # Tarsus
                        0.00,       # Heel Spring
                        0.00,       # Foot Crank
                        0.00,       # Plantar Rod
                        15          # Foot
                       ])
        
        Qvel = np.diag(vel)
        
        # Degrees of freedom (velocity)
        nv = Cassie.model.nv       
        
        Q = np.block([[(10*Qpos + 2000*Qbalance), np.zeros((nv, nv))], [np.zeros((nv, nv)), 0.1*Qvel]])
        
        return Q

    def SS_Model(Cassie, ctrl0, qpos0):
        
        # Set the initial state and control
        mujoco.mj_resetData(Cassie.model, Cassie.data)
        Cassie.data.ctrl = ctrl0
        Cassie.data.qpos = qpos0
    
        # Number of actuators
        nu = Cassie.model.nu       
        
        # Degrees of freedom (velocity)
        nv = Cassie.model.nv       
        
        # Allocate the A and B matrices
        A = np.zeros((2*nv,2*nv))
        B = np.zeros((2*nv,nu))
        
        # Parameters for finite difference approximation
        epsilon = 1e-6
        flg_centered = True
        
        # Compute A and B using finite difference approximations
        mujoco.mjd_transitionFD(Cassie.model, Cassie.data, epsilon, flg_centered, A, B, None, None)

        return A, B

    def compute_K(Cassie, A, B, Q, R):
        
        # Solve discrete Riccati equation to compute matrix P
        P = scipy.linalg.solve_discrete_are(A, B, Q, R)
    
        # Compute the feedback gain matrix K
        K = np.linalg.inv(R + B.T @ P @ B) @ B.T @ P @ A
    
        return K

    def setup_LQR(Cassie):
        
        # Specify the directory to save frames
        output_dir = os.path.expanduser('~/Desktop/THESIS/Code/One_Leg/1.3- Full Control')

        for filename in os.listdir(output_dir): # Clear out images 
            file_path = os.path.join(output_dir, filename)
            try:
                if os.path.isfile(file_path):
                    os.unlink(file_path)
            except Exception as e:
                print(f'Error deleting file {file_path}: {e}')

        # Create the directory if it doesn't exist
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)   

    def main_thread(Cassie):
    
        mujoco.mj_resetDataKeyframe(Cassie.model, Cassie.data, keyframe)

        if toggle == 1:
            Cassie.print_info()
        
            Cassie.setup_no_control()
    
            frames1 = Cassie.run_no_control()
            
            # Create a video from the frames (ensure 'imageio[ffmpeg]' is installed)
            vid1 = os.path.join(Cassie.output_dir, 'No_Control_Single.mp4')
            writer = imageio.get_writer(vid1, format='FFMPEG', fps=FRAMERATE)
            for frame in frames1:
                writer.append_data(frame)
            writer.close()
    
            qpos0, ctrl0 = Cassie.forces_initial_control()
    
            best_offset, vertical_forces, height_offsets = Cassie.height_offset()
            
        else:
            best_offset, vertical_forces, height_offsets = Cassie.height_offset()
        
        # After exiting the viewer, convert lists to NumPy arrays
        best_offset = np.array(best_offset)
        vertical_forces = np.array(vertical_forces)
        height_offsets = np.array(height_offsets)
        
        data_dict_force = {
            'best_offset': best_offset,
            'vertical_forces': vertical_forces,
            'height_offsets': height_offsets
        }

        # Specify output file name
        output_file_path_force = os.path.join(Cassie.output_dir, 'Force_offset.mat')
        savemat(output_file_path_force, data_dict_force)

        qpos0, ctrl0, qfrc0 = Cassie.new_control(best_offset)
        
        # Print forces required and control setpoints
        if toggle == 1:
            
            print('\nActuator Forces:\n', Cassie.data.qfrc_actuator)
            
            print('\nForces Required (with height offset):\n', qfrc0)
            
            # Example actuator-to-joint mapping based on your model's XML file
            actuator_joint_mapping = [
                ("left-hip-roll", 0),
                ("left-hip-yaw", 1),
                ("left-hip-pitch", 2),
                ("left-knee", 3),
                ("left-foot", 4),
                ("right-hip-roll", 5),
                ("right-hip-yaw", 6),
                ("right-hip-pitch", 7),
                ("right-knee", 8),
                ("right-foot", 9)
            ]
            
            # Find the maximum joint name length for proper formatting
            max_joint_name_length = max(len(joint_name) for joint_name, _ in actuator_joint_mapping)
        
            # Print actuator control setpoints in the desired formatted manner
            print("\nControl Setpoints (Torque Values):")
            for j, (joint_name, index) in enumerate(actuator_joint_mapping):
                print(f"    Actuator {j:02d}: {joint_name.ljust(max_joint_name_length)} | Control Setpoint: {ctrl0[index]:.6f} Nm")

            Cassie.setup_initial_control()
    
            frames2, qpos1_data = Cassie.run_initial_control(qpos0, ctrl0)
            
            # Create a video from the frames (ensure 'imageio[ffmpeg]' is installed)
            vid2 = os.path.join(Cassie.output_dir, 'Initial_Control_Single.mp4')
            writer = imageio.get_writer(vid2, format='FFMPEG', fps=FRAMERATE)
            for frame in frames2:
                writer.append_data(frame)
            writer.close()

        # Define the path to the output directory for the CSV files
        csv_output_dir = os.path.expanduser('~/Desktop/THESIS/Code/One_Leg_LQR')

        R = Cassie.R()
        
        # Save matrix R to CSV
        R_csv_path = os.path.join(csv_output_dir, 'R.csv')
        np.savetxt(R_csv_path, R, delimiter=',')

        Q = Cassie.Q()
        
        # Save matrix A to CSV
        Q_csv_path = os.path.join(csv_output_dir, 'Q.csv')
        np.savetxt(Q_csv_path, Q, delimiter=',')
        
        A, B = Cassie.SS_Model(ctrl0, qpos0)
    
        # Save matrix A to CSV
        a_csv_path = os.path.join(csv_output_dir, 'A.csv')
        np.savetxt(a_csv_path, A, delimiter=',')
        
        # Save matrix B to CSV
        b_csv_path = os.path.join(csv_output_dir, 'B.csv')
        np.savetxt(b_csv_path, B, delimiter=',')
        
        K = Cassie.compute_K(A, B, Q, R)
                
        # Create a video from the frames (ensure 'imageio[ffmpeg]' is installed)
        vid3 = os.path.join(Cassie.output_dir, 'Full_Control_Single.mp4')
        writer = imageio.get_writer(vid3, format='FFMPEG', fps=FRAMERATE)
        
        max_simulation_time = 6000.0  # Change this to your desired duration
        
        # Record the start time
        start_time = time.time()
        
        qpos_target = Cassie.model.key_qpos[key_target].copy()
        
        with mujoco.viewer.launch_passive(Cassie.model, Cassie.data) as viewer:
            
            print("\n\n------------------------LQR CONTROL------------------------\n\n")       
            
            frames3 = []
            
            # Lists to collect state data
            time_data = []
            qpos_data = []
            qvel_data = []
            COM_x = []
            COM_y = []
            COM_z = []
            u = []
            COM_vel_x = []
            COM_vel_y = []
            COM_vel_z = []
            
            mujoco.mj_comPos(Cassie.model, Cassie.data)
            sys_com = Cassie.data.subtree_com[0]  # COM for the root

            print("COM Starting Position:",sys_com*100)
            
            x = 1
            
            f = 0
            force = np.array([6, 0, 0])  

            # only edit from here
            while viewer.is_running():
                
                # Check if the elapsed time exceeds the maximum simulation time
                elapsed_time = time.time() - start_time
                
                if elapsed_time > max_simulation_time:
                    # Exit the loop if the time limit is reached
                    break  
                                
                nv = Cassie.model.nv
                
                # Allocate position difference dq.
                dq = np.zeros(nv)

                # MuJoCo differentiates only over the "active" degrees of 
                # freedom (DoFs) of the system, which correspond to the 
                # velocity elements (nv), not the position elements (nx).
                
                x+=1

                if transition_toggle == 1:
                    mujoco.mj_differentiatePos(Cassie.model, dq, 1, qpos_target, Cassie.data.qpos)
                    dx = np.hstack((dq, Cassie.data.qvel)).T
                else:
                    mujoco.mj_differentiatePos(Cassie.model, dq, 1, qpos0, Cassie.data.qpos)
                    dx = np.hstack((dq, Cassie.data.qvel)).T

                Cassie.data.ctrl = ctrl0 - K @ dx
                
                if Cassie.data.time > 1 and f == 1:
                    
                    # No torque applied 
                    torque = np.zeros(3)  
                    
                    com_pelvis = Cassie.data.subtree_com[0]
                    print(com_pelvis)
                    mujoco.mj_applyFT(Cassie.model, Cassie.data, force, torque, com_pelvis, 1, Cassie.data.qfrc_applied)
                    
                    print("Done")
                                        
                    f = f+1
                
                # Step the simulation.
                mujoco.mj_step(Cassie.model, Cassie.data)
                
                mujoco.mj_comPos(Cassie.model, Cassie.data)
                sys_com = Cassie.data.subtree_com[0]  # COM for the root
                COM_x.append(sys_com[0])
                COM_y.append(sys_com[1])
                COM_z.append(sys_com[2])
                
                # Calculate the velocity of the COM
                mujoco.mj_comVel(Cassie.model, Cassie.data)
                sys_com_vel = Cassie.data.subtree_linvel[0]  # COM velocity for the root
                COM_vel_x.append(sys_com_vel[0])
                COM_vel_y.append(sys_com_vel[1])
                COM_vel_z.append(sys_com_vel[2])
                
                time_data.append(Cassie.data.time)
                qpos_data.append(Cassie.data.qpos.copy())  # Save position
                qvel_data.append(Cassie.data.qvel.copy())  # Save velocity
                u.append(Cassie.data.ctrl)
                

                # Update the viewer scene
                with viewer.lock():
                    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(Cassie.data.time % 2)
                
                #Render and save frames.
                if len(frames3) < Cassie.data.time * FRAMERATE:
                  
                    #Cassie.camera.lookat = Cassie.data.body('cassie-pelvis').subtree_com
                    Cassie.camera.lookat = Cassie.data.body('right-foot-crank').subtree_com

                    # Set the camera parameters (elevation, distance, azimuth)
                    Cassie.camera.elevation = -30  # Set the fixed elevation
                    Cassie.camera.distance = 2.3   # Set the fixed distance
                    Cassie.camera.azimuth = 165    # Set the fixed azimuth
                  
                    Cassie.renderer.update_scene(Cassie.data, Cassie.camera)
                    frame = Cassie.renderer.render()
                    
                    frame_filename = os.path.join(Cassie.output_dir3, f'{len(frames3):04d}:{Cassie.data.time:.2f}s.png')
                    imageio.imwrite(frame_filename, frame)
                    
                    frames3.append(frame)
                    writer.append_data(frame)

                viewer.sync()
        
            writer.close()

            # After exiting the viewer, convert lists to NumPy arrays
            time_data = np.array(time_data)
            qpos_data = np.array(qpos_data)
            qvel_data = np.array(qvel_data)
            COM_x = np.array(COM_x)
            COM_y = np.array(COM_y)
            COM_z = np.array(COM_z)
            u = np.array(u)

            data_dict = {
                'time': time_data,
                'qpos': qpos_data,
                'qvel': qvel_data,
                'qpos0': qpos0,
                'qpos_t': qpos_target,
                'COM_x': COM_x,
                'COM_y': COM_y,
                'COM_z': COM_z,
                'COM_vel_x': COM_vel_x,
                'COM_vel_y': COM_vel_y,
                'COM_vel_z': COM_vel_z,
                'u': u
            }

            # Specify output file name
            output_file_path = os.path.join(Cassie.output_dir, 'Final_Data1.mat')
            savemat(output_file_path, data_dict)


if __name__ == '__main__':
    Cassie_LQR().main_thread()
    
