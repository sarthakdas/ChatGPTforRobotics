import os
import time
import math
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
import pybullet as p
import pybullet_data
import cv2
import imageio_ffmpeg
from base64 import b64encode
from IPython.display import HTML

import camera
import robot

class tableTopEnv: 

    def __init__(self):
        self.dt = 1/480
        self.sim_step = 0

        p.connect(p.GUI) #p.DIRECT or p.GUI for graphical version
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0,0,-10)

        
        self.planeId = p.loadURDF("plane.urdf")
        self.table_id = p.loadURDF("table/table.urdf", basePosition=[1.0, -0.0, 0.0], baseOrientation=[0, 0, 0.7071, 0.7071])
        self.cube_id = p.loadURDF("cube.urdf", basePosition=[0.85, -0.0, 0.65], globalScaling=0.05)
        self.cube_center = p.loadURDF("cube.urdf", basePosition=[0.0, 0, 0], globalScaling=0.1)
        # self.cube_id1 = p.loadURDF("cube.urdf", basePosition=[0.85, -0.1, 0.65], globalScaling=0.1)
        self.robot = robot.KukaRobot(tool="gripper", basePosition=[1.40, -0.0, 0.60], baseOrientation=[0, 0, 0, 1])

        self.moveable_objects = []
        self.moveable_objects.append(self.cube_id)
        self.moveable_objects.append(self.cube_center)

        self.cameras = []
        
        self.main_cam = camera.video_recorder()
        self.cameras.append(self.main_cam)

        # create a camera that looks top down on the table
        # self.topdown_cam = camera.video_recorder(pos=[1.0, -0.2, 0.5], distance=1.0, yaw=0, pitch=-90, roll=0, width=480, height=360, filename='topdown.mp4')
        # self.cameras.append(self.topdown_cam)

    # def kuka_control(self, target_pos, gripper_val):
    #     jointPoses = p.calculateInverseKinematics(self.kuka_id, self.n_joints, target_pos)
    #     for i in range(self.n_joints):
    #         p.setJointMotorControl2(self.kuka_id, i, p.POSITION_CONTROL, jointPoses[i], force=5 * 240.)
    #     p.setJointMotorControl2(self.kuka_id, 7, p.POSITION_CONTROL, gripper_val, force=5 * 240.)

    def time_sequence(self, target_pos, target_gripper_val):
        '''Moves the robot to the target position and set the gripper value, will do this until the target position is reached.
        Input: target position and gripper value. Output: None.'''
        running = True
        
        while running:
            print(f'\rtimestep {self.sim_step}...', end='')


            self.robot.kuka_control(target_pos, target_gripper_val)

            # Step the simulation forward
            self.time_step()
            time.sleep(self.dt)

            # get EE position
            EE_pos = self.robot.get_end_effector_position()
            joint_pos = self.robot.get_joint_position(5)

            # get the force of the EE
            EE_force = self.robot.get_end_effector_force()

            # check if its close to target
            print(f'EE position x: {EE_pos[0]:.2f}, y: {EE_pos[1]:.2f}, z: {EE_pos[2]:.2f}...', end='')
            print(f'Joint 5 position x: {joint_pos[0]:.2f}, y: {joint_pos[1]:.2f}, z: {joint_pos[2]:.2f}...', end='')
            print(f'Target position x: {target_pos[0]:.2f}, y: {target_pos[1]:.2f}, z: {target_pos[2]:.2f}...', end='')

            # # print the gripper value and force and gripper target valye
            gripper_val = p.getJointState(self.robot.kuka_gripper_id, 4)[0]
            print(f'Gripper value: {gripper_val:.2f}, force: {EE_force}...', end='')
            print(f'Target gripper value: {target_gripper_val:.2f}...', end='')

            
            # check if the EE is close to the target and gripper is at the target value or force is too high
            if (np.linalg.norm(np.array(EE_pos) - np.array(target_pos)) < 0.04):
                running = False

    def get_object_positons(self):
        '''Return a dictionary of object positions. Output: dictionary of object positions.'''
        # return a dictionary of object positions
        object_positions = {}
        for obj in self.moveable_objects:
            pos, _ = p.getBasePositionAndOrientation(obj)
            object_positions[obj] = pos
        return object_positions
    
    def get_object_position(self, obj):
        '''Return the position of the object. Input: object id. Output: object position.'''
        return p.getBasePositionAndOrientation(obj)

    def time_step(self):
        '''Step the simulation forward by one time step, and record the frame. Output: None.'''
        for cam in self.cameras:
            cam.record_frame()
        p.stepSimulation()
        self.sim_step += 1

    def stop(self):
        '''Disconnect from the physics server and close the cameras. Output: None.'''
        p.disconnect()
        for cam in self.cameras:
            cam.close()

