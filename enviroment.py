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

COLORS = {
    'blue':   (78/255,  121/255, 167/255, 255/255),
    'red':    (255/255,  87/255,  89/255, 255/255),
    'green':  (89/255,  169/255,  79/255, 255/255),
    'orange': (242/255, 142/255,  43/255, 255/255),
    'yellow': (237/255, 201/255,  72/255, 255/255),
    'purple': (176/255, 122/255, 161/255, 255/255),
    'pink':   (255/255, 157/255, 167/255, 255/255),
    'cyan':   (118/255, 183/255, 178/255, 255/255),
    'brown':  (156/255, 117/255,  95/255, 255/255),
    'gray':   (186/255, 176/255, 172/255, 255/255),
}

CORNER_POS = {
  'top left corner':     (-0.3 + 0.05, -0.2 - 0.05, 0),
  'top side':            (0,           -0.2 - 0.05, 0),
  'top right corner':    (0.3 - 0.05,  -0.2 - 0.05, 0),
  'left side':           (-0.3 + 0.05, -0.5,        0),
  'middle':              (0,           -0.5,        0),
  'right side':          (0.3 - 0.05,  -0.5,        0),
  'bottom left corner':  (-0.3 + 0.05, -0.8 + 0.05, 0),
  'bottom side':         (0,           -0.8 + 0.05, 0),
  'bottom right corner': (0.3 - 0.05,  -0.8 + 0.05, 0),
}

ALL_BLOCKS = ['blue block', 'red block', 'green block', 'orange block', 'yellow block', 'purple block', 'pink block', 'cyan block', 'brown block', 'gray block']
ALL_BOWLS = ['blue bowl', 'red bowl', 'green bowl', 'orange bowl', 'yellow bowl', 'purple bowl', 'pink bowl', 'cyan bowl', 'brown bowl', 'gray bowl']

PIXEL_SIZE = 0.00267857
# BOUNDS = np.float32([[-0.85, -0.45, 0.65], [1.2, -0.45, 0.65], [1.2, 0.45, 0.65], [-0.85, 0.45, 0.65]])  # X Y Z
BOUNDS = np.float32([[0.85, -0.0, 0.65], [0.85, -0.0, 0.65], [0.85, -0.0, 0.65], [0.85, -0.0, 0.65]])  # X Y Z

class tableTopEnv: 

    def __init__(self):
        self.dt = 1/480
        self.sim_step = 0

        p.connect(p.DIRECT) #p.DIRECT or p.GUI for graphical version
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0,0,-10)

        
        self.planeId = p.loadURDF("plane.urdf")
        self.table_id = p.loadURDF("table/table.urdf", basePosition=[1.0, -0.0, 0.0], baseOrientation=[0, 0, 0.7071, 0.7071])
        self.cube_center = p.loadURDF("cube.urdf", basePosition=[0.0, 0, 0], globalScaling=0.1)
        self.robot = robot.KukaRobot(tool="gripper", basePosition=[1.40, -0.0, 0.60], baseOrientation=[0, 0, 0, 1])
        
        self.cameras = []
        
        self.main_cam = camera.video_recorder()
        self.cameras.append(self.main_cam)



        min_x, min_y, min_z = np.min(BOUNDS, axis=0)
        max_x, max_y, max_z = np.max(BOUNDS, axis=0)

        num_blocks = 3 #@param {type:"slider", min:0, max:4, step:1}

        block_list = np.random.choice(ALL_BLOCKS, size=num_blocks, replace=False).tolist()
        obj_list = block_list 

        self.object_list = obj_list
        self.obj_name_to_id = {}
        obj_xyz = np.zeros((0, 3))
        for obj_name in obj_list:
            if ('block' in obj_name):

                # Get random position 15cm+ from other objects.
                while True:
                    random_x = np.random.uniform(0.95, 0.75)
                    random_y = np.random.uniform(-0.1, 0.1)
                    rand_xyz = np.array([[random_x, random_y, 0.65]])
                    if len(obj_xyz) == 0:
                        obj_xyz = np.concatenate((obj_xyz, rand_xyz), axis=0)
                        break
                    else:
                        nn_dist = np.min(np.linalg.norm(obj_xyz - rand_xyz, axis=1)).squeeze()
                        if nn_dist > 0.15:
                            obj_xyz = np.concatenate((obj_xyz, rand_xyz), axis=0)
                            break
                
                object_color = COLORS[obj_name.split(' ')[0]]
                object_type = obj_name.split(' ')[1]
                object_position = rand_xyz.squeeze()
                if object_type == 'block':
                    object_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.025, 0.025, 0.025])
                    object_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.025, 0.025, 0.025])
                    object_id = p.createMultiBody(0.05, object_shape, object_visual, basePosition=object_position)
                p.changeVisualShape(object_id, -1, rgbaColor=object_color)
                self.obj_name_to_id[obj_name] = object_id
        print(self.obj_name_to_id)

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

            # # check if its close to target
            # print(f'EE position x: {EE_pos[0]:.2f}, y: {EE_pos[1]:.2f}, z: {EE_pos[2]:.2f}...', end='')
            # print(f'Joint 5 position x: {joint_pos[0]:.2f}, y: {joint_pos[1]:.2f}, z: {joint_pos[2]:.2f}...', end='')
            # print(f'Target position x: {target_pos[0]:.2f}, y: {target_pos[1]:.2f}, z: {target_pos[2]:.2f}...', end='')

            # # # print the gripper value and force and gripper target valye
            # gripper_val = p.getJointState(self.robot.kuka_gripper_id, 4)[0]
            # print(f'Gripper value: {gripper_val:.2f}, force: {EE_force}...', end='')
            # print(f'Target gripper value: {target_gripper_val:.2f}...', end='')

            
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

