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
from urdf_models import models_data

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

    def __init__(self, display=False):
        self.dt = 1/480
        self.sim_step = 0

        self.distance_threshold = 0.05
        self.orientation_threshold = 0.2
        self.time_step_movement_max = 50 #300

        if display:
            p.connect(p.GUI)
        else:
            p.connect(p.DIRECT) #p.DIRECT or p.GUI for graphical version
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        print(pybullet_data.getDataPath())
        p.setGravity(0,0,-10)

        
        self.planeId = p.loadURDF("plane.urdf")
        self.table_id = p.loadURDF("table/table.urdf", basePosition=[1.0, -0.0, 0.0], baseOrientation=[0, 0, 0.7071, 0.7071])
        self.cube_center = p.loadURDF("cube.urdf", basePosition=[0.0, 0, 0], globalScaling=0.1)
        self.robot = robot.PandaRobot(tool="gripper", basePosition=[1.40, -0.0, 0.60], baseOrientation=[0, 0, 0, 1])
        
        self.cameras = []
        
        self.main_cam = camera.video_recorder()
        
        self.cameras.append(self.main_cam)

        # create a camera that looks top down on the table
        # self.topdown_cam = camera.video_recorder(pos=[1.0, -0.2, 0.5], distance=1.0, yaw=0, pitch=-90, roll=0, width=480, height=360, filename='topdown.mp4')
        # self.cameras.append(self.topdown_cam)

        self.gripped_object = None
        self.obj_name_to_id = {}


        # 3 plates and food items
        self.load_objects(["plate","blue_plate","square_plate_4", "plastic_plum", "plastic_pear", "plastic_apple", "plastic_strawberry", "plastic_lemon"], [[0.85, -0.20, 0.65],[0.9, 0.0, 0.65], [0.85, 0.2, 0.65], [0.85, -0.2, 0.7], [0.9, -0.2, 0.7], [0.8, -0.2, 0.7], [0.9, 0.0, 0.7], [0.85, 0.2, 0.7]])

        # 3 plates and 3 cubes
        # self.load_objects(["plate","blue_plate","square_plate_4"], [[0.85, -0.20, 0.65],[0.9, 0.0, 0.65], [0.85, 0.2, 0.65]], num_cubes=0)



        # timestep 50 times to let the objects settle
        for _ in range(50):
            self.time_step()
        
        print("Environment initialized")
        print("Objects in the enviroment: ", self.obj_name_to_id.keys())


        # num_bodies = p.getNumBodies()
        # # Print information about each body
        # for i in range(num_bodies):
        #     body_id = p.getBodyUniqueId(i)
        #     body_info = p.getBodyInfo(body_id)
        #     body_name = body_info[1].decode('utf-8')  # Decode byte string to a regular string
        #     print(f"Body {i}: ID = {body_id}, Name = {body_name}")

        num_joints = p.getNumJoints(self.robot.panda_id)
        # Print details for each link
        for i in range(num_joints):
            joint_info = p.getJointInfo(self.robot.panda_id, i)
            print(f"Link index: {i}, Link name: {joint_info[12].decode('utf-8')}")



    def time_sequence(self, target_pos, target_orn, target_gripper_val):
        '''Moves the robot to the target position and set the gripper value, will do this until the target position is reached.
        Input: target position and gripper value. Output: None.'''
        running = True
        current_time_step = self.sim_step
        while running:
            self._debugg_robot_joint_positions()

            print(f'\rtimestep {self.sim_step}...', end='')

            # update the camera text to 2 decimal places
            EE_pos = self.robot.get_end_effector_position()
            EE_orn = self.robot.get_end_effector_orientation()

            # print("Enviroment Target Position: ", target_pos)
            self.main_cam.robot_pos = f'Robot position x: {EE_pos[0]:.2f}, y: {EE_pos[1]:.2f}, z: {EE_pos[2]:.2f}'
            self.main_cam.robot_target_pos = f'Target position x: {target_pos[0]:.2f}, y: {target_pos[1]:.2f}, z: {target_pos[2]:.2f}'

            end_effector_euler = p.getEulerFromQuaternion(EE_orn)
            target_euler = p.getEulerFromQuaternion(target_orn)         

            # Convert radians to degrees
            end_effector_euler_degrees = [math.degrees(angle) for angle in end_effector_euler]
            target_euler_degrees = [math.degrees(angle) for angle in target_euler]

            # Format strings with degrees
            self.main_cam.robot_orn = f'Robot orientation roll: {end_effector_euler_degrees[0]:.2f}, pitch: {end_effector_euler_degrees[1]:.2f}, yaw: {end_effector_euler_degrees[2]:.2f}'
            self.main_cam.robot_target_orn = f'Target orientation roll: {target_euler_degrees[0]:.2f}, pitch: {target_euler_degrees[1]:.2f}, yaw: {target_euler_degrees[2]:.2f}'

            self.robot.robot_control(target_pos, target_orn, target_gripper_val)
            
            # if self.robot.tool == "suction":
            #     if self.robot.suction and self.gripped_object is None:
            #         # get the object that is being sucked
            #         obj = self.object_id_of_being_sucked()
            #         if obj is not None:
            #             # get the object position and orientation
            #             obj_pos, obj_orn = p.getBasePositionAndOrientation(obj)
            #             obj_orn = p.getQuaternionFromEuler([0, math.pi, 0])
            #             # create a constraint between the end effector and the object that is being sucked
            #             self.gripped_object = p.createConstraint(self.robot.kuka_id, 6, obj, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0.05], [0, 0, 0], childFrameOrientation=obj_orn)
            #     elif not self.robot.suction and self.gripped_object is not None:
            #         # remove the constraint if the object is not being sucked
            #         p.removeConstraint(self.gripped_object)
            #         self.gripped_object = None
            #         self.robot.suction = False
            
            # elif self.robot.tool == "gripper":
            #     pass

            # Step the simulation forward
            self.time_step()
            time.sleep(self.dt)

            # get EE position
            EE_pos = self.robot.get_end_effector_position()
            EE_orn = self.robot.get_end_effector_orientation()

            # check if the EE is close to the target and gripper is at the target value or force is too high
            if (np.linalg.norm(np.array(EE_pos) - np.array(target_pos)) < self.distance_threshold) and (np.linalg.norm(np.array(EE_orn) - np.array(target_orn)) < self.orientation_threshold) or (current_time_step + self.time_step_movement_max < self.sim_step):
                running = False
            
            pos_dist = np.linalg.norm(np.array(EE_pos) - np.array(target_pos))
            orn_dist = np.linalg.norm(np.array(EE_orn) - np.array(target_orn))

            # Format the reach condition with two decimal places
            reach_condition = f"{(pos_dist < self.distance_threshold):.2f}, {(orn_dist < self.orientation_threshold):.2f}, :===> ,{pos_dist:.2f}, {orn_dist:.2f}"

            # Update the task description
            self.main_cam.task = f"Reach condition: {reach_condition}"


    def object_id_of_being_sucked(self):
         # get cube_id that is closest to the end effector if no object is within 3 cm then return None
        min_dist = 0.13
        cube_id = None
        for obj_name, obj_id in self.obj_name_to_id.items():
            obj_pos, _ = p.getBasePositionAndOrientation(obj_id)
            EE_pos = self.robot.get_end_effector_position()
            dist = np.linalg.norm(np.array(obj_pos) - np.array(EE_pos))
            print(f'dist from object: {obj_name} is {dist:.2f}...', end='')
            if dist < min_dist:
                min_dist = dist
                cube_id = obj_id

        return cube_id

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
        self.capture_frames()
        p.stepSimulation()
        self.sim_step += 1

    def capture_frames(self):
         for cam in self.cameras:
            cam.record_frame()


    def stop(self):
        '''Disconnect from the physics server and close the cameras. Output: None.'''
        p.disconnect()
        for cam in self.cameras:
            cam.close()

    def load_objects(self, objects, positions=False, num_cubes=0):
        
        flags = p.URDF_USE_INERTIA_FROM_FILE
        models = models_data.model_lib()
        namelist = models.model_name_list

        if positions != False and len(objects) != len(positions):
            raise ValueError("If positions are given, they must be the same length as the objects list.")

        for i in range(len(objects)):
            if positions == False:
                random_x = np.random.uniform(0.95, 0.75)
                random_y = np.random.uniform(-0.1, 0.1)
                rand_xyz = np.array([[random_x, random_y, 0.65]])
                object_position = rand_xyz.squeeze()
            else:
                object_position = positions[i]
            self.obj_name_to_id[objects[i]] = p.loadURDF(models[objects[i]], object_position, flags=flags)

        if num_cubes > 0:
            self.load_random_cubes(num_cubes)

    def load_random_cubes(self,numer_of_cubes):
        block_list = np.random.choice(ALL_BLOCKS, size=numer_of_cubes, replace=False).tolist() 


        for obj_name in block_list:
            if ('block' in obj_name):
                random_x = np.random.uniform(0.95, 0.75)
                random_y = np.random.uniform(-0.1, 0.1)
                rand_xyz = np.array([[random_x, random_y, 0.7]])
                    
                
                object_color = COLORS[obj_name.split(' ')[0]]
                object_type = obj_name.split(' ')[1]
                object_position = rand_xyz.squeeze()
                if object_type == 'block':
                    object_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.025, 0.025, 0.025])
                    object_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.025, 0.025, 0.025])
                    object_id = p.createMultiBody(0.1, object_shape, object_visual, basePosition=object_position)
                p.changeVisualShape(object_id, -1, rgbaColor=object_color)
                self.obj_name_to_id[obj_name] = object_id
    
    def _debugg_robot_joint_positions(self):
        '''Print the robot joint positions. Output: None.'''
        # Get the number of joints
        num_joints = p.getNumJoints(self.robot.panda_id)
        # Get the joint positions as a list of XYZ coordinates
        link_indices = list(range(num_joints))
        link_states = p.getLinkStates(self.robot.panda_id, link_indices, computeForwardKinematics=1)
        joint_positions = [link_state[4] for link_state in link_states]  # [4] is the world position

        # Update camera with the joint positions string on new line for each joint
        self.main_cam.other = [f'Joint {i}: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})' for i, pos in enumerate(joint_positions)]
