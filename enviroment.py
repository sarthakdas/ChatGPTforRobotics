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
    'blue': (78/255, 121/255, 167/255, 255/255),
    'red': (255/255, 87/255, 89/255, 255/255),
    'green': (89/255, 169/255, 79/255, 255/255),
    'orange': (242/255, 142/255, 43/255, 255/255),
    'yellow': (237/255, 201/255, 72/255, 255/255),
    'purple': (176/255, 122/255, 161/255, 255/255),
    'pink': (255/255, 157/255, 167/255, 255/255),
    'cyan': (118/255, 183/255, 178/255, 255/255),
    'brown': (156/255, 117/255, 95/255, 255/255),
    'gray': (186/255, 176/255, 172/255, 255/255),
}

CORNER_POS = {
    'top left corner': (-0.3 + 0.05, -0.2 - 0.05, 0),
    'top side': (0, -0.2 - 0.05, 0),
    'top right corner': (0.3 - 0.05, -0.2 - 0.05, 0),
    'left side': (-0.3 + 0.05, -0.5, 0),
    'middle': (0, -0.5, 0),
    'right side': (0.3 - 0.05, -0.5, 0),
    'bottom left corner': (-0.3 + 0.05, -0.8 + 0.05, 0),
    'bottom side': (0, -0.8 + 0.05, 0),
    'bottom right corner': (0.3 - 0.05, -0.8 + 0.05, 0),
}

ALL_BLOCKS = ['blue block', 'red block', 'green block', 'orange block', 'yellow block', 'purple block', 'pink block', 'cyan block', 'brown block', 'gray block']
ALL_BOWLS = ['blue bowl', 'red bowl', 'green bowl', 'orange bowl', 'yellow bowl', 'purple bowl', 'pink bowl', 'cyan bowl', 'brown bowl', 'gray bowl']

PIXEL_SIZE = 0.00267857
BOUNDS = np.float32([[0.85, -0.0, 0.65], [0.85, -0.0, 0.65], [0.85, -0.0, 0.65], [0.85, -0.0, 0.65]])  # X Y Z

class TableTopEnv: 

    def __init__(self, display=False, objects=None):
        self.dt = 1/480
        self.sim_step = 0
        self.distance_threshold = 0.05
        self.orientation_threshold = 0.2
        self.time_step_movement_max = 50

        p.connect(p.GUI if display else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -10)

        self.planeId = p.loadURDF("plane.urdf")
        self.table_id = p.loadURDF("table/table.urdf", basePosition=[1.0, -0.0, 0.0], baseOrientation=[0, 0, 0.7071, 0.7071])
        self.cube_center = p.loadURDF("cube.urdf", basePosition=[0.0, 0, 0], globalScaling=0.1)
        self.robot = robot.PandaRobot(tool="gripper")
        
        self.cameras = []
        self.main_cam = camera.VideoRecorder()
        self.cameras.append(self.main_cam)

        self.gripped_object = None
        
        self.obj_name_to_id = {}
        if objects:
             self.load_objects(objects)
        else:
            colors = [
                [1, 0, 0, 1],  # Red
                [0, 1, 0, 1],  # Green
                [0, 0, 1, 1],  # Blue
            ]
            self.load_random_cubes_with_colors(3, colors)

        for _ in range(50):
            self.time_step()
        
        print("Environment initialized")
        print("Objects in the environment: ", self.obj_name_to_id)

        num_joints = p.getNumJoints(self.robot.panda_id)
        for i in range(num_joints):
            joint_info = p.getJointInfo(self.robot.panda_id, i)
            print(f"Link index: {i}, Link name: {joint_info[12].decode('utf-8')}")

    def time_sequence(self, target_pos, target_orn=[0, 0, 0, 1], target_gripper_val=0.04):
        running = True
        current_time_step = self.sim_step
        while running:
            self._debug_robot_joint_positions()

            EE_pos = self.robot.get_end_effector_position()
            EE_orn = self.robot.get_end_effector_orientation()
            self.main_cam.robot_pos = f"EE pos: {EE_pos}"
            self.main_cam.robot_orn = f"EE orn: {EE_orn}"

            self.robot.robot_control(target_pos, target_orn, target_gripper_val)
            
            self.time_step()
            time.sleep(self.dt)

            EE_pos = self.robot.get_end_effector_position()
            EE_orn = self.robot.get_end_effector_orientation()

            if (np.linalg.norm(np.array(EE_pos) - np.array(target_pos)) < self.distance_threshold) and (np.linalg.norm(np.array(EE_orn) - np.array(target_orn)) < self.orientation_threshold) or (current_time_step + self.time_step_movement_max < self.sim_step):
                running = False
            
            pos_dist = np.linalg.norm(np.array(EE_pos) - np.array(target_pos))
            orn_dist = np.linalg.norm(np.array(EE_orn) - np.array(target_orn))

            reach_condition = f"{(pos_dist < self.distance_threshold):.2f}, {(orn_dist < self.orientation_threshold):.2f}, :===> ,{pos_dist:.2f}, {orn_dist:.2f}"
            self.main_cam.task = f"Reach condition: {reach_condition}"
            self._debug_robot_joint_positions()

    def object_id_of_being_sucked(self):
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

    def get_object_positions(self):
        object_positions = {}
        for obj_name in self.obj_name_to_id:
            position, orientation = p.getBasePositionAndOrientation(self.obj_name_to_id[obj_name])
            object_positions[obj_name] = {
                'position': position,
                'orientation': orientation
            }
        return object_positions

    
    def get_object_position(self, obj):
        return p.getBasePositionAndOrientation(obj)

    def time_step(self):
        self.capture_frames()
        p.stepSimulation()
        self.sim_step += 1

    def capture_frames(self):
        for cam in self.cameras:
            cam.record_frame()

    def stop(self):
        p.disconnect()
        for cam in self.cameras:
            cam.close()

    def load_random_cubes_with_colors(self, number_of_cubes, colors):
        if len(colors) != number_of_cubes:
            raise ValueError("The number of colors provided must match the number of cubes")

        obj_name_to_id = {}
        for i, obj_name in enumerate(range(number_of_cubes)):
            object_position = self.generate_random_position()
            object_color = colors[i]
            object_id = self.create_block(object_position, object_color)
            obj_name_to_id[obj_name] = object_id

        return obj_name_to_id
    
    def load_objects(self, objects_with_positions, num_cubes=0):
        flags = p.URDF_USE_INERTIA_FROM_FILE
        models = models_data.model_lib()


        if not isinstance(objects_with_positions, dict):
            raise ValueError("Input must be a dictionary with object names as keys and positions as values.")

        for object_name, object_position in objects_with_positions.items():
            if object_position is None:
                object_position = self.generate_random_position()

            if not isinstance(object_position, list) or len(object_position) != 3:
                raise ValueError(f"Position for {object_name} must be a list of three elements.")

            self.obj_name_to_id[object_name] = p.loadURDF(models[object_name], object_position, flags=flags)
            print(f"Loaded {object_name} at position {object_position}")
            print(self.obj_name_to_id)
        if num_cubes > 0:
            self.load_random_cubes(num_cubes)

    def load_random_cubes(self, number_of_cubes):
        block_list = np.random.choice(ALL_BLOCKS, size=number_of_cubes, replace=False).tolist()

        for block_name in block_list:
            object_position = self.generate_random_position()
            object_color = COLORS[block_name.split()[0]]
            object_id = self.create_block(object_position, object_color)
            self.obj_name_to_id[block_name] = object_id

    def generate_random_position(self):
        random_x = np.random.uniform(0.75, 0.95)
        random_y = np.random.uniform(-0.1, 0.1)
        return [random_x, random_y, 0.7]

    def create_block(self, position, color):
        half_extents = [0.025, 0.025, 0.025]
        mass = 0.1
        
        object_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)
        object_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=half_extents)
        object_id = p.createMultiBody(mass, object_shape, object_visual, basePosition=position)
        p.changeVisualShape(object_id, -1, rgbaColor=color)
        
        return object_id
    
    def _debug_robot_joint_positions(self):
        num_joints = p.getNumJoints(self.robot.panda_id)
        link_indices = list(range(num_joints))
        link_states = p.getLinkStates(self.robot.panda_id, link_indices, computeForwardKinematics=1)
        joint_positions = [link_state[4] for link_state in link_states]  # [4] is the world position

        self.main_cam.other = [f'Joint {i}: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})' for i, pos in enumerate(joint_positions)]
