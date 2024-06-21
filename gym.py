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
import random
import keyboard
import json

import camera
import enviroment
import gpt
import core
import robotController
from dotenv import load_dotenv

# Initialize environment and code generator
env = enviroment.tableTopEnv()
load_dotenv()
openai_api_key = os.getenv("OPENAI_API_KEY")
code_gen = gpt.OpenAIClient(api_key=openai_api_key)

def go_to_position(target_pos, target_orn, gripper_val, duration=10):
    '''Move the robot to the target position and set the gripper value over a duration.'''
    pos = env.robot.get_end_effector_position()
    # Linear interpolate to target position
    for i in range(duration):
        new_pos = (pos[0] + (target_pos[0] - pos[0]) * i / duration,
                   pos[1] + (target_pos[1] - pos[1]) * i / duration,
                   pos[2] + (target_pos[2] - pos[2]) * i / duration)
        env.time_sequence(new_pos, target_orn, gripper_val)
    env.time_sequence(target_pos, target_orn, gripper_val)

def object_name_to_id_dic():
    '''Get the object dictionary and return the object names and ids.'''
    return env.obj_name_to_id

def get_object_position(obj_id):
    '''Get the object position.'''
    pos, _ = env.get_object_position(obj_id)
    return pos

def wait(steps=10):
    for _ in range(steps):
        env.time_step()

def execute_waypoints_from_json(filepath, skip=0):
    '''Execute waypoints from a JSON file, skipping waypoints as specified by the skip parameter.'''
    # Load the JSON file
    with open(filepath, 'r') as f:
        data = json.load(f)
    
    # Get the waypoints from the JSON data
    waypoints = data['demonstration']

    env.robot.set_joint_positions(data['starting_joint_pos'])

    # Execute waypoints with the specified skip parameter
    for i in range(0, len(waypoints), skip + 1):
        waypoint = waypoints[i]
        pos = waypoint['xyz_position']
        orn = waypoint['quaternion_orientation']
        gripper_val = waypoint['gripper_state']
        go_to_position(pos, orn, gripper_val, duration=1)
        print(f"Moved to position: {pos} with orientation: {orn} and gripper value: {gripper_val}")

    env.stop()

# Hyperparameter to control how many waypoints to skip
skip_waypoints = 0

# Path to the JSON file
# json_filepath = 'prompts/demonstrations/waypoints.json'
json_filepath = 'prompts/response_waypoints.json'

print("=======EXECUTING MAIN CODE=========")

# Execute the waypoints from the JSON file
execute_waypoints_from_json(json_filepath, skip=skip_waypoints)
