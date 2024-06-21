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
<<<<<<< Updated upstream
=======
import random
import keyboard
import json
>>>>>>> Stashed changes

import camera
import enviroment
import gpt

<<<<<<< Updated upstream
=======
# OpenAI API key from env
openai_api_key = os.getenv('OPENAI_API_KEY')


# Initialize environment and code generator
>>>>>>> Stashed changes
env = enviroment.tableTopEnv()
code_gen = gpt.OpenAIClient(api_key=openai_api_key)

def go_to_position(target_pos, gripper_val, time=10):
    '''move the robot to the target position and set the gripper value in time steps,
    gripper open = 0, gripper close = 1
    . Input: target position, gripper value and time. Output: None.'''
    # linear interpolate to target position in time steps
    # get current position
    pos = env.robot.get_end_effector_position()
    # print("target_pos", target_pos)
    # print("pos", pos)
    for i in range(time):
        # calculate new position for time step, interpolate between current and target position, current state is a tuple of position and orientation
        new_pos = (pos[0] + (target_pos[0] - pos[0]) * i / time, pos[1] + (target_pos[1] - pos[1]) * i / time, pos[2] + (target_pos[2] - pos[2]) * i / time)
        # move to new position
        # print(new_pos)
        env.time_sequence(new_pos, gripper_val)
        # # wait for time
        # env.time_step()

def object_name_to_id_dic():
    '''get the object dictionary and return the object names and ids. Output: object dictionary where the colour_name is the key and object_id is the value
    e.g {'yellow block': 5, 'pink block': 6, 'orange block': 7}'''
    # get the object list
    object_list = env.obj_name_to_id 
    return object_list

def get_object_position(obj_id):
    '''get the object position and return the object position. Input: object id. Output: object position [X,Y,Z].'''
    # get the object position
    pos, _ = env.get_object_position(obj_id)
    return pos

def wait(steps=10):
    for _ in range(steps):
        env.time_step()

<<<<<<< Updated upstream

# ========================
# ========================
# get prompt from user and generate code
# get objects in the scene
objects = object_name_to_id_dic()
# get the postions of all the objects using a for loop and save to dictionary
object_positions = {}
for key in objects.keys():
    object_positions[key] = get_object_position(objects[key])
prompt = code_gen.process(object_positions)

with open("prompts/response.py", 'r') as file:
    code = file.read()


print("=======EXECUTING GPT CODE=========")
# read code form response.py
exec(code)


# go_to_position([0.85, -0.20, 0.65], 0, 10)
# go_to_position([0.85, -0.20, 0.65], 1, 10)
# go_to_position([0.85, -0.20, 1], 1, 10)
=======
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
>>>>>>> Stashed changes


<<<<<<< Updated upstream
# ========================
# ========================



env.stop()
=======
# Execute the waypoints from the JSON file
execute_waypoints_from_json(json_filepath, skip=skip_waypoints)
>>>>>>> Stashed changes
