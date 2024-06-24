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
import gpt

import camera
import enviroment
import gpt
import core
import robotController
from dotenv import load_dotenv

# Initialize environment and code generator


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
json_filepath = 'data/demonstrations/compressed/waypoints1.json'





print("=======EXECUTING MAIN CODE=========")

# # # get all the file paths in the directory data/demonstrations/compressed/
# for filename in os.listdir('data/demonstrations/compressed/'):
#     if filename.endswith('.json'): 
#         json_filepath = os.path.join('data/demonstrations/compressed/', filename)

#         # open the filepath and get the object dictionary
#         with open(json_filepath, 'r') as f:
#             data = json.load(f)
#             objects = data["objects"]

#         print(f"Executing waypoints from file: {json_filepath} with objects: {objects}")
#         print(objects)

#         env = enviroment.TableTopEnv(display=False, objects=objects)
#         load_dotenv()
#         openai_api_key = os.getenv("OPENAI_API_KEY")
#         code_gen = gpt.OpenAIClient(api_key=openai_api_key)

#         execute_waypoints_from_json(json_filepath, skip=skip_waypoints)

#         # rename static.mp4 to the filename
#         os.rename('static.mp4', f'{filename}.mp4')
        

# given an array of waypoints go to each waypoint

    
waypoints = [[0.98, -0.2, 0.92], [0.98, -0.2, 0.92], [0.98, -0.2, 0.92], [1.04, -0.16, 0.8], [1.09, -0.16, 0.75], [1.1, -0.17, 0.73], [1.12, -0.19, 0.71], [1.13, -0.2, 0.68], [1.13, -0.2, 0.66], [1.13, -0.21, 0.66], [1.13, -0.21, 0.65], [1.13, -0.22, 0.65], [1.12, -0.22, 0.65], [1.1, -0.22, 0.65], [1.07, -0.22, 0.64], [1.03, -0.22, 0.63], [0.99, -0.22, 0.63], [0.97, -0.22, 0.62], [0.95, -0.22, 0.62], [0.94, -0.22, 0.62], [0.93, -0.22, 0.62]]
objects = {'orange_cup': None, 'green_cup': None, 'blue_cup': None}
# load the environment
env = enviroment.TableTopEnv(display=False, objects=objects)

# get the object dictionary
objects = env.get_object_positions()
# delete the second element (rotation) from the object dictionary
for obj in objects:
    objects[obj] = objects[obj]["position"]
# print(objects)

context_description = "You are a robot and you are to genereate 4 possible waypoint paths that you can do based on the input: "
scene_objects = objects
instruction = "Generate 4 possible waypoint paths that you can do based on the objects in the scene: "

# get api from .env 
load_dotenv()
api_key = os.getenv('OPENAI_API_KEY')

print("API Key: ", api_key, type(api_key))

client = gpt.OpenAIClient(api_key=api_key)
client.process(instruction, scene_objects, context_description)


for i,waypoint in enumerate(waypoints):
        go_to_position(waypoint, [-0.1, 1, 0.1, 0.3], 0.04, duration=1)
        print(f"Moved to position: {waypoint}, index: {i} out of {len(waypoints)}")
env.stop()