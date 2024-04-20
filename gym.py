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
import enviroment



env = enviroment.tableTopEnv()

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


# ========================
# ========================
print("========================")

def arrange_cubes_in_line():
    # Fetch the cube information
    cubes = object_name_to_id_dic()
    print("Cube information:", cubes)
    
    # Set the target positions
    start_x = 1.1  # Starting x position for the first cube
    increment_x = 0.1  # Distance between each cube
    table_height = 0.65
    safe_height_above_cube = 0.1
    safe_height_above_table = 0.15  # Safe height to drop the cube
    gap = 0.1  # Gap between cubes

    # Iterate over each cube to move them
    for index, (cube_name, cube_id) in enumerate(cubes.items()):
        print(f"Moving {cube_name}")
        # Set task description
        env.main_cam.task = f"Locating {cube_name}"
        
        # Get current position of the cube
        cube_pos = get_object_position(cube_id)
        print(f"Current position of {cube_name}:", cube_pos)
        
        # Move to cube position at safe height
        target_pos_above_cube = [cube_pos[0], cube_pos[1], cube_pos[2] + safe_height_above_cube]
        target_pos_above_cube_safe = [cube_pos[0], cube_pos[1], cube_pos[2] + 2 * safe_height_above_cube]
        go_to_position(target_pos_above_cube, gripper_val=0)
        wait(5)  # Wait for gripper stabilization
        
        # Move down to grip the cube
        go_to_position(target_pos_above_cube, gripper_val=0)
        wait(5)  # Wait for stabilization
        go_to_position(target_pos_above_cube, gripper_val=1)  # Close gripper
        wait(5)  # Wait for gripper to close
        
        # Lift the cube up to safe height
        go_to_position(target_pos_above_cube_safe, gripper_val=1)
        
        # Calculate new position in the line
        target_x = start_x - index * (gap + 0.07)  # Position adjusted by cube size and gap
        target_pos_line = [target_x, 0, table_height + safe_height_above_table]
        env.main_cam.task = f"Placing {cube_name} at new position"
        
        # Move to new position above the target
        go_to_position([target_x, 0, cube_pos[2] + 2 * safe_height_above_cube], gripper_val=1)
        
        # Lower cube to new position
        go_to_position(target_pos_line, gripper_val=1)
        wait(5)
        go_to_position(target_pos_line, gripper_val=0)  # Open gripper
        wait(5)  # Wait for gripper to open
        
        # Move back to safe height
        go_to_position([target_x, 0, cube_pos[2] + safe_height_above_cube], gripper_val=0)
    
    # Return to original safe position after all movements
    go_to_position([1.40, 0.0, 0.1], gripper_val=0)  # Return to starting position
    env.main_cam.task = "Task completed"

# Call the function to arrange the cubes
arrange_cubes_in_line()


# ========================
# ========================



env.stop()