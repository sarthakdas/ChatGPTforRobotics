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


def pick_up_cube(cube_id):
    '''get the cube position and move above it then close the gripper and lift the cube up'''
    # get cube position 
    cube_pos, _ = env.get_object_position(cube_id)

    # add 32 cm to z for gripper to be above the cube
    target_pos = [cube_pos[0], cube_pos[1], cube_pos[2] + 0.32]

    # go to position
    go_to_position(target_pos, 0)
    wait(10)

    # close gripper
    go_to_position(target_pos, 0.05)
    wait(10)

    # lift cube
    target_pos[2] += 0.3

    # go to position slowly move to target
    go_to_position(target_pos, 0.05, 50)

    # wait for 50 steps
    wait(10)

def place_cube(target_pos):
    ''' get the target position and move above it then open the gripper and place the cube'''
    # go to position
    go_to_position(target_pos, 0.05, 25)
    wait(10)

    # open gripper
    go_to_position(target_pos, 0)
    wait(10)

def move_cube(cube_id, target_pos):
    '''get the cube position and move above it then use the command pick up cube then move to target position [x,y,z] and place cube'''
    # get cube position
    cube_pos, _ = env.get_object_position(cube_id)

    # go to position above cube
    above_cube = [cube_pos[0], cube_pos[1], cube_pos[2] + 0.5]

    # go to position
    go_to_position(above_cube, 0)
    wait(10)

    # pick up cube
    pick_up_cube(cube_id)

    # add 32 cm to z for gripper to be above the target position
    target_pos[2] += 0.32

    # place cube
    place_cube(cube_id, target_pos)


def go_to_position(target_pos, gripper_val, time=10):
    '''move the robot to the target position and set the gripper value in time steps,
    gripper open = 0, gripper close = 0.05
    . Input: target position, gripper value and time. Output: None.'''
    # linear interpolate to target position in time steps
    # get current position
    pos = env.robot.get_end_effector_position()
    for i in range(time):
        # calculate new position for time step
        new_pos = [pos[0] + (target_pos[0] - pos[0]) / time * i, pos[1] + (target_pos[1] - pos[1]) / time * i, pos[2] + (target_pos[2] - pos[2]) / time * i]
        # move to new position
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

def wait(steps=50):
    for _ in range(steps):
        env.time_step()


# ========================
# ========================
print("========================")


def move_cubes_to_square(center_x, center_y, z_height=0.65):
    # Get the dictionary of all cubes, where keys are color names and values are cube IDs
    cubes = object_name_to_id_dic()
    print("Retrieved object list from the environment. Available cubes:", cubes)

    # Ensure there are at least four cubes to form a square
    if len(cubes) < 4:
        print("Not enough cubes to form a square.")
        return

    # Define the side length of the square in meters
    side_length = 0.5  # Side length of the square

    # Square positions assuming cubes are moved clockwise starting from the bottom left corner
    square_positions = [
        [center_x - side_length / 2, center_y - side_length / 2, z_height],  # Bottom left
        [center_x + side_length / 2, center_y - side_length / 2, z_height],  # Bottom right
        [center_x + side_length / 2, center_y + side_length / 2, z_height],  # Top right
        [center_x - side_length / 2, center_y + side_length / 2, z_height],  # Top left
    ]

    # Iterate over the first four cubes in the dictionary and their target positions
    count = 0
    for color_name, cube_id in cubes.items():
        if count >= 4:
            break
        target_pos = square_positions[count]
        print(f"Moving {color_name} cube (ID {cube_id}) from current position to {target_pos}")

        # Move the cube to the new position
        move_cube_custom(cube_id, target_pos)
        count += 1

    print("All selected cubes have been moved to form a square.")

def move_cube_custom(cube_id, target_pos):
    '''Move the cube from its current position to the specified target position.'''
    print(f"Retrieving current position for cube ID {cube_id}")
    # Get the current position of the cube
    cube_pos = get_object_position(cube_id)

    # Move to position directly above the cube
    above_cube_pos = [cube_pos[0], cube_pos[1], cube_pos[2] + 0.5]
    print(f"Moving to position above cube ID {cube_id} at {above_cube_pos}")
    env.main_cam.task = f"Moving to position above cube ID {cube_id}"
    go_to_position(above_cube_pos, 0)
    wait(10)

    # Pick up the cube
    print(f"Picking up cube ID {cube_id}")
    env.main_cam.task = f"Picking up cube ID {cube_id}"
    pick_up_cube(cube_id)

    # Adjust the target position to the specified Z height of 0.65 meters for placement
    target_pos_with_height = [target_pos[0], target_pos[1], target_pos[2] + 0.5]
    print(f"Moving cube ID {cube_id} to new position at {target_pos_with_height}")
    env.main_cam.task = f"Moving cube ID {cube_id} to new position"
    go_to_position(target_pos_with_height, 0.05)
    wait(10)

    # Place the cube at the target position
    print(f"Placing cube ID {cube_id} at {target_pos_with_height}")
    env.main_cam.task = f"Placing cube ID {cube_id}"
    place_cube(target_pos_with_height)

# Example of calling the function with a specified center (2.0, 2.0)
move_cubes_to_square(1, 0)



# ========================
# ========================



env.stop()