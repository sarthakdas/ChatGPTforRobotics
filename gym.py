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
    target_pos[2] += 0.5

    # go to position slowly move to target
    go_to_position(target_pos, 0.05, 50)

    # wait for 50 steps
    wait(10)

def place_cube(cube_id, target_pos):
    # go to position
    go_to_position(target_pos, 0.05, 25)
    wait(10)

    # open gripper
    go_to_position(target_pos, 0)
    wait(10)

def move_cube(cube_id, target_pos):
    '''get the cube position and move above it then use the command pick up cube then move to target position and place cube'''
    # get cube position
    cube_pos, _ = env.get_object_position(cube_id)

    # add 32 cm to z for gripper to be above the cube
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

def wait(steps=50):
    for _ in range(steps):
        env.time_step()


# Robots gripper length is 0.32

# get the cube position
cube_pos, _ = env.get_object_position(env.cube_id)
target_pos = [cube_pos[0], cube_pos[1]+0.4, cube_pos[2]]

p.addUserDebugLine(cube_pos, target_pos, [1, 0, 0], 5)

# Use function to move the cube
move_cube(env.cube_id, target_pos)

# move the robot arm up from the cubes current position 
target_pos[2] += 0.5
go_to_position(target_pos, 0, 25)
wait(25)

env.stop()