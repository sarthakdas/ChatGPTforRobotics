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

# get cube position 
cube_pos, _ = p.getBasePositionAndOrientation(env.cube_id)

# add 32 cm to z for gripper to be above the cube
target_pos = [cube_pos[0], cube_pos[1], cube_pos[2] + 0.32]


# go to position
go_to_position(target_pos, 0)
wait(25)
# close gripper
go_to_position(target_pos, 0.05)
wait(25)

# lift cube
target_pos[2] += 0.5

# go to position slowly move to target
go_to_position(target_pos, 0.05, 50)

# wait for 50 steps
wait(50)

# open gripper
go_to_position(target_pos, 0)

wait(50)




env.stop()