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

class video_recorder:

    def __init__(self, pos=[0.95, -0.2, 0.2], distance=2.05, yaw=-50, pitch=-40, roll=0, width=480, height=360, up=[0, 0, 1], up_axis_idx=2, near_plane=0.01, far_plane=100, fov=60, filename='static.mp4'):
        self.cam_width = width
        self.cam_height = height
        self.cam_target_pos = pos
        self.cam_distance = distance
        self.cam_yaw = yaw
        self.cam_pitch = pitch
        self.cam_roll = roll
        self.cam_up = up
        self.cam_up_axis_idx = up_axis_idx
        self.cam_near_plane = near_plane
        self.cam_far_plane = far_plane
        self.cam_fov = fov

        #video = cv2.VideoWriter('vid.avi', cv2.VideoWriter_fourcc(*'XVID'), 30, (cam_width, cam_height)) # Does not seem to support h264!
        self.vid = imageio_ffmpeg.write_frames(filename, (self.cam_width, self.cam_height), fps=30)
        self.vid.send(None) # seed the video writer with a blank frame
        
        # Set the camera properties
        p.resetDebugVisualizerCamera(self.cam_distance, self.cam_yaw, self.cam_pitch, self.cam_target_pos)
        p.getDebugVisualizerCamera()
    
    def record_frame(self):
        cam_view_matrix = p.computeViewMatrixFromYawPitchRoll(self.cam_target_pos, self.cam_distance, self.cam_yaw, self.cam_pitch, self.cam_roll, self.cam_up_axis_idx)
        cam_projection_matrix = p.computeProjectionMatrixFOV(self.cam_fov, self.cam_width*1./self.cam_height, self.cam_near_plane, self.cam_far_plane)
        image = p.getCameraImage(self.cam_width, self.cam_height, cam_view_matrix, cam_projection_matrix)[2][:, :, :3]
        # video.write(cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
        # add text to the image
        # image = self.prepare_array_for_cv2(image)
        # cv2.putText(image, f'Task: {target_pos}, Gripper:{gripper_val}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.25, (0, 0, 0), 1, cv2.LINE_AA)
        # get robot coordinates
        # linkState = p.getLinkState(kuka_id, kuka_end_effector_idx)
        # convert tuple to array 
        # linkState = np.array(linkState[4])
        # convert to 2dp 
        # linkState = np.round(linkState, 2)
        # cv2.putText(image, f'Robot: {linkState}', (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.25, (0, 0, 0), 1, cv2.LINE_AA)
        self.vid.send(np.ascontiguousarray(image))

    def close(self):
        self.vid.close()

    def prepare_array_for_cv2(array):
        # Check if the array is in RGB format and convert to BGR
        if array.ndim == 3 and array.shape[2] == 3:
            array = cv2.cvtColor(array, cv2.COLOR_RGB2BGR)

        # Ensure the datatype is uint8
        if array.dtype != np.uint8:
            # Normalize the array if it's not in the 0-255 range
            if array.max() > 1:
                array = (array / 255).astype(np.uint8)
            else:
                array = (array * 255).astype(np.uint8)
        
        return array