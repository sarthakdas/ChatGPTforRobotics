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


class KukaRobot:

    def __init__(self, tool="suction", basePosition=[1.400000, -0.200000, 0.600000], baseOrientation=[0.000000, 0.000000, 0.000000, 1.000000]):
        self.kuka_id = p.loadURDF("kuka_iiwa/model_vr_limits.urdf", basePosition=basePosition, baseOrientation=baseOrientation)

        jointPositions = [-0.000000, -0.000000, 0.000000, 1.570793, 0.000000, -1.036725, 0.000001]
        for jointIndex in range(p.getNumJoints(self.kuka_id)):
            p.resetJointState(self.kuka_id, jointIndex, jointPositions[jointIndex])
            p.setJointMotorControl2(self.kuka_id, jointIndex, p.POSITION_CONTROL, jointPositions[jointIndex], 0)

        if tool == "gripper":
            self.kuka_gripper_id = p.loadSDF("gripper/wsg50_one_motor_gripper_new_free_base.sdf")[0]

            # attach gripper to kuka arm
            self.kuka_cid = p.createConstraint(self.kuka_id, 6, self.kuka_gripper_id, 0, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0.05], [0, 0, 0])
            self.kuka_cid2 = p.createConstraint(self.kuka_gripper_id, 4, self.kuka_gripper_id, 6, jointType=p.JOINT_GEAR, jointAxis=[1,1,1], parentFramePosition=[0,0,0], childFramePosition=[0,0,0])
            p.changeConstraint(self.kuka_cid2, gearRatio=-1, erp=0.5, relativePositionTarget=0, maxForce=100)

            # reset kuka


            # reset gripper
            p.resetBasePositionAndOrientation(self.kuka_gripper_id, [0.923103, -0.200000, 1.250036], [-0.000000, 0.964531, -0.000002, -0.263970])
            jointPositions = [0.000000, -0.011130, -0.206421, 0.205143, -0.009999, 0.000000, -0.010055, 0.000000]
            for jointIndex in range(p.getNumJoints(self.kuka_gripper_id)):
                p.resetJointState(self.kuka_gripper_id, jointIndex, jointPositions[jointIndex])
                p.setJointMotorControl2(self.kuka_gripper_id, jointIndex, p.POSITION_CONTROL, jointPositions[jointIndex], 0)
            
            self.kuka_end_effector_idx = 6
        
        elif tool == "suction":
            
            # Treat last link as end effector
            self.n_joints = p.getNumJoints(self.kuka_id)
            self.kuka_gripper_id = None
            self.kuka_end_effector_idx = 6
            self.kuka_cid = None
            self.suction = False
        else:
            raise ValueError(f"Unknown tool: {tool}")
        
        self.tool = tool
        self.n_joints = p.getNumJoints(self.kuka_id)
        
        print(f'KukaRobot: {self.n_joints} joints')
        
    def kuka_control(self, target_pos, gripper_val):
        '''Move the robot to the target position and set the gripper value. Input: target position and gripper value. Output: None.'''
        target_orn = p.getQuaternionFromEuler([0, 1.01*math.pi, 0])

        joint_poses = p.calculateInverseKinematics(self.kuka_id, self.kuka_end_effector_idx, target_pos, target_orn)
        for j in range (self.n_joints):
            p.setJointMotorControl2(bodyIndex=self.kuka_id, jointIndex=j, controlMode=p.POSITION_CONTROL, targetPosition=joint_poses[j], targetVelocity=0.001)

        if self.tool == "gripper":
            p.setJointMotorControl2(self.kuka_gripper_id, 4, p.POSITION_CONTROL, targetPosition=gripper_val, force=100)
            p.setJointMotorControl2(self.kuka_gripper_id, 6, p.POSITION_CONTROL, targetPosition=gripper_val, force=100)
        elif self.tool == "suction":
            if gripper_val == 0:
                self.suction = False
            if gripper_val == 1:
                self.suction = True


    def get_end_effector_force(self):
        '''Return the force of the end effector. Output: end effector force vector.'''
        return p.getJointState(self.kuka_id, self.kuka_end_effector_idx)[2]

    def get_end_effector_position(self):
        '''Return the position of the end effector. Output: end effector position.'''
        # print(p.getLinkState(self.kuka_id, self.kuka_end_effector_idx)[0])
        return p.getLinkState(self.kuka_id, self.kuka_end_effector_idx)[0]

    def get_joint_position(self, joint_id):
        '''Return the position of the joint. Input: joint id. Output: joint position.'''
        # return joint cartesian position
        return p.getLinkState(self.kuka_id, joint_id)[0]
        
