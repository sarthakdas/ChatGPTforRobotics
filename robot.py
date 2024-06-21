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
import urdf_models.models_data as md

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
        
<<<<<<< Updated upstream
=======
    def adjust_target_for_gripper(self,target_pos, gripper_length, end_effector_orn):
        """
        Adjust the target position to account for the gripper length.
        
        Parameters:
        target_pos (list): The desired target position [x, y, z].
        gripper_length (float): The length of the gripper.
        end_effector_orn (list): The orientation of the end effector as a quaternion [qx, qy, qz, qw].
        
        Returns:
        list: The adjusted target position [x, y, z].
        """
        # Convert quaternion to rotation matrix
        rot_matrix = p.getMatrixFromQuaternion(end_effector_orn)
        rot_matrix = np.array(rot_matrix).reshape(3, 3)
        
        # Define the gripper offset in the local frame
        gripper_offset_local = np.array([0, 0, -gripper_length])
        
        # Transform the gripper offset to the world frame
        gripper_offset_world = np.dot(rot_matrix, gripper_offset_local)
        
        # Adjust the target position
        adjusted_target_pos = np.array(target_pos) + gripper_offset_world
        
        return adjusted_target_pos.tolist()
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        Convert Euler angles (in degrees) to quaternion.
        """
        roll = math.radians(roll)
        pitch = math.radians(pitch)
        yaw = math.radians(yaw)
        
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        
        return [qx, qy, qz, qw]



class PandaRobot: 
    def __init__(self, tool="suction", basePosition=[1.400000, -0.200000, 0.600000], baseOrientation=[0.000000, 0.000000, 0.000000, 1.000000], jointPositions=[-0.45556,-1.86304,-1.76341, -0.90334,-1.67523,-1.29374,0.28787]):
        self.panda_id = p.loadURDF("franka_panda/panda.urdf", basePosition=basePosition, baseOrientation=baseOrientation, useFixedBase=True)
        self.n_joints = 7
        self.tool = tool
        self.end_effector_id = 11
        self.gripper_joints = [9, 10]

        for jointIndex in range(self.n_joints):
            p.resetJointState(self.panda_id, jointIndex, jointPositions[jointIndex])
            p.setJointMotorControl2(self.panda_id, jointIndex, p.POSITION_CONTROL, jointPositions[jointIndex], 0)
    
    def set_joint_positions(self, joint_positions):
        '''Set the joint positions. Input: joint positions. Output: None.'''
        for jointIndex in range(self.n_joints):
            p.resetJointState(self.panda_id, jointIndex, joint_positions[jointIndex])
            p.setJointMotorControl2(self.panda_id, jointIndex, p.POSITION_CONTROL, joint_positions[jointIndex], 0)
        
    def robot_control(self, target_pos, target_orn = [0,0,0,1], gripper_val = 0.04, joint_id = None):
        '''Move the robot to the target position and set the gripper value. Input: target position and gripper value. Output: None.'''
        if joint_id is None:
            joint_id = self.end_effector_id
        
        # print("target_orientation: ", target_orn)
        joint_angles = p.calculateInverseKinematics(self.panda_id, joint_id, target_pos, target_orn)
        for i in range(self.n_joints):
            p.setJointMotorControl2(self.panda_id, i, p.POSITION_CONTROL, targetPosition=joint_angles[i])

        self.control_gripper(gripper_val)

    def control_gripper(self, value):
        '''Control the gripper. Input: value (0 to close, 1 to open). Output: None.'''
        for joint in self.gripper_joints:
            p.setJointMotorControl2(bodyIndex=self.panda_id, jointIndex=joint, controlMode=p.POSITION_CONTROL, targetPosition=value, force=100)


    def get_end_effector_force(self):
        '''Return the force of the end effector. Output: end effector force vector.'''
        return p.getJointState(self.panda_id, self.panda_gripper_id)[2]

    def get_end_effector_position(self):
        '''Return the position of the end effector. Output: end effector position.'''
        return p.getLinkState(self.panda_id, self.end_effector_id)[0]
        
    
    def get_end_effector_orientation(self):
        '''Return the orientation of the end effector. Output: end effector orientation.'''
        return p.getLinkState(self.panda_id, self.end_effector_id)[1]

    def get_joint_position(self, joint_id):
        '''Return the position of the joint. Input: joint id. Output: joint position.'''
        # return joint cartesian position
        return p.getLinkState(self.panda_id, joint_id)[0]

    def get_joint_positions(self):
        '''Return the positions of all joints. Output: joint positions.'''
        joint_positions = []
        for j in range(11):
            # p.getJointState(self.panda_id, j)[0] returns the joint position value as a float number in a tuple of x, y, z
            joint_positions.append(p.getJointState(self.panda_id, j)[0])
        return joint_positions
        
    def get_joint_positions_xyz(self):
        """
        Returns a list where each index corresponds to a joint ID and the value is a tuple containing
        the XYZ world coordinates of that joint.
        """
        num_joints = p.getNumJoints(self.panda_id)
        link_indices = list(range(num_joints))
        link_states = p.getLinkStates(self.panda_id, link_indices, computeForwardKinematics=1)

        joint_positions = [link_state[4] for link_state in link_states]  # [4] is the world position
        # convert to list
        joint_positions = [list(joint_position) for joint_position in joint_positions]
        return joint_positions

    def adjust_target_for_gripper(self,target_pos, gripper_length, end_effector_orn):
        """
        Adjust the target position to account for the gripper length.
        
        Parameters:
        target_pos (list): The desired target position [x, y, z].
        gripper_length (float): The length of the gripper.
        end_effector_orn (list): The orientation of the end effector as a quaternion [qx, qy, qz, qw].
        
        Returns:
        list: The adjusted target position [x, y, z].
        """
        # Convert quaternion to rotation matrix
        rot_matrix = p.getMatrixFromQuaternion(end_effector_orn)
        rot_matrix = np.array(rot_matrix).reshape(3, 3)
        
        # Define the gripper offset in the local frame
        gripper_offset_local = np.array([0, 0, -gripper_length])
        
        # Transform the gripper offset to the world frame
        gripper_offset_world = np.dot(rot_matrix, gripper_offset_local)
        
        # Adjust the target position
        adjusted_target_pos = np.array(target_pos) + gripper_offset_world
        
        return adjusted_target_pos.tolist()
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        Convert Euler angles (in degrees) to quaternion.
        """
        roll = math.radians(roll)
        pitch = math.radians(pitch)
        yaw = math.radians(yaw)
        
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        
        return [qx, qy, qz, qw]
>>>>>>> Stashed changes
