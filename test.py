import pybullet as p
import pybullet_data
import time
import robot as r 
import math

# Initialize simulation
physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # path to pybullet data
p.setGravity(0, 0, -9.8)
planeId = p.loadURDF("plane.urdf")
p.setTimeStep(1./240.)

# Initialize Panda robot
panda = r.PandaRobot(tool="gripper")

panda.rotate_joint(6)

# Additional control logic with specified time steps
for t in range(1000):
    if t < 150:
        target_pos, gripper_val = [0.85, -0.5, 0.97], 1
    elif t >= 150 and t < 250:
        target_pos, gripper_val = [0.85, -0.5, 0.97], 0  # grab object
    elif t >= 250 and t < 400:
        target_pos, gripper_val = [0.85, -0.5, 0.97], 0  # move up after picking object
    elif t >= 400 and t < 600:
        target_pos, gripper_val = [0.85, -0.2 + 0.4 * (t - 400) / 200, 1.1], 0  # move to target position
    elif t >= 600 and t < 700:
        target_pos, gripper_val = [0.85, 0.2, 1.1], 0  # stop at target position
    elif t >= 700:
        target_pos, gripper_val = [0.85, 0.2, 1.1], 1 # drop object
    print(f"Time step: {t}, target position: {target_pos}, gripper value: {gripper_val}")
    target_orn = panda.euler_to_quaternion(0, 180, 0)

    # print out joint positions to 2 dp for debugging
    joint_states = panda.get_joint_positions()
    
    print(f"Joint states: {joint_states}")
    panda.set_joint_position(6, 0)
    panda.robot_control(target_pos=target_pos, target_orn=target_orn, gripper_val=gripper_val, joint_id=7)

    p.stepSimulation()
    time.sleep(1./240.)

panda.rotate_joint(6)
# Disconnect from simulation
p.disconnect()
