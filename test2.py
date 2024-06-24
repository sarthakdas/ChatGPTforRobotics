import pybullet as p
import pybullet_data
import time
from robot import PandaRobot
import math


def setup_environment():
    # Connect to the physics server
    physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version

    # Set the gravity for the environment
    p.setGravity(0, 0, -9.8)

    # Load the plane URDF file (for a ground plane)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    planeId = p.loadURDF("plane.urdf")

    # Set the simulation time step
    p.setTimeStep(1. / 240.)

    return physicsClient

if __name__ == "__main__":
    physicsClient = setup_environment()
    panda_robot = PandaRobot()
    panda_robot.rotate_joints()
    
    # Keep the simulation running
    while True:
        p.stepSimulation()
        time.sleep(1. / 240.)