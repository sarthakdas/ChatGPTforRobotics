import pybullet as p
import pybullet_data
import time
import threading
import curses
import robot
import enviroment
import numpy as np
import json
import math

# Global variables
key_pressed = None
selected_joint = 7  # Initially set to the end effector joint
waypoints = []  # List to store waypoints
robot_starting_joint_pos = []

def keyboard_listener(stdscr):
    global key_pressed
    while True:
        key = stdscr.getch()
        if key != -1:
            key_pressed = key
        time.sleep(0.01)  # Sleep briefly to reduce CPU usage

def quaternion_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    return [w, x, y, z]

def control_robot(key, target_pos, target_orn, robotObj, pos_step_size, orn_step_size, stdscr):
    global selected_joint

    stdscr.addstr(2, 0, f"Key pressed: {chr(key)}   ")

    if key in range(ord('1'), ord('9') + 1):
        # If a number key (1-9) is pressed, set the selected joint
        selected_joint = key - ord('1')
        stdscr.addstr(3, 0, f"Selected joint: {selected_joint + 1}   ")
        return True
    elif key == ord('0'):
        # If '0' is pressed, set the selected joint to 10
        selected_joint = 10
        stdscr.addstr(3, 0, f"Selected joint: {selected_joint + 1}   ")
        return True
    elif key == ord('-'):
        # If '-' is pressed, set the selected joint to 11
        selected_joint = 11
        stdscr.addstr(3, 0, f"Selected joint: {selected_joint + 1}   ")
        return True

    if selected_joint is not None:
        if key == curses.KEY_UP:
            target_pos[2] += pos_step_size  # Move up
        elif key == curses.KEY_DOWN:
            target_pos[2] -= pos_step_size  # Move down
        elif key == curses.KEY_LEFT:
            target_pos[0] -= pos_step_size  # Move left
        elif key == curses.KEY_RIGHT:
            target_pos[0] += pos_step_size  # Move right
        elif key == ord(','):
            target_pos[1] -= pos_step_size  # Move y axis backward
        elif key == ord('.'):
            target_pos[1] += pos_step_size  # Move y axis forward
        elif key == ord('r'):
            reset_robot(robotObj)  # Reset the robot to the default state
        elif key == ord('z'):
            control_gripper(robotObj.panda_id, True)  # Open gripper
        elif key == ord('x'):
            control_gripper(robotObj.panda_id, False)  # Close gripper
        elif key == ord('p'):
            stdscr.addstr(4, 0, "Stopping all joints   ")
            return False  # Stop the main loop
        elif key == ord('w'):
            pitch_up = p.getQuaternionFromEuler([orn_step_size, 0, 0])
            target_orn = quaternion_multiply(target_orn, pitch_up)
        elif key == ord('s'):
            pitch_down = p.getQuaternionFromEuler([-orn_step_size, 0, 0])
            target_orn = quaternion_multiply(target_orn, pitch_down)
        elif key == ord('a'):
            yaw_left = p.getQuaternionFromEuler([0, 0, orn_step_size])
            target_orn = quaternion_multiply(target_orn, yaw_left)
        elif key == ord('d'):
            yaw_right = p.getQuaternionFromEuler([0, 0, -orn_step_size])
            target_orn = quaternion_multiply(target_orn, yaw_right)
        elif key == ord('q'):
            roll_left = p.getQuaternionFromEuler([0, orn_step_size, 0])
            target_orn = quaternion_multiply(target_orn, roll_left)
        elif key == ord('e'):
            roll_right = p.getQuaternionFromEuler([0, -orn_step_size, 0])
            target_orn = quaternion_multiply(target_orn, roll_right)

        # Move the robot to the new target position if not resetting or controlling the gripper
        if key not in [ord('r'), ord('z'), ord('x')]:
            # Print target pos to 2sf
            stdscr.addstr(5, 0, f"Target position: {[round(pos, 2) for pos in target_pos]}   ")
            stdscr.addstr(6, 0, f"Target orientation: {[round(orn, 2) for orn in target_orn]}   ")
            robotObj.robot_control(target_pos, target_orn=[0, 1.01*math.pi, 0], joint_id=6)
    stdscr.refresh()
    return True  # Continue the main loop

def reset_robot(robotObj):
    robotId = robotObj.panda_id
    num_joints = p.getNumJoints(robotId)
    default_joint_positions = [0.0] * num_joints

    for joint in range(num_joints):
        p.resetJointState(robotId, joint, targetValue=default_joint_positions[joint])
    print("Robot reset to default state")

def control_gripper(robotId, open_grip):
    gripper_joint_indices = [9, 10]  # Indices for the gripper joints
    gripper_positions = [0.04, 0.04] if open_grip else [0.0, 0.0]  # Open or close gripper
    for index, pos in zip(gripper_joint_indices, gripper_positions):
        p.setJointMotorControl2(robotId, index, p.POSITION_CONTROL, targetPosition=pos)
    print("Gripper", "opened" if open_grip else "closed")

def capture_waypoints(robotObj, waypoints):
    end_effector_index = 11
    pos, orn = p.getLinkState(robotObj.panda_id, end_effector_index)[4:6]
    gripper_state = p.getJointState(robotObj.panda_id, 9)[0]  # Assume both gripper fingers move together
    waypoints.append((time.time(), [round(coord, 2) for coord in pos], [round(q, 2) for q in orn], round(gripper_state, 2)))

def save_waypoints_to_json(waypoints, robot_starting_joint_pos, objects):
    data = {
        'demonstration_name': 'FILL_ME_IN',
        'starting_joint_pos': [round(pos, 2) for pos in robot_starting_joint_pos],
        'objects': objects,
        'demonstration': []
    }
    for index, waypoint in enumerate(waypoints):
        data['demonstration'].append({
            'index': index,
            'time_step': round(waypoint[0], 2),
            'xyz_position': list(waypoint[1]),
            'quaternion_orientation': list(waypoint[2]),
            'gripper_state': waypoint[3]
        })
    
    with open('data/demonstrations/waypoints.json', 'w') as jsonfile:
        json.dump(data, jsonfile, indent=4)

def set_robot_end_effector(robotObj, position, orientation):
    robotId = robotObj.panda_id
    joint_angles = p.calculateInverseKinematics(robotId, 7, position, orientation)
    num_joints = min(len(joint_angles), p.getNumJoints(robotId))
    for i in range(num_joints):
        p.resetJointState(robotId, i, joint_angles[i])
    p.stepSimulation()

def main(stdscr):
    env = enviroment.TableTopEnv(display=True, objects={'orange_cup': None, 'green_cup': None, 'blue_cup': None})
    global key_pressed, waypoints, robot_starting_joint_pos
    # Clear screen
    stdscr.clear()
    stdscr.nodelay(True)  # Make getch() non-blocking

    # Start the keyboard listener thread
    listener_thread = threading.Thread(target=keyboard_listener, args=(stdscr,))
    listener_thread.daemon = True
    listener_thread.start()

    # Load plane and robot
    # planeId = p.loadURDF("plane.urdf")
    robotObj = env.robot
    robotId = robotObj.panda_id

    # Set gravity
    p.setGravity(0, 0, -9.8)

    # Get the number of joints in the robot
    num_joints = p.getNumJoints(robotId)
    stdscr.addstr(0, 0, f"Number of joints in robot: {num_joints}")
    stdscr.refresh()

    # End effector link index
    end_effector_index = 11  # Franka Panda's end effector link index

    # Step size for changing positions and orientations
    pos_step_size = 0.01
    orn_step_size = 0.05

    # Define initial waypoint for the end effector
    initial_waypoint_position = robotObj.get_end_effector_position()
    # convert to list
    initial_waypoint_position = list(initial_waypoint_position)

    # Orientation quaternion for forward-facing gripper (rotated 90 degrees around X-axis)
    initial_waypoint_orientation = robotObj.get_end_effector_orientation() # Rotate 90 degrees around X-axis

    # save initial orientation to json file
    
    # convert to list
    initial_waypoint_orientation = list(initial_waypoint_orientation)

    # Store initial joint positions
    robot_starting_joint_pos = robotObj.get_joint_positions()
    # Save the initial joint positions to a json file
    with open('joint_pos.json', 'w') as jsonfile:
        json.dump({'Starting Joint Positions': [round(pos, 2) for pos in robot_starting_joint_pos]}, jsonfile, indent=4)

    # Store initial position
    capture_waypoints(robotObj, waypoints)

    # Main loop
    running = True
    last_capture_time = time.time()
    objects = env.get_object_positions()
    while running:
        # Step the simulation
        # env.capture_frames()
        p.stepSimulation()

        # Capture waypoints at 4 Hz
        current_time = time.time()
        if current_time - last_capture_time >= 0.25:  # Capture waypoints every 0.25 seconds
            capture_waypoints(robotObj, waypoints)
            last_capture_time = current_time

        # Check if a key was pressed
        if key_pressed is not None:
            running = control_robot(key_pressed, initial_waypoint_position, initial_waypoint_orientation, robotObj, pos_step_size, orn_step_size, stdscr)
            key_pressed = None

        # Sleep to make the simulation real-time
        time.sleep(1./240.)

    # Save waypoints to JSON
    save_waypoints_to_json(waypoints, robot_starting_joint_pos, objects)

    # Disconnect from the physics server
    p.disconnect()

# Start the curses application
curses.wrapper(main)
