def place_lemon_on_plate():
    # Fetch the object information
    objects = object_name_to_id_dic()
    print("Object information:", objects)
    
    # Get the IDs of the lemon and plate
    lemon_id = objects['plastic_lemon']
    plate_id = objects['plate']
    
    # Set the target height above the plate
    height_above_plate = 0.5  # 10cm above the plate
    
    # Get the current position of the plate
    plate_pos = get_object_position(plate_id)
    print("Current position of the plate:", plate_pos)
    
    # Move to the lemon position at safe height
    lemon_pos = get_object_position(lemon_id)
    target_pos_above_lemon = [lemon_pos[0], lemon_pos[1], lemon_pos[2]+0.1]
    go_to_position(target_pos_above_lemon, gripper_val=0)
    wait(5)  # Wait for gripper stabilization
    env.main_cam.task = "Moving to the lemon position"

    # Move down to grip the lemon
    go_to_position(target_pos_above_lemon, gripper_val=0)
    wait(10)  # Wait for stabilization

    env.main_cam.task = "Gripping the lemon"
    go_to_position(target_pos_above_lemon, gripper_val=1)  # Close gripper
    wait(10)  # Wait for gripper to close
    
    # Lift the lemon up to safe height
    env.main_cam.task = "Lifting the lemon"
    target_pos_above_plate = [plate_pos[0], plate_pos[1], plate_pos[2] + height_above_plate]
    go_to_position(target_pos_above_plate, gripper_val=1)
    
    # Move to the top of the plate
    go_to_position([plate_pos[0], plate_pos[1], plate_pos[2] + 0.2], gripper_val=1)
    
    # Lower lemon to the top of the plate
    go_to_position([plate_pos[0], plate_pos[1], plate_pos[2] + 0.2], gripper_val=0)
    wait(5)
    go_to_position(target_pos_above_plate, gripper_val=0)  # Open gripper
    wait(5)  # Wait for gripper to open
    
    # Move back to safe height above the lemon
    go_to_position(target_pos_above_lemon, gripper_val=0)
    
    # Return to original safe position
    go_to_position([0.923103, -0.200000, 1.250036], gripper_val=0)
    env.main_cam.task = "Task completed"

# Call the function to place the lemon on the plate
place_lemon_on_plate()