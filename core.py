import math

def euler_to_quaternion(roll, pitch, yaw):
    """
    Convert Euler angles (in radians) to quaternion.
    """
    
    qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
    qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
    qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
    qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
    
    return [qx, qy, qz, qw]

def quaternion_to_euler(x, y, z, w):
    """
    Convert quaternion to Euler angles (in radians).
    output: roll_x, pitch_y, yaw_z
    """
    
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z

def degrees_to_radians(degrees):
    """
    Convert degrees to radians.
    """
    return degrees * math.pi / 180

def radians_to_degrees(radians):
    """
    Convert radians to degrees.
    """
    return radians * 180 / math.pi

# for i in range(5):
#     roll = degrees_to_radians(180 + (i * 45))
#     pitch = degrees_to_radians(90)
#     yaw = degrees_to_radians(90)
    
#     target_orn = euler_to_quaternion(roll, pitch, yaw)
#     target_orn_euler = quaternion_to_euler(target_orn[0], target_orn[1], target_orn[2], target_orn[3])
#     target_orn_euler_degrees = [radians_to_degrees(target_orn_euler[0]), radians_to_degrees(target_orn_euler[1]), radians_to_degrees(target_orn_euler[2])]
    
#     print(f"Iteration {i}:")
#     print(f"Original Euler angles (degrees): Roll: {180 + (i * 45)}, Pitch: 90, Yaw: 90")
#     print(f"Quaternion: {target_orn}")
#     print(f"Converted back to Euler angles (radians): {target_orn_euler}")
#     print(f"Converted back to Euler angles (degrees): {target_orn_euler_degrees}")
#     print("-" * 50)
