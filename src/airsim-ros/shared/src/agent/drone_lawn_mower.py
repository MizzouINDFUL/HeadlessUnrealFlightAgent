import airsim
import os
import math
import yaml
import sys
import time
import socket

# lawn mower pattern parameters
grid_spacing = 35  # Distance between grid lines
speed = 25  # Movement speed in m/s
height = 35  # Fixed altitude for the grid search, adjust as needed
adjust_yaw = False  # Adjust yaw to face the next target point

# Check if enough arguments are provided
if len(sys.argv) < 3:
    print("Not enough arguments provided.")
    sys.exit(1)

config_path = "/config.yml"
if len(sys.argv) > 3:
    config_path = sys.argv[3]

print("Config file: " + config_path)

print("Behavior of this agent: drone will get up and start in a grid search pattern.")
port = 41451
world_bounds_msg_listen_port = -1

# If config.yml exists, read the file and set the connection port
if os.path.exists(config_path):
    with open(config_path, 'r') as file:
        config = yaml.load(file, Loader=yaml.FullLoader)
        port = config["ports_to_reserve"][4]["airsim_api"]
        print(f"Port number found in the config file: {port}")

        world_bounds_msg_listen_port = config["ports_to_reserve"][5]["airsim_world_bounds_msg"]
        print(f"Waiting for the world bounds info at port: {world_bounds_msg_listen_port}")

        grid_spacing = config["airsim"]["lawn_mower_grid_spacing"]
        speed = config["airsim"]["lawn_mower_speed"]
        height = config["airsim"]["lawn_mower_height"]
        adjust_yaw = config["airsim"]["lawn_mower_face_towards_velocity"]
else:
    print("Port number not found in the config file.")
    sys.exit(1)

# Connect to AirSim
client = airsim.MultirotorClient(ip="", port=port)
client.confirmConnection()
client.enableApiControl(True)



# Wait until we get a socket message from Unreal at port world_bounds_msg_listen_port
world_bound_str = ""  # Will receive the world bounds message in the following format: origin_x:origin_y:origin_z:extent_x:extent_y:extent_z
while True:
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # Allow reuse of the address
        sock.bind(('', world_bounds_msg_listen_port))
        sock.listen(1)

        # Send command to Unreal to trigger world bounds retrieval
        client.simRunConsoleCommand(f"py send_world_bounds_to_airsim.py {world_bounds_msg_listen_port}")

        print(f"Asked Unreal for the bounding box of the AirSimWorldBounds actor. Waiting at {world_bounds_msg_listen_port}")
        conn, addr = sock.accept()
        
        print(f"Received connection from: {addr}")
        data = conn.recv(1024)
        if data:
            print(f"Received message: {data.decode()}")
            world_bound_str = data.decode()
            conn.close()
            break
    except socket.error as e:
        print(f"Socket error: {e}")
        time.sleep(1)  # Add a delay before retrying

# Parse the received bounding box string
try:
    origin_x, origin_y, origin_z, extent_x, extent_y, extent_z = map(float, world_bound_str.split(":"))
    print(f"Parsed world bounds: Origin=({origin_x}, {origin_y}, {origin_z}), Extent=({extent_x}, {extent_y}, {extent_z})")
except ValueError as e:
    print(f"Failed to parse world bounds: {e}")
    sys.exit(1)

# Convert from Unreal units (cm) to AirSim units (meters)
origin_x /= 100
origin_y /= 100
origin_z /= 100
extent_x /= 100
extent_y /= 100
extent_z /= 100

# Calculate the corners of the bounding box
min_x = origin_x - extent_x
max_x = origin_x + extent_x
min_y = origin_y - extent_y
max_y = origin_y + extent_y
target_z = 35  # Fixed altitude for the grid search, adjust as needed

client.takeoffAsync().join()

# Teleport the drone to the starting corner of the bounding box
start_x, start_y = min_x, min_y
start_position = airsim.Vector3r(start_x, start_y, -target_z)
start_orientation = airsim.to_quaternion(0, 0, 0)  # No rotation
start_pose = airsim.Pose(start_position, start_orientation)

client.simSetVehiclePose(start_pose, True)
print(f"Teleported to start position: x={start_x}, y={start_y}, z={-target_z}")

# Function to calculate yaw angle to face target direction
def calculate_yaw(current_x, current_y, target_x, target_y):
    delta_x = target_x - current_x
    delta_y = target_y - current_y
    yaw = math.atan2(delta_y, delta_x)
    return yaw

# Perform lawn mower pattern
y = start_y
direction = 1  # Start by moving right

while y <= max_y:
    x_start = min_x if direction == 1 else max_x
    x_end = max_x if direction == 1 else min_x

    # Calculate yaw angle to face the start of the current row
    yaw = calculate_yaw(client.getMultirotorState().kinematics_estimated.position.x_val, 
                        client.getMultirotorState().kinematics_estimated.position.y_val, 
                        x_start, y)
    orientation = airsim.to_quaternion(0, 0, yaw)
    current_pose = client.getMultirotorState().kinematics_estimated.position
    current_pose = airsim.Pose(current_pose, orientation)
    client.simSetVehiclePose(current_pose, True)
    time.sleep(1)  # Small delay to ensure orientation update

    # Move to the start of the current row
    client.moveToPositionAsync(x_start, y, -target_z, speed).join()
    print(f"Moving to start of row: x={x_start}, y={y}")

    # Calculate yaw angle to face the end of the row
    yaw = calculate_yaw(x_start, y, x_end, y)
    orientation = airsim.to_quaternion(0, 0, yaw)
    current_pose = client.getMultirotorState().kinematics_estimated.position
    current_pose = airsim.Pose(current_pose, orientation)
    
    if adjust_yaw:
        client.simSetVehiclePose(current_pose, True)
    
    time.sleep(1)  # Small delay to ensure orientation update

    # Move across the row
    client.moveToPositionAsync(x_end, y, -target_z, speed).join()
    print(f"Moving across row to: x={x_end}, y={y}")

    # Move to the next row
    y += grid_spacing
    direction *= -1

client.landAsync().join()
client.enableApiControl(False)

print("Lawn mower grid search complete.")
