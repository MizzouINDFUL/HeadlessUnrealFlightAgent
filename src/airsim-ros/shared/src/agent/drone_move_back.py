import airsim
import os
import yaml
import sys

# Check if enough arguments are provided
if len(sys.argv) < 3:
    print("Not enough arguments provided.")

config_path = "/config.yml"
if len(sys.argv) > 3:
    config_path = sys.argv[3]

print("config file: " + config_path)

print("Behavior of this agent: drone will get up and start in a grid search pattern.")
port = 41451
#if /config.yml exists, read the file and set the connection port
if os.path.exists(config_path):
    with open(config_path, 'r') as file:
        config = yaml.load(file, Loader=yaml.FullLoader)
        port = config["ports_to_reserve"][4]["airsim_api"]
        print(f"Port number found in the config file: {port}")
else:
    print("Port number not found in the config file.")
    sys.exit(1)

# Connect to AirSim
client = airsim.MultirotorClient(ip="", port=port)
client.confirmConnection()
client.enableApiControl(True)

speed = 10  # Speed of movement
move_up_distance = 80  # Distance to move up

# Define the volume boundaries and step size
x_min, x_max, x_step = -50, 50, 10
y_min, y_max, y_step = -50, 50, 10
z_min, z_max, z_step = -50, 50, 10

# Take off
client.takeoffAsync().join()

# Move up
client.moveByVelocityAsync(0, 0, -speed, duration=move_up_distance / speed).join()

# Perform the grid search
for x in range(x_min, x_max, x_step):
    for y in range(y_min, y_max, y_step):
        for z in range(z_min, z_max, z_step):
            # Move to the next point in the grid
            client.moveToPositionAsync(x, y, z, speed).join()

# Land
client.landAsync().join()
