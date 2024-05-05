import airsim
import datetime
import sys
import glob
import os
import yaml

print("Behavior of this agent: drone will get up and start moving backwards")
port = 41451
#if /config.yml exists, read the file and set the connection port
if os.path.exists('/config.yml'):
    with open('/config.yml', 'r') as file:
        config = yaml.load(file, Loader=yaml.FullLoader)
        port = config["ports_to_reserve"][4]["airsim_api"]
        print(f"Port number found in the config file: {port}")
else:
    print("Port number not found in the config file.")
    sys.exit(1)

# Check if enough arguments are provided
if len(sys.argv) < 3:
    print("Not enough arguments provided.")

# Connect to AirSim
client = airsim.MultirotorClient(ip="", port=port)
client.confirmConnection()

import datetime

tod_morning = datetime.datetime.strptime("2018-02-12 10:15:00", "%Y-%m-%d %H:%M:%S")
tod_day = datetime.datetime.strptime("2018-02-12 14:00:00", "%Y-%m-%d %H:%M:%S")
tod_evening = datetime.datetime.strptime("2018-02-12 19:25:00", "%Y-%m-%d %H:%M:%S")
tod_night = datetime.datetime.strptime("2018-02-12 19:35:00", "%Y-%m-%d %H:%M:%S")

def get_time_of_day(tod_alpha):
    if tod_alpha < 0 or tod_alpha > 1:
        raise ValueError("tod_alpha must be between 0 and 1")

    if tod_alpha <= 0.25:
        start_time = tod_morning
        end_time = tod_day
        alpha = tod_alpha / 0.25
    elif tod_alpha <= 0.5:
        start_time = tod_day
        end_time = tod_evening
        alpha = (tod_alpha - 0.25) / 0.25
    elif tod_alpha <= 0.75:
        start_time = tod_evening
        end_time = tod_night
        alpha = (tod_alpha - 0.5) / 0.25
    else:
        start_time = tod_night
        end_time = tod_morning + datetime.timedelta(days=1)
        alpha = (tod_alpha - 0.75) / 0.25

    delta = end_time - start_time
    interpolated_time = start_time + alpha * delta

    return interpolated_time.strftime("%Y-%m-%d %H:%M:%S")
    
curr_tod = tod_night

if len(sys.argv) >= 2:

    # Get the index of the current life and the max number of lives
    current_life_index = int(sys.argv[1])
    idx : float = (current_life_index) % 10
    curr_tod = get_time_of_day(idx / 13.0)

    print("current life: " + str(current_life_index))
    print("idx: " + str(idx))
    print(curr_tod)
else:
    print("not enough args")

# client.simSetTimeOfDay(start_datetime=curr_tod, is_enabled=True)

#temp

speed = 5  # Speed of movement
move_up_distance = 10  # Distance to move up
move_forward_distance = 60  # Distance to move forward/backwards
rain_percentage = 300  # Percentage of rain

import time

def map_int(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

# Take off
client.takeoffAsync().join()
client.moveByVelocityAsync(0, 0, -speed, duration=move_up_distance / speed).join()

# Take off
client.takeoffAsync().join()

# Move up
client.moveByVelocityAsync(0, 0, -speed, duration=move_up_distance / speed).join()

while True:
    # Move backwards
    client.moveByVelocityAsync(speed, 0, 0, duration=move_forward_distance / speed).join()

    # Move forwards
    client.moveByVelocityAsync(-speed, 0, 0, duration=move_forward_distance / speed).join()

# Land
client.landAsync().join()
