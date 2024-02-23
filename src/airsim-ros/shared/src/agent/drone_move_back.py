import airsim
import sys
import glob
import os

print("Behavior of this agent: drone will get up and start moving backwards")

# Check if enough arguments are provided
if len(sys.argv) < 3:
    print("Not enough arguments provided.")

# Connect to AirSim
client = airsim.MultirotorClient()
client.confirmConnection()

# Set variables
speed = 5  # Speed of movement
move_up_distance = 10  # Distance to move up
move_forward_distance = 60  # Distance to move forward/backwards
rain_percentage = 300  # Percentage of rain

# Set the time of day and rain percentage only if enough arguments are provided
if len(sys.argv) >= 3:
    # Get the index of the current life and the max number of lives
    current_life_index = int(sys.argv[1])
    max_lives = int(sys.argv[2])

    current_life_index = current_life_index - 1
    max_lives = max_lives - 1

    print("Current life index: {}".format(current_life_index))
    print("Max lives: {}".format(max_lives))

    time_eraliest = 24
    time_latest = 17

    # Calculate the rain percentage based on the current life index
    rain_percentage = (current_life_index // 10) * (current_life_index // 10) * 100
    # rain_percentage = 1000

    # Calculate the time of day alpha based on the current life index
    time_of_day_alpha = (current_life_index % 10) / 10

    # time_of_day_alpha = 1 - time_of_day_alpha

    print("Rain percentage: {}%".format(rain_percentage))
    print("Time of day alpha: {}".format(time_of_day_alpha))

    time_of_day = time_eraliest + (time_latest - time_eraliest) * time_of_day_alpha

    if time_of_day > 23:
        time_of_day = time_of_day - 24

    time = "2018-04-12 {}:00:00".format(int(time_of_day))

    #save the config for this airsim run in /root/session/[last created folder]/airsim_settings.txt
    settings_string = "time of day: {}\nrain percentage: {}%\n".format(time, rain_percentage)
    settings_string += "current life index: {}\nmax lives: {}\n".format(current_life_index, max_lives)
    settings_string += "time of day alpha: {}\n".format(time_of_day_alpha)
    settings_string += "time_eraliest: {}\n".format(time_eraliest)
    settings_string += "time_latest: {}\n".format(time_latest)

    #get the latest folder in /root/session
    list_of_folders = glob.glob("/root/session/*")
    latest_folder = max(list_of_folders, key=os.path.getctime)

    #save the settings to a file
    with open(latest_folder + "/airsim_settings.txt", "w") as file:
        file.write(settings_string)

    print("Time of day: {}".format(time))

    client.simRunConsoleCommand("py unreal.MindfulLib.add_rain_follow(unreal.UnrealEditorSubsystem().get_game_world(), {})".format(rain_percentage*100))
    # client.simRunConsoleCommand("py unreal.MindfulLib.add_rain_follow(unreal.UnrealEditorSubsystem().get_game_world(), {})".format(100000))
    # Set the time of day
    # client.simSetTimeOfDay(start_datetime=time, is_enabled=True)

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
