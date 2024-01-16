import airsim
import sys

# Check if enough arguments are provided
if len(sys.argv) < 3:
    print("Not enough arguments provided.")

# Connect to AirSim
client = airsim.MultirotorClient()
client.confirmConnection()

# Set variables
speed = 5  # Speed of movement
move_up_distance = 10  # Distance to move up
move_forward_distance = 20  # Distance to move forward/backwards
rain_percentage = 100  # Percentage of rain

# Enable rain
client.simEnableWeather(True)
client.simSetWeatherParameter(airsim.WeatherParameter.Rain, rain_percentage)

# Take off
client.takeoffAsync().join()

# Set the time of day only if enough arguments are provided
if len(sys.argv) >= 3:
    # Get the index of the current life and the max number of lives
    current_life_index = int(sys.argv[1])
    max_lives = int(sys.argv[2])

    time_eraliest = 6
    time_latest = 23

    # Calculate the time of day based on the current life index
    time_of_day_alpha = current_life_index / max_lives

    time_of_day = time_eraliest + (time_latest - time_eraliest) * time_of_day_alpha

    time = "2018-04-12 {}:00:00".format(int(time_of_day))

    # Set the time of day
    client.simSetTimeOfDay(enabled=True, is_start_datetime=False, start_datetime=time, is_enabled=True)

# Move up
client.moveByVelocityAsync(0, 0, -speed, duration=move_up_distance / speed).join()

# Move backwards
client.moveByVelocityAsync(speed, 0, 0, duration=move_forward_distance / speed).join()

# Move forwards
client.moveByVelocityAsync(-speed, 0, 0, duration=move_forward_distance / speed).join()

# Land
client.landAsync().join()
