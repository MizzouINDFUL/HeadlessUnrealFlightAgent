import airsim

# Connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()

# Set the drone's mode to "GUIDED"
client.enableApiControl(True)
client.armDisarm(True)
client.takeoffAsync().join()
client.moveToPositionAsync(0, 0, -10, 5).join()

# Get a list of all actors in the scene
all_actors = client.simListSceneObjects()

# Print all actor names
for actor in all_actors:
    print(actor)

# Set the drone's camera to look at the target actor
target_actor_label = "TrackerActor_0"
camera_name = "front_center"  # Change this to the appropriate camera name
camera_pose = airsim.Pose(airsim.Vector3r(0, 0, 0), airsim.to_quaternion(0, 0, 0))
client.simSetCameraPose(camera_name, camera_pose)

# Main loop
while True:
    # Get the position of the target actor
    target_actor_pose = client.simGetObjectPose(target_actor_label)
    target_actor_position = target_actor_pose.position

    print(f"Target actor name: {target_actor_label}")
    print(f"Target actor position: {target_actor_position}")

    # Get the drone's current position
    drone_pose = client.simGetVehiclePose()
    drone_position = drone_pose.position

    # Calculate the direction vector from the drone to the target actor
    direction_vector = target_actor_position - drone_position

    # Set the drone's velocity to follow the target actor
    velocity = direction_vector * 0.5  # Adjust the speed as needed

    # print(f"Velocity: {velocity}")

    client.moveByVelocityAsync(velocity.x_val, velocity.y_val, velocity.z_val, 1).join()

    # Sleep for a short duration
    airsim.time.sleep(0.1)
