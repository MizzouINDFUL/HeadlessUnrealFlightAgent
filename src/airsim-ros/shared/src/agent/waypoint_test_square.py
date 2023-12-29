import numpy as np
import airsim
import time

class WaypointController():

    def __init__(self):

        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()

        self.client.enableApiControl(True)
        self.client.armDisarm(True)

        #set weather to 90% rain
        self.client.simEnableWeather(True)
        self.client.simSetWeatherParameter(airsim.WeatherParameter.Rain, 0.9)

        self.client.takeoffAsync().join()

        # self.client.simSetCameraOrientation("0", airsim.to_quaternion(-np.pi/2, 0, 0)) # Downward facing camera

        alt = -30
        width = 100
        height = 100

        self.waypoints = [(0, 0, alt, 270),
                          (width, 0, alt, 0),
                          (width, height, alt, 90),
                          (0, height, alt, 180)]

        self.wpi = 0

    def move_to_waypoint(self, waypoint):
        print(f"Going to {waypoint[0]} {waypoint[1]} {waypoint[2]} {waypoint[3]}")
        self.client.moveToPositionAsync(waypoint[0], waypoint[1], waypoint[2], 5).join()
        self.client.rotateToYawAsync(waypoint[3]).join()
        print('At goal')

    def run(self):
        while True:
            self.move_to_waypoint(self.waypoints[self.wpi])
            self.wpi = (self.wpi + 1) % len(self.waypoints)


if __name__ == '__main__':
    ctrl = WaypointController()
    ctrl.run()