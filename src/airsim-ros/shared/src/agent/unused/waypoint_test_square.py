import numpy as np
import airsim
import time
import sys

class WaypointController():

    def __init__(self):

        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()

        self.client.enableApiControl(True)
        self.client.armDisarm(True)

        # set weather to 90% rain
        self.client.simEnableWeather(True)
        self.client.simSetWeatherParameter(airsim.WeatherParameter.Rain, 1.0)

        time_morning = "2018-04-12 08:30:00"
        time_day = "2018-04-12 15:30:00"
        time_evening = "2018-04-12 22:30:00"

        target_time = time_morning

        if len(sys.argv) > 1:
            arg = int(sys.argv[1])
            print(f"Life number {arg}")
            if arg == 1:
                target_time = time_morning
            elif arg == 2:
                target_time = time_day
            elif arg == 3:
                target_time = time_evening

        self.client.simSetTimeOfDay(True, start_datetime=target_time, is_start_datetime_dst=False, celestial_clock_speed=1, update_interval_secs=60, move_sun=True)

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