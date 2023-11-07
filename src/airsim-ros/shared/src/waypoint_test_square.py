import numpy as np
import rospy

from airsim_ros_pkgs.msg import GimbalAngleEulerCmd
from airsim_ros_pkgs.srv import Takeoff, SetLocalPosition

class WaypointController():

    def __init__(self):

        rospy.init_node('waypoint_controller', anonymous=True)

        self.takeoff = rospy.ServiceProxy('/airsim_node/drone_1/takeoff', Takeoff)
        self.goto = rospy.ServiceProxy('/airsim_node/local_position_goal', SetLocalPosition)

        self.takeoff()

        self.setGimbal = rospy.Publisher('/airsim_node/gimbal_angle_euler_cmd', GimbalAngleEulerCmd, queue_size=1)

        gimbalMsg = GimbalAngleEulerCmd()
        gimbalMsg.camera_name = 'front'
        gimbalMsg.vehicle_name = 'drone_1'
        gimbalMsg.roll = 0
        gimbalMsg.pitch = -90
        gimbalMsg.yaw = 0

        alt = 30
        width = 100
        height = 100

        waypoints = [(0, 0, -alt, 270),
                     (width, 0, -alt, 0),
                     (width, height, -alt, 90),
                     (0, height, -alt, 180)]

        wpi = 0

        while True:

            wp = waypoints[wpi]

            print(f"Going to {wp[0]} {wp[1]} {wp[2]} {wp[3]}")

            moving = True
            while moving:
                try:
                    # self.setGimbal.publish(gimbalMsg)
                    self.goto(wp[0], wp[1], wp[2], wp[3]*np.pi/180, '')

                    moving = False
                    print('At goal')
                except rospy.ServiceException:

                    rospy.sleep(1)
                    continue
            
            wpi = (wpi + 1) % len(waypoints)

        rospy.spin()


if __name__ == '__main__':
    ctrl = WaypointController()