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

        alt = 40
        width = 100
        height = 100

        steps = 3
        step_w = width / steps

        waypoints = []
        x = 0

        while x <= width:
            waypoints.append((0, x, -alt, 90))
            waypoints.append((height, x, -alt, 0))

            x += step_w

            waypoints.append((height, x, -alt, 90))
            waypoints.append((0, x, -alt, 180))

            x += step_w

        # Fix orientation on first waypoint
        waypoints[0] = (0, 0, -alt, 0)

        wpi = 0

        while wpi < len(waypoints):

            wp = waypoints[wpi]

            moving = True
            while moving:
                try:
                    self.setGimbal.publish(gimbalMsg)
                    self.goto(wp[0], wp[1], wp[2], wp[3]*np.pi/180, '')
                    print(f"Going to {wp[0]} {wp[1]} {wp[2]} {wp[3]}")
                    moving = False
                except rospy.ServiceException:

                    rospy.sleep(1)
                    continue
            
            wpi += 1
        
        moving = True
        while moving:
            try:
                self.setGimbal.publish(gimbalMsg)
                self.goto(wp[0], wp[1], wp[2], wp[3]*np.pi/180, '')
                print(f"Done")
                moving = False
            except rospy.ServiceException:

                rospy.sleep(1)
                continue

        rospy.spin()


if __name__ == '__main__':
    ctrl = WaypointController()