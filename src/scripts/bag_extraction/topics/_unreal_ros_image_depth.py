import rospy
import sys
import os
os.environ["OPENCV_IO_ENABLE_OPENEXR"]="1"
import numpy as np
import socket
import cv2
from sensor_msgs.msg import Image
import yaml

print("getting ready to extract color imagery from Unreal")
if len(sys.argv) < 4:
    print("usage: python3 _unreal_ros_image_color.py num_messages real_num_messages config_path")
    exit(1)
else:
    num_messages = int(sys.argv[1])
    print("expecting " + str(num_messages) + " messages")

port = 1234
curr_life = 1
sessionname = ""
running_from_container = False
config_path = sys.argv[3]

if not os.path.exists(config_path):
    print("config.yml not found")
    exit(1)

with open(config_path, 'r') as stream:
    try:
        config = yaml.safe_load(stream)
        port = config["ports_to_reserve"][2]["rosbag_extraction_listener"]
        curr_life = config["current_life"]
        sessionname = config["sessionname"]
        running_from_container = config["ros"]["use_docker"]
        print(f"Port number found in the config file: {port}")
        print(f"Current life found in the config file: {curr_life}")
        print(f"Session name found in the config file: {sessionname}")
    except yaml.YAMLError as exc:
        print(exc)
        exit(1)

class ImageExtractor():
    def __init__(self) -> None:
        self.session_path = "/session/"
        if not running_from_container:
            self.session_path = os.path.join("bags/", sessionname)

        print("extracting images to " + self.session_path)
        self.session_path = os.path.join(self.session_path, str(curr_life))
        

        self.rgb_path = os.path.join(self.session_path, "depth")
        if not os.path.exists(self.rgb_path):
            os.makedirs(self.rgb_path)

        self.image_count = 0

        full_topic = "/" + sessionname + "/unreal_ros/image_depth"
        self.image_sub = rospy.Subscriber(full_topic, Image, self.image_callback)
        rospy.spin()

    def image_callback(self, data):
        cv_image = self.imgmsg_to_cv2(data)
        
        # Naming format should be 00000001.exr, 00000002.exr, etc
        exrname = str(self.image_count) + ".exr"
        for i in range(11 - len(exrname)):
            exrname = "0" + exrname

        cv2.imwrite(os.path.join(self.rgb_path, exrname), cv_image.astype(np.float32))
        self.image_count += 1

        if self.image_count == num_messages:
            print("done extracting images")

            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect(('localhost', port))
            s.sendall('2'.encode('utf-8'))
            s.close()
            print('sent 2')

            rospy.signal_shutdown("done")

    def imgmsg_to_cv2(self, img_msg):
        if img_msg.encoding != "bgr8":
            rospy.logerr("This Coral detect node has been hardcoded to the 'bgr8' encoding.  Come change the code if you're actually trying to implement a new camera")
        dtype = np.dtype("uint8") # Hardcoded to 8 bits
        dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
        image_opencv = np.ndarray(
            shape=(img_msg.height, img_msg.width, 3),
            dtype=dtype, buffer=img_msg.data
        )
        if img_msg.is_bigendian == (sys.byteorder == 'little'):
            image_opencv = image_opencv.byteswap().newbyteorder()
        return image_opencv

    def cv2_to_imgmsg(self, cv_image):
        img_msg = Image()
        img_msg.height = cv_image.shape[0]
        img_msg.width = cv_image.shape[1]
        img_msg.encoding = "bgr8"
        img_msg.is_bigendian = 0
        img_msg.data = cv_image.tostring()
        img_msg.step = len(img_msg.data) // img_msg.height
        return img_msg

if __name__ == "__main__":
    rospy.init_node("image_extractor", anonymous=True)
    image_extractor = ImageExtractor()
