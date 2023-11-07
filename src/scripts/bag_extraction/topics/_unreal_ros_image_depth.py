import rospy
import sys
import os
import numpy as np
import socket
from sensor_msgs.msg import Image

print("getting ready to extract depth imagery from Unreal")
if (len(sys.argv) != 2):
    print("usage: python3 _unreal_ros_image_color.py num_messages")
    exit(1)
else:
    num_messages = int(sys.argv[1])
    print("expecting " + str(num_messages) + " messages")

#this wwill subscribe to the image topic and extract the images
class ImageExtractor():
    def __init__(self) -> None:

        #create an rgb folder in /session/*latest_folder*/
        self.session_path = "/session/"
        #get the latest subfolder
        self.session_path = os.path.join(self.session_path, max(os.listdir(self.session_path)))

        self.rgb_path = os.path.join(self.session_path, "depth")
        if not os.path.exists(self.rgb_path):
            os.makedirs(self.rgb_path)
        
        self.image_count = 0

        #subscribe to the image topic
        self.image_sub = rospy.Subscriber("/unreal_ros/image_depth", Image, self.image_callback)
        rospy.spin()
    
    def image_callback(self, data):
        #save this data as a numpy array
        depth_image = np.frombuffer(data.data, dtype=np.float32)
        depth_image = depth_image.reshape(data.height, data.width)
        #save the array
        np.save(os.path.join(self.rgb_path, str(self.image_count) + ".npy"), depth_image)
        self.image_count += 1
        if self.image_count == num_messages:
            print("done extracting depth")

            #send done message. just send "1" to indicate it's done
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect(('localhost', 1234))

            s.sendall('3'.encode('utf-8'))

            s.close()

            print('sent 3')

            rospy.signal_shutdown("done")

if __name__ == "__main__":
    rospy.init_node("depth_extractor", anonymous=True)
    image_extractor = ImageExtractor()