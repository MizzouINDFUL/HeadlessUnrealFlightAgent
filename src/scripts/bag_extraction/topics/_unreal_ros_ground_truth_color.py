import rospy
import sys
import os
import numpy as np
import socket
import cv2
from sensor_msgs.msg import Image

folder_name = None

print("getting ready to extract color imagery from Unreal")
if (len(sys.argv) < 2):
    print("usage: python3 _unreal_ros_groundt_truth_color.py num_messages")
    exit(1)
else:
    num_messages = int(sys.argv[1])
    print("expecting " + str(num_messages) + " messages")

    if (len(sys.argv) > 2):
        folder_name = sys.argv[2]
        print("extracting images to " + folder_name)

#this wwill subscribe to the image topic and extract the images
class ImageExtractor():
    def __init__(self, folder_name=None) -> None:
        self.session_path = "/session/"

        if folder_name is not None:
            self.session_path = os.path.join(self.session_path, folder_name)
        else:
            # Get the latest folder in /session
            subfolders = [f.path for f in os.scandir(self.session_path) if f.is_dir()]

            # Get the latest subfolder
            self.session_path = max(subfolders, key=os.path.getmtime)
        
        print("extracting images to " + self.session_path)

        self.rgb_path = os.path.join(self.session_path, "gt_color")
        if not os.path.exists(self.rgb_path):
            os.makedirs(self.rgb_path)
        
        self.image_count = 0

        #subscribe to the image topic
        self.image_sub = rospy.Subscriber("/unreal_ros/ground_truth_color", Image, self.image_callback)
        rospy.spin()
    
    def image_callback(self, data):
        #convert the image to a cv2 image
        cv_image = self.imgmsg_to_cv2(data)
        #save the image. maming foramt should be 00000001.png, 00000002.png, etc
        pngname = str(self.image_count) + ".png"

        for i in range(11 - len(pngname)):
            pngname = "0" + pngname

        cv2.imwrite(os.path.join(self.rgb_path, pngname), cv_image)
        self.image_count += 1
        if self.image_count == num_messages:
            print("done extracting images")
            
            #send done message. just send "1" to indicate it's done
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect(('localhost', 1234))

            s.sendall('2'.encode('utf-8'))

            s.close()

            print('sent 2')

            rospy.signal_shutdown("done")
    
    def imgmsg_to_cv2(self, img_msg):
        if img_msg.encoding != "bgr8":
            rospy.logerr("This Coral detect node has been hardcoded to the 'bgr8' encoding.  Come change the code if you're actually trying to implement a new camera")
        dtype = np.dtype("uint8") # Hardcode to 8 bits...
        dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
        image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3), # and three channels of data. Since OpenCV works with bgr natively, we don't need to reorder the channels.
                        dtype=dtype, buffer=img_msg.data)
        # If the byt order is different between the message and the system.
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
        img_msg.step = len(img_msg.data) // img_msg.height # That double line is actually integer division, not a comment
        return img_msg

if __name__ == "__main__":
    rospy.init_node("image_extractor", anonymous=True)
    image_extractor = ImageExtractor(folder_name)