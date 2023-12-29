import rospy
import sys
import os
import json
from std_msgs.msg import String

import socket

print("getting ready to extract ground truth data from Unreal")
if (len(sys.argv) != 2):
    print("usage: python3 _unreal_ros_ground_truth.py num_messages")
    exit(1)
else:
    num_messages = int(sys.argv[1])
    print("expecting " + str(num_messages) + " messages")

class Extractor():
    def __init__(self, num_messages) -> None:
        self.num_messages = num_messages
        self.session_path = "/session/"
        self.session_path = os.path.join(self.session_path, max(os.listdir(self.session_path)))
        self.gt_path = os.path.join(self.session_path, "gt")
        if not os.path.exists(self.gt_path):
            os.makedirs(self.gt_path)
        self.message_count = 0
        self.ground_truth = {
            "collection": "MyCollection",
            "fileUID": "example",
            "startTime": "2022-1-5T13:21:05.431000",
            "stopTime": "2022-1-5T13:22:18.725000",
            "nFrames": 0,
            "frameAnnotations": {}
        }
        self.image_sub = rospy.Subscriber("/unreal_ros/ground_truth", String, self.gt_callback)
        rospy.spin()

    def gt_callback(self, data):
        print("received ground truth message")
        print(data.data)
        frame_annotation = {
            "frameAnnotations": {
                "f0": {
                    "boundingBoxes": [],
                    "timestamp": "2022-1-5T13:21:05.431000"
                }
            }
        }
        if data.data == "[]":
            print("empty ground truth message")
            return

        frame_annotation["frameAnnotations"]["f" + str(self.message_count)] = frame_annotation["frameAnnotations"].pop(list(frame_annotation["frameAnnotations"].keys())[0])
        self.message_count += 1

        print("frame annotation: ")
        print(frame_annotation)

        frame_name = list(frame_annotation["frameAnnotations"].keys())[0]
        self.ground_truth["frameAnnotations"].update(frame_annotation["frameAnnotations"])
        self.ground_truth["nFrames"] += 1
        if self.ground_truth["nFrames"] == self.num_messages:
            print("done extracting ground truth")
            output_path = os.path.join(self.gt_path, "ground_truth.json")
            with open(output_path, "w") as f:
                json.dump(self.ground_truth, f)

            #send done message. just send "1" to indicate it's done
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect(('localhost', 1234))

            s.sendall('1'.encode('utf-8'))

            s.close()

            print('sent 1')
            
            rospy.signal_shutdown("done")
        

if __name__ == "__main__":
    rospy.init_node("gt_extractor", anonymous=True)
    image_extractor = Extractor(num_messages)