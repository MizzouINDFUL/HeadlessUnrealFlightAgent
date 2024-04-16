import rospy
import sys
import os
import json
from std_msgs.msg import String

import socket

real_num_msgs = -1

print("getting ready to extract ground truth data from Unreal")
if (len(sys.argv) < 2):
    print("usage: python3 _unreal_ros_ground_truth.py num_messages")
    exit(1)
else:
    num_messages = int(sys.argv[1])
    print("expecting " + str(num_messages) + " messages")
    if (len(sys.argv) > 2):
        real_num_msgs = int(sys.argv[2])
    else:
        real_num_msgs = num_messages
        

class Extractor():
    def __init__(self, num_messages, num_msg_to_burn=0) -> None:
        self.num_messages = num_messages
        self.session_path = "/session/"
        self.num_to_ignore = num_msg_to_burn

        # Get the latest folder in /session
        subfolders = [f.path for f in os.scandir(self.session_path) if f.is_dir()]

        # Get the latest subfolder
        self.session_path = max(subfolders, key=os.path.getmtime)

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

        if self.message_count < self.num_to_ignore:
            self.message_count += 1
            print("ignoring message")
            return

        print("received ground truth message")
        # print(data.data)

        frame_annotation = {
            "frameAnnotations": {
                
            }
        }
        if data.data == "[]":
            print("empty ground truth message")
            return

        new_frame = json.loads(data.data)['frameAnnotations'][list(json.loads(data.data)['frameAnnotations'].keys())[0]]

        received_frame_number = list(json.loads(data.data)['frameAnnotations'].keys())[0]

        if self.num_to_ignore > 0:
            #1. remove 'f' in front of the number and convert to int
            received_frame_number = received_frame_number[1:]
            received_frame_number = str(int(received_frame_number) - self.num_to_ignore - 1)
            received_frame_number = 'f' + received_frame_number

        print("received frame number: " + received_frame_number)

        # Iterate over the annotations in reverse order to avoid issues when removing items
        for i in reversed(range(len(new_frame['annotations']))):
            # If the data array is [0,0,0,0], remove the annotation
            if new_frame['annotations'][i]['shape']['data'] == [0,0,0,0]:
                del new_frame['annotations'][i]

        frame_annotation["frameAnnotations"][received_frame_number] = new_frame
        self.ground_truth["frameAnnotations"].update(frame_annotation["frameAnnotations"])
        self.message_count += 1

        self.ground_truth["nFrames"] += 1
        if self.ground_truth["nFrames"] == self.num_messages:
            print("done extracting ground truth")

            output_path = os.path.join(self.gt_path, "ground_truth.json")
            with open(output_path, "w") as f:
                json.dump(self.ground_truth, f, ensure_ascii=False)

            #send done message. just send "1" to indicate it's done
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect(('localhost', 1234))

            s.sendall('1'.encode('utf-8'))

            s.close()

            print('sent 1')
            
            rospy.signal_shutdown("done")
        

if __name__ == "__main__":
    rospy.init_node("gt_extractor", anonymous=True)

    to_burn = 0
    if real_num_msgs > num_messages:
        to_burn = real_num_msgs - num_messages

    image_extractor = Extractor(num_messages, to_burn)