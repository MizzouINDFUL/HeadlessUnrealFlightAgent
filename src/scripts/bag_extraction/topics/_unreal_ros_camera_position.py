import rospy
import sys
import os
import json
from std_msgs.msg import String
import yaml
import socket

print("getting ready to extract camera position info from Unreal")
if len(sys.argv) < 4:
    print("usage: python3 _unreal_ros_pose.py num_messages real_num_messages config_path")
    exit(1)
else:
    num_messages = int(sys.argv[1])
    print("expecting " + str(num_messages) + " messages")

port = 1234
curr_life = 1
session_basename = ""
sessionname = ""
runninng_from_container = False
config_path = sys.argv[3]

if not os.path.exists(config_path):
    print("config.yml not found")
    exit(1)

with open(config_path, 'r') as stream:
    try:
        config = yaml.safe_load(stream)
        port = config["ports_to_reserve"][2]["rosbag_extraction_listener"]
        curr_life = config["current_life"]
        session_basename = config["session"]["basename"]
        sessionname = config["sessionname"]
        runninng_from_container = config["ros"]["use_docker"]
        print(f"Port number found in the config file: {port}")
        print(f"Current life found in the config file: {curr_life}")
        print(f"Session name found in the config file: {session_basename}")
    except yaml.YAMLError as exc:
        print(exc)
        exit(1)

#example: 1 -> f0000001
def format_frame_num(in_num):
    return "f" + str(in_num).zfill(7)

class CameraPositionExtractor():
    def __init__(self) -> None:
        self.session_path = "/session/"
        if not runninng_from_container:
            self.session_path = os.path.join("bags/", session_basename)
        
        print("extracting images to " + self.session_path)
        self.session_path = os.path.join(self.session_path, str(curr_life))
        print("extracting camera position info to " + self.session_path)
        self.camera_pos_path = os.path.join(self.session_path, "camera_position.json")
        self.frame_annotations = []

        full_topic = "/" + sessionname + "/unreal_ros/camera_position"
        self.image_sub = rospy.Subscriber(full_topic, String, self.callback_to_json)
        rospy.spin()

    def callback_to_json(self, data):
        # Parse the incoming JSON string
        camera_data = json.loads(data.data)
        frame_num = len(self.frame_annotations)
        frame_key = format_frame_num(frame_num)
        frame_data = [frame_key, camera_data]
        self.frame_annotations.append(frame_data)

        if len(self.frame_annotations) == num_messages:
            print("done extracting camera position info from Unreal")
            final_data = {"frameAnnotations": self.frame_annotations}
            with open(self.camera_pos_path, "w") as json_file:
                json.dump(final_data, json_file, indent=4)
            print(f"Camera position info saved to {self.camera_pos_path}")

            # Send done message. Just send "1" to indicate it's done
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect(('localhost', port))
            s.sendall('1'.encode('utf-8'))
            s.close()
            print('sent 1')
            rospy.signal_shutdown("Extraction complete")

if __name__ == "__main__":
    rospy.init_node("camera_position_extractor", anonymous=True)
    extractor = CameraPositionExtractor()