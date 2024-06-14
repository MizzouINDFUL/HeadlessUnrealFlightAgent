import rospy
import sys
import os
import json
from std_msgs.msg import String
import yaml
import socket

print("getting ready to extract camera info from Unreal")
if len(sys.argv) < 4:
    print("usage: python3 _unreal_ros_camera_info.py num_messages real_num_messages config_path")
    exit(1)

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
        print(f"Port number found in the config file: {port}")
        print(f"Current life found in the config file: {curr_life}")
        print(f"Session name found in the config file: {sessionname}")
    except yaml.YAMLError as exc:
        print(exc)
        exit(1)

class CameraInfoExtractor():
    def __init__(self) -> None:
        self.session_path = "/session/"
        self.session_path += str(curr_life)
        print("extracting camera info to " + self.session_path)
        self.camera_info_path = os.path.join(self.session_path, "camera_info.json")

        full_topic = "/" + sessionname + "/unreal_ros/camera_info"
        self.info_sub = rospy.Subscriber(full_topic, String, self.callback_to_json)
        rospy.spin()

    def callback_to_json(self, data):
        # Parse the incoming JSON string
        camera_info = json.loads(data.data)
        print("received camera info from Unreal")
        with open(self.camera_info_path, "w") as json_file:
            json.dump(camera_info, json_file, indent=4)
        print(f"Camera info saved to {self.camera_info_path}")

        # Send done message. Just send "1" to indicate it's done
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect(('localhost', port))
        s.sendall('1'.encode('utf-8'))
        s.close()
        print('sent 1')
        rospy.signal_shutdown("Extraction complete")

if __name__ == "__main__":
    rospy.init_node("camera_info_extractor", anonymous=True)
    extractor = CameraInfoExtractor()