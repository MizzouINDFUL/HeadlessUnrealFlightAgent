import re
import json
import sys

config_file_path = sys.argv[1]
json_file_path = sys.argv[2]

with open(config_file_path, 'r') as file:
    config_data = file.read()

# Extract the port number using regular expressions
port_match = re.search(r'\bairsim_api:\s*(\d+)', config_data)
if port_match:
    port = int(port_match.group(1))
    print(f"Port number found in the config file: {port}")
else:
    print("Port number not found in the config file.")
    sys.exit(1)

with open(json_file_path, 'r') as file:
    json_data = json.load(file)
    json_data["ApiServerPort"] = port

with open(json_file_path, 'w') as file:
    json.dump(json_data, file, indent=4)