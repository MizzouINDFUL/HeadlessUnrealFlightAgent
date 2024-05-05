import socket
import sys
import yaml

def find_available_ports(start_port, num_ports):
    ports = []
    while len(ports) < num_ports:
        if is_port_available(start_port):
            ports.append(start_port)
        start_port += 1
    return ports

def is_port_available(port):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        return s.connect_ex(('localhost', port)) != 0

if __name__ == "__main__":
    session_basename = sys.argv[1]
    num_sim_sessions = int(sys.argv[2])
    start_port = 5000 + 1000 * num_sim_sessions
    config_file = f"tmp/{session_basename}{num_sim_sessions}-config.yml"

    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)

    ports_to_reserve = config.get('ports_to_reserve', [])
    ports = find_available_ports(start_port, len(ports_to_reserve))

    for i, port_name in enumerate(ports_to_reserve):
        config['ports_to_reserve'][i] = {port_name: ports[i]}

    with open(config_file, 'w') as f:
        yaml.safe_dump(config, f)