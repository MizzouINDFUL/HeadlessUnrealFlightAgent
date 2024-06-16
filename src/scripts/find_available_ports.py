import socket
import sys
import yaml

def find_available_ports(start_port, num_ports_needed, reserved_ports):
    ports = []
    while len(ports) < num_ports_needed:
        if start_port not in reserved_ports and is_port_available(start_port):
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
    already_reserved_ports = {item[port_name]: port_name for item in ports_to_reserve if isinstance(item, dict) for port_name in item}
    ports_needed = [port_name for port_name in ports_to_reserve if not isinstance(port_name, dict)]

    num_ports_needed = len(ports_needed)
    ports = find_available_ports(start_port, num_ports_needed, already_reserved_ports.keys())

    updated_ports_to_reserve = []
    for port_name in ports_to_reserve:
        if isinstance(port_name, dict):  # If port is already set, keep it as is
            updated_ports_to_reserve.append(port_name)
        else:
            assigned_port = ports.pop(0)  # Assign new port
            updated_ports_to_reserve.append({port_name: assigned_port})

    config['ports_to_reserve'] = updated_ports_to_reserve

    with open(config_file, 'w') as f:
        yaml.safe_dump(config, f)