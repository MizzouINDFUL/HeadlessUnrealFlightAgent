import socket
import sys

def send_command_to_unreal_engine(port, command):
    # Create a socket object
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Connect to the server
    s.connect(("localhost", int(port)))

    # Send the command
    s.sendall(command.encode())

    # Close the connection
    s.close()

if __name__ == "__main__":
    # Get the port and command from the command line arguments
    port = sys.argv[1]
    command = ' '.join(sys.argv[2:])
    
    # Send the command to Unreal Engine
    send_command_to_unreal_engine(port, command)