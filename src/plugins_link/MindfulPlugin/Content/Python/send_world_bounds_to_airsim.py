import unreal
import socket
import threading
import sys

# Function to handle socket operations
def send_world_bounds(center, extents, port):
    try:
        # Create a socket and set it up to listen for connections
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # Connect to the server
        server_address = ('localhost', port)
        sock.connect(server_address)

        # Send data
        data_to_send = f"{center.x}:{center.y}:{center.z}:{extents.x}:{extents.y}:{extents.z}"
        sock.sendall(data_to_send.encode())

        # Close the socket
        sock.close()
    except Exception as e:
        unreal.log_error(f"Error in send_world_bounds: {e}")

# Function to validate and convert the port argument
def get_port_from_arguments():
    if len(sys.argv) < 2:
        unreal.log_error("Port number argument is missing.")
        sys.exit(1)
    
    try:
        port = int(sys.argv[1])
        if port < 0 or port > 65535:
            raise ValueError("Port number must be between 0 and 65535.")
        return port
    except ValueError as ve:
        unreal.log_error(f"Invalid port number: {ve}")
        sys.exit(1)

# Main execution
if __name__ == '__main__':
    # Get port number from command-line arguments
    port_number = get_port_from_arguments()
    
    # Perform Unreal Engine operations in the main thread
    try:
        # Get the game world
        world = unreal.UnrealEditorSubsystem().get_game_world()
        
        # Get the bounds of the actors with the specified tag
        center, extents = unreal.GameplayStatics.get_actor_array_bounds(
            unreal.GameplayStatics.get_all_actors_with_tag(world, "AirSimWorldBounds"), False
        )
        
        unreal.log('Retrieved world bounds, getting ready to send to airsim')
        
        # Start a new thread to run the socket operations with the specified port
        socket_thread = threading.Thread(target=send_world_bounds, args=(center, extents, port_number))
        socket_thread.start()
    except Exception as e:
        unreal.log_error(f"Error in Unreal operations: {e}")
