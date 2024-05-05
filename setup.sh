THISFOLDER=$(dirname $(readlink -f $0))

# Check if --clear flag is passed
if [[ $1 == "--clear" ]]; then
    echo "Clearing Docker images..."
    docker rmi unreal-launcher-airsim-ros
    docker rmi ultralytics/ultralytics
    docker rmi ghcr.io/epicgames/unreal-engine:dev-5.1.0
    exit 0
elif [[ $1 == "--help" ]]; then
    echo "Usage: ./setup.sh [--clear]"
    echo "  --clear: Clear all Docker images"
    exit 0
fi

#create a link folder called agent that leads to src/airsim-ros/shared/src/agent folder
ln -s $THISFOLDER/src/airsim-ros/shared/src/agent $THISFOLDER/agent 

# Create a new session and detach from it
tmux new-session -d -s unreal-setup

echo "Installing ROS + AirSim docker image. You can check the progress by running 'tmux attach -t unreal-setup'"

tmux new-window -t unreal-setup
tmux send-keys -t unreal-setup "cd $THISFOLDER/src/airsim-ros; docker build -t unreal-launcher-airsim-ros .; exit" C-m

echo "Installing a YOLO docker image. You can check the progress by running 'tmux attach -t unreal-setup'"
cd $THISFOLDER/src/yolo
tmux new-window -t unreal-setup
tmux send-keys -t unreal-setup "cd $THISFOLDER/src/yolo; ./setup.sh; exit" C-m

#ask users if they want to install an unreal docker image
echo "Do you want to install an Unreal Engine docker image? (y/n)"
read install_unreal_docker

if [ $install_unreal_docker == "y" ]; then
    echo "Enter your GitHub username (make sure it has access to Epic's GitHub repo):"
    read github_username
    echo "Enter your GitHub token (make sure it has write:packages and read:packages checked):"
    read github_token

    export CR_PATH=$github_token
    echo $CR_PAT | docker login ghcr.io -u $github_username --password-stdin
    echo "Pulling Unreal Engine docker 5.1. This may take a while..."
    echo "The installation will continue in the background inside a tmux session."
    echo "You can check the progress by running 'tmux attach -t unreal-setup'"
    tmux new-window -t unreal-setup
    tmux send-keys -t unreal-setup "docker pull ghcr.io/epicgames/unreal-engine:dev-5.1.0; exit" C-m
else
    echo "Skipping Unreal Engine docker image installation"
    echo "Setup is complete. You can now run the simulator by running ./run.sh"
fi