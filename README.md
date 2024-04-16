# MizSIM: Headless Unreal Flight Agent

An extension of Headless Unreal Launcher with focus on collecting simulation data from AirSim flights. Every Unreal project laucnhed with this solution will be hooked up with AirSim, ROSIntegration, ROSIntegrationVision, and a couple of other plugins to establish communication between the engine and rosbridge. All data is collected into a rosbag, and then extracted into a folder in the "bags" directory.

## How to use

1. Run __setup.sh__ - this will install docker images you need. Later you may run setup.sh --clean to remove the images from your machine.

2. Run __run.sh__ to start the simulation.

## Dependencies

* Docker
* tmux
* Unreal Engine 5.1 (if using a local build of the engine)

## config.yml
This yaml file exposes parameters that change the behavior of the simulation.

__Category: session__
| Parameter | Description |
| -------- | -------- |
| basename   | basename for tmux sessions    |
| max   | max number of tmux sessions that can be running at the same time    |

__Category: unreal__
| Parameter | Description |
| -------- | -------- |
| separate_session (true/false)   | creates a separate tmux session for the Unreal  instance when true. Otherwise, the engine will be included with your main session in a separate window   |
| start_game (true/false)   | If true, will automatically start the game once the editor is ready   |
| use_docker (true/false)   | When true, the unreal project will be launched from inside a docker container that has a build of Unreal Engine in it. Otherwise it will run a local build of Unreal specified in the _engine_path_ parameter   |
| headless (true/false)   | When true, will run the Unreal project with -RenderOffscreen flag. Set this to true when runnning on a headless server   |
| docker_image | the name of the docker image to run when _use_docker_ is set to true   |
| engine_path   | The root folder of the engine build to use when _use_docker_ is false   |
| project_path   | The root folder of the project to launch   |
| actors_to_tag   | Will add tags to actors in the level. Format: "ActorName:Tag1, ActoName2:Tag2, ..."   |

__Categroy: simulation__

| Parameter | Description |
| -------- | -------- |
| outputSizeY   | Resolution of the output imagery  |
| outputSizeX   | Resolution of the output imagery  |
| store_in_bag   | whether to run "rosbag record" for each life  |
| num_lives   | How many times to run the lives (aka play the game in the editor) within this session  |
| time (seconds)   | how long to run each life   |
| target_is_time   | if true, will treat the "target" varaible as a number of seconds that each simulations needs to be running for. Otherwise, target will be a number of frames   |
| record_delay   | allows for a slight delay before we run "rosbag record"   |
| tags_to_track   | Actors with these tags will be included in the ground truth data. Format: "Tag1, Tag2, ..."   |
| extract_on_end   | When false, this will skip the process of extracting ros bags and running the YOLO detector   |
