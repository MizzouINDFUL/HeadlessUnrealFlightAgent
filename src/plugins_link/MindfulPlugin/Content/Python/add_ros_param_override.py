import unreal
import sys
from unreal import Vector

rosbridge_port = 9090

args = unreal.SystemLibrary.get_command_line()
tokens, switches, params = unreal.SystemLibrary.parse_command_line(args)

if "rosbridge_port" in params:
    rosbridge_port = int(params["rosbridge_port"])
    unreal.log("rosbridge_port: " + str(rosbridge_port))

# Get the editor actor subsystem
actor_subsys = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)

lst_actors = unreal.EditorLevelLibrary.get_all_level_actors()

for act in lst_actors:
    act_label = act.get_actor_label()
    if 'ROSBridgeParamOverride' in act_label:
        print(act_label)
        if len(sys.argv) > 1:
            act.set_editor_property("ROSBridgeServerPort", rosbridge_port)

        quit()

# Actually spawn the actor
spawn_location = unreal.Vector(0, 0, 0)
inst = actor_subsys.spawn_actor_from_class(unreal.ROSBridgeParamOverride, spawn_location)

inst.set_editor_property("ROSBridgeServerPort", rosbridge_port)