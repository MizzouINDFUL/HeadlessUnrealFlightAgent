import unreal
import sys
from unreal import Vector

# Get a class for spawning
my_class = unreal.EditorAssetLibrary.load_blueprint_class('/MindfulPlugin/BP_SimplerSky.BP_SimplerSky')

# Get the editor actor subsystem
actor_subsys = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)

lst_actors = unreal.EditorLevelLibrary.get_all_level_actors()

for act in lst_actors:
    act_label = act.get_actor_label()
    if 'BP_SimplerSky' in act_label:
        print(act_label)
        if len(sys.argv) > 1:
            val = float(sys.argv[1])
            scale = Vector(1.0, 1.0 ,1.0)
            if len(sys.argv) > 2:
                scale = Vector(float(sys.argv[2]), float(sys.argv[2]), float(sys.argv[2]))
            act.set_editor_property("Progress", val)
            act.set_actor_scale3d(scale)
        quit()

# Actually spawn the actor
spawn_location = unreal.Vector(0, 0, 0)
inst = actor_subsys.spawn_actor_from_class(my_class, spawn_location)

if len(sys.argv) > 1:
    val = float(sys.argv[1])
    inst.set_editor_property("Progress", val)
    scale = Vector(1.0, 1.0 ,1.0)
    if len(sys.argv) > 2:
        scale = Vector(float(sys.argv[2]), float(sys.argv[2]), float(sys.argv[2]))
    inst.set_actor_scale3d(scale)