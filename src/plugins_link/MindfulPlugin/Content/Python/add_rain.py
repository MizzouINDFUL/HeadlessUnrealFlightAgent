import unreal
import sys
from unreal import Vector

# Get a class for spawning
my_class = unreal.EditorAssetLibrary.load_blueprint_class('/MindfulPlugin/MRQ/BP_MRQ_Rain.BP_MRQ_Rain')

# Get the editor actor subsystem
actor_subsys = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)

lst_actors = unreal.EditorLevelLibrary.get_all_level_actors()

for act in lst_actors:
    act_label = act.get_actor_label()
    if 'BP_MRQ_Rain' in act_label:
        print(act_label)
        if len(sys.argv) > 1:
            val = float(sys.argv[1])
            act.set_editor_property("Intensity", val)

            children = act.get_components_by_class(unreal.PrimitiveComponent)
            for child in children:
                child.set_render_custom_depth(False)
        quit()

# Actually spawn the actor
spawn_location = unreal.Vector(0, 0, 0)
inst = actor_subsys.spawn_actor_from_class(my_class, spawn_location)

if len(sys.argv) > 1:
    val = float(sys.argv[1])
    inst.set_editor_property("Intensity", val)