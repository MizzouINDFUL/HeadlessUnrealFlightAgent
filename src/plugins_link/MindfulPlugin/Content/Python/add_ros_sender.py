import unreal
import sys

# Get a class for spawning
my_class = unreal.EditorAssetLibrary.load_blueprint_class('/MindfulPlugin/MRQ/BP_ROSSender.BP_ROSSender')

# Get the editor actor subsystem
actor_subsys = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)

lst_actors = unreal.EditorLevelLibrary.get_all_level_actors()

for act in lst_actors:
    act_label = act.get_actor_label()
    if 'BP_ROSSender' in act_label:
        quit()

# Actually spawn the actor
spawn_location = unreal.Vector(0, 0, 0)
inst = actor_subsys.spawn_actor_from_class(my_class, spawn_location)