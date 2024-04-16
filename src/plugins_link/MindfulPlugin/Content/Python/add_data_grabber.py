import unreal
import sys
from unreal import Vector

#needs to work while editor is in game
lst_actors = unreal.EditorLevelLibrary.get_all_level_actors()

flying_pawn_name_start = 'BP_FlyingPawn'

for act in lst_actors:
    act_label = act.get_actor_label()
    if flying_pawn_name_start in act_label:
        print(act_label)

#unreal.EditorLevelLibrary.get_editor_world()