import unreal

lst_actors = unreal.EditorLevelLibrary.get_all_level_actors()

for act in lst_actors:
	act_label = act.get_actor_label()
	print(act_label)