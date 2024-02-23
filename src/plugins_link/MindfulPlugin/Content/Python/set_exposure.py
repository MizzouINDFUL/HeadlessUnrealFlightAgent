import unreal
import sys
from unreal import Vector

lst_actors = unreal.EditorLevelLibrary.get_all_level_actors()

pp_volume_name = "PostProcessVolume"
dl_name = "Light Source"
new_exposure = 0.0
if len(sys.argv) > 1:
	new_exposure = float(sys.argv[1])
	if len(sys.argv) > 2:
		pp_volume_name = sys.argv[2]

for act in lst_actors:
	act_label = act.get_actor_label()
	if pp_volume_name in act_label:
		act.settings.override_auto_exposure_bias = True
		act.settings.auto_exposure_bias = new_exposure
	elif dl_name in act_label:
		lightComp = act.light_component
		lightComp.set_editor_property("Intensity", new_exposure)
		print("modified light source intensity to " + str(new_exposure))
	
