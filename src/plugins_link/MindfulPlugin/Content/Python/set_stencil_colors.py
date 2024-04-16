import unreal

ground_truth_input = "ASplineCar1:car,SM_Ramp2:ramp,SM_Cube4:cube"

args = unreal.SystemLibrary.get_command_line()
tokens, switches, params = unreal.SystemLibrary.parse_command_line(args);

if "ground_truth" in params:
    ground_truth_input = str(params["ground_truth"])

#separate ground_truth_input by coma
gt_requests = ground_truth_input.split(',')
actor_names = []

for gt in gt_requests:
    actor_names.append(gt.split(':')[0])

# Get the editor actor subsystem
actor_subsys = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)

lst_actors = unreal.EditorLevelLibrary.get_all_level_actors()

non_gt_index = len(actor_names)

for act in lst_actors:

    if act.root_component == None:
        continue

    actor_in_ground_truth = False
    act_label = act.get_actor_label()
    
    ground_truth_index = non_gt_index

    for target_actor in actor_names:
        if target_actor == act_label:
            #find the index of the ground truth actor in actor_names
            ground_truth_index = actor_names.index(target_actor)
            print("Ground truth actor found: " + act_label)
            print("With Ground Truth Index: " + str(ground_truth_index))
            print(actor_names)
            break
    children = act.get_components_by_class(unreal.PrimitiveComponent)
    #find all PrimitiveComponents
    for child in children:
        child.set_render_custom_depth(True)
        child.set_custom_depth_stencil_value(ground_truth_index)
