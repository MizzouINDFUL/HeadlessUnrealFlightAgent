import sys
import unreal

#make sure that there is at least one argument provided that indicated currrent life index
if len(sys.argv) < 2:
    print("Please provide the current life index as an argument.")
    sys.exit()

#parse the life index
life_idx = int(sys.argv[1])

args = unreal.SystemLibrary.get_command_line()
tokens, switches, params = unreal.SystemLibrary.parse_command_line(args)

#an example of levels_to_load parameter: "1:FlyingExampleMap+3:FlyingExampleMap2". This will load FlyingExampleMap if life index is 1 and FlyingExampleMap2 if life index is 3
loaded_new_level = False
levels_to_load=""
if "levels_to_load" in params:
    levels_to_load = params["levels_to_load"]
    unreal.log("levels_to_load: " + levels_to_load)
    #split string using "+" separator
    levels = levels_to_load.split("+")
    #iterate over all levels
    for level in levels:
        #split level string using ":" separator
        level_data = level.split(":")
        #if life index matches the one from the parameter
        if int(level_data[0]) == life_idx:
            #load the level
            print("Attempting to load level: " + level_data[1])
            unreal.EditorLoadingAndSavingUtils.load_map(level_data[1])
            loaded_new_level = True
            print("READY TO START SIM")
            break

print("READY TO START SIM")