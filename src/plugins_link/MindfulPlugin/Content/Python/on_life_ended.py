import unreal
import sys
import time

args = unreal.SystemLibrary.get_command_line()
tokens, switches, params = unreal.SystemLibrary.parse_command_line(args)

#an example of levels_to_load parameter: "1:FlyingExampleMap+3:FlyingExampleMap2". This will load FlyingExampleMap if life index is 1 and FlyingExampleMap2 if life index is 3
levels_to_load=""
if "levels_to_load" in params:
    levels_to_load = params["levels_to_load"]
    unreal.log("levels_to_load: " + levels_to_load)
    #spliut string using "+" separator
    levels = levels_to_load.split("+")