import unreal
import sys
import time

curr_life = int(sys.argv[1]) + 1
total_lives = int(sys.argv[2])

time_of_day_alpha = (curr_life % 10) / 10

#ensure that 10th life is 1
if curr_life % 10 == 0:
    time_of_day_alpha = 1

alpha_invered = 1 - time_of_day_alpha

command = "py add_simple_sky.py {}".format(time_of_day_alpha)
command2 = "py set_exposure.py {}".format(alpha_invered)

unreal.SystemLibrary.execute_console_command(None, command)

time.sleep(0.15)

unreal.SystemLibrary.execute_console_command(None, command2)