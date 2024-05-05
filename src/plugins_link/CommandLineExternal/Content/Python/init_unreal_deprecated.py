import unreal
import glob
import os
import time

class CommandLineReader():
    def __init__(self) -> None:

        self.command_file = unreal.Paths.project_plugins_dir() + "CommandLineExternal/command.txt"

        tickhandle = unreal.register_slate_pre_tick_callback(self.tick)

        print("External Command Line object is initialized.")
        print("command_file: ", self.command_file)
    
    def tick(self, delta_time):
        self.read_file_commands()

    def read_file_commands(self):

        if os.path.exists(self.command_file):
            with open(self.command_file, "r") as f:
                command = f.read()
            #check if the string is empty
            if not command:
                return
            #execute the command
            unreal.SystemLibrary.execute_console_command(None, command)

            #empty the file
            with open(self.command_file, "w") as f:
                f.write("")

CommandLineReader()
