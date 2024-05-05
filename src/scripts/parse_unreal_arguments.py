import yaml
import os
import sys

if (len(sys.argv) < 2):
    print("Please provide the path to the config file")
    sys.exit(1)

def try_parse_value(value, full_config) -> str:

    entries = value.split('.')

    #if the first entry doesnt exist in the config, return false right away and dont bother iterating
    if not full_config[str(entries[0])]:
        return ""
    else:
        curr_entry = full_config[entries[0]]
        for i in range(1, len(entries)):

            if isinstance(curr_entry, list):
                for item in curr_entry:
                    if entries[i] in item:
                        curr_entry = item
                        break

            if entries[i] not in curr_entry:
                return ""
            curr_entry = curr_entry[str(entries[i])]
        return str(curr_entry)

config_file_path = sys.argv[1]

with open(config_file_path, 'r') as file:
    data = yaml.safe_load(file)

unreal_arguments = data['unreal']['arguments']
processed_arguments = []

for argument in unreal_arguments:
    arg_as_string = str(argument)
    if ":" in arg_as_string:
        arg_as_string = arg_as_string.replace(": ", "=").replace(" ", "")

        #get value of the dict argument
        key = arg_as_string.split("=")[0]
        value = arg_as_string.split("=")[1]

        #remove the first and the last symbols (' symbols)
        value = value[1:-1]

        #check if value starts with ${ and ends with }
        if ":{" in value and "}" in value:
            #remove ${
            value = value[2:]

            #remove }
            value = value[:-1]
            value = value[:-1]

            maybe_value = try_parse_value(value, data)
            if maybe_value != "":
                arg_as_string = key + "=" + maybe_value
    
    processed_arguments.append("-" + arg_as_string)

arguments_string = " ".join(processed_arguments)
arguments_string = arguments_string.replace("{", "").replace("}", "").replace("'", "")

output_file_name = data['sessionname'] + '-cmd.txt' if 'sessionname' in data else 'unreal_arguments.txt'
output_directory = data['unreal']['project_path'] if 'project_path' in data['unreal'] else os.getcwd()
output_file_path = os.path.join(output_directory, output_file_name)

with open(output_file_path, 'w') as file:
    file.write(arguments_string)