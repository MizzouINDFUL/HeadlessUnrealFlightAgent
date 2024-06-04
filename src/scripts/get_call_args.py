import yaml
import sys

def main(yaml_file, index, arg_index=None):
    with open(yaml_file, 'r') as file:
        data = yaml.safe_load(file)

    calls = []
    for event, bindings in data['script_bindings'].items():
        if isinstance(bindings, list):
            for binding in bindings:
                script = binding['script']
                calls.append((event, script, str(binding.get('clear_logs_on_complete', False)).lower(), str(binding.get('auto_rebind_after_each_life', False)).lower()))
        else:
            script = bindings['script']

            calls.append((event, script, str(bindings.get('clear_logs_on_complete', False)).lower(), str(bindings.get('auto_rebind_after_each_life', False)).lower()))

    if arg_index is not None:
        print(calls[index][arg_index])
    else:
        print(' '.join(arg for arg in calls[index]))

if __name__ == "__main__":
    if len(sys.argv) == 4:
        main(sys.argv[1], int(sys.argv[2]), int(sys.argv[3]))
    elif len(sys.argv) == 3:
        main(sys.argv[1], int(sys.argv[2]))