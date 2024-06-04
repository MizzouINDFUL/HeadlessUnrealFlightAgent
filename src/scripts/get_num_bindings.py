import yaml
import sys

def main(yaml_file):
    with open(yaml_file, 'r') as file:
        data = yaml.safe_load(file)

    total_calls = 0
    for event, bindings in data['script_bindings'].items():
        total_calls += len(bindings) if isinstance(bindings, list) else 1

    print(total_calls)

if __name__ == "__main__":
    main(sys.argv[1])
