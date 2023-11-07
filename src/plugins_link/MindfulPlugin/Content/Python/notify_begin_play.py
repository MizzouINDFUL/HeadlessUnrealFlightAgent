import json

#open ./shared/unreal.json and set begin_play to 1
with open("/shared/unreal.json", 'r') as stream:
    try:
        data = json.load(stream)
        data['begin_play'] = 1
        with open("/shared/unreal.json", 'w') as outfile:
            json.dump(data, outfile)
    except json.JSONDecodeError as exc:
        print(exc)
