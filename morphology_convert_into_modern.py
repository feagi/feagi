import json

# Opening JSON file
f = open('test.json')

# returns JSON object as
# a dictionary
data = json.load(f)
name_stored = {"neuron_morphologies": {}}

for i in data["neuron_morphologies"]:
    for x in data["neuron_morphologies"][i]:
        if "parameters" in data["neuron_morphologies"][i]:
            pass
        else:
            if isinstance(data["neuron_morphologies"][i][x], bool):
                name_stored["neuron_morphologies"][i] = dict()
                name_stored["neuron_morphologies"][i]["parameters"] = dict()
                name_stored["neuron_morphologies"][i]["type"] = dict()
                name_stored["neuron_morphologies"][i]["parameters"][x] = data["neuron_morphologies"][i][x]
                name_stored["neuron_morphologies"][i]["type"] = x
            else:
                name_stored["neuron_morphologies"][i] = dict()
                name_stored["neuron_morphologies"][i]["parameters"] = dict()
                name_stored["neuron_morphologies"][i]["type"] = dict()
                if x == "patterns":
                    name_stored["neuron_morphologies"][i]["parameters"][x] = [data["neuron_morphologies"][i][x]]
                if x == "vectors":
                    name_stored["neuron_morphologies"][i]["parameters"][x] = data["neuron_morphologies"][i][x]
                name_stored["neuron_morphologies"][i]["type"] = x

data["neuron_morphologies"] = name_stored["neuron_morphologies"]
with open("new_json.json", "w") as data_file:
    data_file.seek(0)  # rewind
    data_file.write(json.dumps(data, indent=3))
    data_file.truncate()
f.close()