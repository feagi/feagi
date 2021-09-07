
from inf import runtime_data


def convert_char_to_fire_list(char):
    utf_value = ord(char)
    fire_set = set()
    for key in runtime_data.brain["utf8_ipu"]:
        if utf_value == runtime_data.brain["utf8_ipu"][key]['soma_location'][0][2]:
            fire_set.add(key)
    return fire_set
