
"""
This module contains a virtual set of hardware controllers used for simulation and testing only

todo: Need to have all of the controller.py modules follow the same convention and be consistent especially input data
"""

from inf import runtime_data


def stimulate():
    stimuli = dict()
    try:
        for play in runtime_data.stimulation_script:
            if "counter" not in runtime_data.stimulation_script[play]:
                runtime_data.stimulation_script[play]["counter"] = 0
                runtime_data.stimulation_script[play]["instance_repeat"] = 0

            play_length = len(runtime_data.stimulation_script[play]["definition"])
            try:
                if runtime_data.stimulation_script[play]["counter"] == runtime_data.stimulation_script[play]["repeat"]:
                    continue
                if play_length == 0:
                    continue
            except KeyError:
                pass
            if play not in runtime_data.stimulation_index:
                runtime_data.stimulation_index[play] = 0
            print("\nStimulator sequence:")
            print("--------@  @  @  @  @  @  @  @  @---------")
            print(play, runtime_data.stimulation_script[play]["counter"], runtime_data.stimulation_script[play]["instance_repeat"])
            print("--------@  @  @  @  @  @  @  @  @---------\n")

            for cortical_area in runtime_data.stimulation_script[play]["definition"][runtime_data.stimulation_index[play]][0]:
                if cortical_area in runtime_data.cortical_list:
                    stimuli[cortical_area] = runtime_data.stimulation_script[play]["definition"][runtime_data.stimulation_index[play]][0][cortical_area]

            runtime_data.stimulation_script[play]["instance_repeat"] += 1
            if runtime_data.stimulation_script[play]["instance_repeat"] == runtime_data.stimulation_script[play]["definition"][runtime_data.stimulation_index[play]][1]:
                runtime_data.stimulation_index[play] += 1
                runtime_data.stimulation_script[play]["instance_repeat"] = 0
            if runtime_data.stimulation_index[play] == play_length:
                runtime_data.stimulation_index[play] = 0
                runtime_data.stimulation_script[play]["counter"] += 1
    except Exception as e:
        print("\n\nError: Unsupported stimulation script format", e)

    return stimuli

#
# def build_message_to_feagi():
#     """
#     This function encodes the sensory information in a dictionary that can be decoded on the FEAGI end.
#
#     expected ipu_data structure:
#
#         ipu_data = {
#             "capabilities": {},
#             "network": {},
#             "data": {
#                 "direct_stimulation": {
#                     "cortical_area_id": {voxel},
#                     "cortical_area_id": sensor_data,
#                     "cortical_area_id": sensor_data
#                     ...
#                     },
#                 "sensory_data": {
#                     "sensor type": sensor data,
#                     "sensor type": sensor data,
#                     "sensor type": sensor data,
#                     ...
#                 }
#             }
#     """
#
#     message = dict()
#     message["controller_burst_id"] = runtime_data["current_burst_id"]
#     message['data'] = dict()
#     message['data']["direct_stimulation"] = dict()
#     message['data']["sensory_data"] = dict()
#     message['data']["direct_stimulation"] = stimulate(runtime_data["current_burst_id"])
#
#     return message
