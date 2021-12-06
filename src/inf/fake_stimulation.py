"""
The fake_stimulation will provide one or multiple stimulations that can be played in sequence against the brain.
each instance of the list will be injected to a single burst. if a sequence is left empty then the corresponding burst
sequence will go without injection.

Note: This method can help us record brain activities and have them replayed back.

Every key in the stimulation_data dictionary is corresponding to the corresponding burst_id that the stimuli exposure is
to happen.

# todo: need to introduce a mechanism so a sequence can be replaced in a loop

"""

stimulation_data = {
    11: {"battery_ipu": ["0-0-9"]},
    12: {"battery_ipu": ["0-0-9"]},
    13: {"battery_ipu": ["0-0-9"]},
    14: {"battery_ipu": ["0-0-8"]},
    15: {"battery_ipu": ["0-0-8"]},
    16: {"battery_ipu": ["0-0-7"]},
    17: {"battery_ipu": ["0-0-7"]},
    18: {"battery_ipu": ["0-0-6"]},
    19: {"battery_ipu": ["0-0-6"]},
    20: {"battery_ipu": ["0-0-5"]},
    21: {"battery_ipu": ["0-0-5"]},
    22: {"battery_ipu": ["0-0-4"]},
    23: {"battery_ipu": ["0-0-4"]},
    24: {"battery_ipu": ["0-0-3"]},
    25: {"battery_ipu": ["0-0-3"]},
    26: {"battery_ipu": ["0-0-2"]},
    27: {"battery_ipu": ["0-0-2"]},
    28: {"battery_ipu": ["0-0-1"]},
    29: {"battery_ipu": ["0-0-1"]},
    30: {"battery_ipu": ["0-0-0"]},
    31: {"battery_ipu": ["0-0-0"]},
    # 11: {"proximity_ipu": ["0-0-1"]},
    # 12: {"proximity_ipu": ["0-0-2"]},
    # 13: {"proximity_ipu": ["0-0-3"]},
    # 14: {"proximity_ipu": ["0-0-4"]},
    # 15: {"proximity_ipu": ["0-0-5"]},
    # 16: {"proximity_ipu": ["0-0-6"]},
    # 17: {"proximity_ipu": ["0-0-7"]},
    # 18: {"proximity_ipu": ["0-0-8"]},
    # 19: {"proximity_ipu": ["0-0-9"]},
    # 20: {"proximity_ipu": ["0-0-10"]},
    # 21: {"proximity_ipu": ["0-0-11"]},
    # 22: {"proximity_ipu": ["0-0-12"]},
    # 23: {"proximity_ipu": ["0-0-13"]},
    # 24: {"proximity_ipu": ["0-0-14"]},
    # 25: {"proximity_ipu": ["0-0-15"]},
    # 26: {"proximity_ipu": ["0-0-16"]},
    # 27: {"proximity_ipu": ["0-0-17"]},
    # 28: {"proximity_ipu": ["0-0-18"]},
    # 11: {"proximity_ipu": ["0-0-18"]},
    # 12: {"proximity_ipu": ["0-0-17"]},
    # 13: {"proximity_ipu": ["0-0-16"]},
    # 14: {"proximity_ipu": ["0-0-15"]},
    # 15: {"proximity_ipu": ["0-0-14"]},
    # 16: {"proximity_ipu": ["0-0-13"]},
    # 17: {"proximity_ipu": ["0-0-12"]},
    # 18: {"proximity_ipu": ["0-0-11"]},
    # 19: {"proximity_ipu": ["0-0-10"]},
    # 20: {"proximity_ipu": ["0-0-9"]},
    # 21: {"proximity_ipu": ["0-0-8"]},
    # 22: {"proximity_ipu": ["0-0-7"]},
    # 23: {"proximity_ipu": ["0-0-6"]},
    # 24: {"proximity_ipu": ["0-0-5"]},
    # 25: {"proximity_ipu": ["0-0-4"]},
    # 26: {"proximity_ipu": ["0-0-3"]},
    # 27: {"proximity_ipu": ["0-0-2"]},
    # 28: {"proximity_ipu": ["0-0-1"]},
}
