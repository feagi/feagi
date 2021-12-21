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
    # 11: {"servo_opu": ["0-0-90", "1-0-0"]},
    # 15: {"servo_opu": ["0-0-90", "1-0-90"]},
    # 20: {"servo_opu": ["0-0-90", "1-0-180"]},
    # 25: {"servo_opu": ["0-0-90", "1-0-90"]},
    # 30: {"servo_opu": ["0-0-0", "1-0-90"]},
    # 35: {"servo_opu": ["0-0-90", "1-0-90"]},
    # 40: {"servo_opu": ["0-0-180", "1-0-90"]},
    # 11: {"battery_ipu": ["0-0-9"]},
    # 12: {"battery_ipu": ["0-0-8"]},
    # 13: {"battery_ipu": ["0-0-7"]},
    # 14: {"battery_ipu": ["0-0-6"]},
    # 15: {"battery_ipu": ["0-0-5"]},
    # 16: {"battery_ipu": ["0-0-4"]},
    # 17: {"battery_ipu": ["0-0-3"]},
    # 18: {"battery_ipu": ["0-0-2"]},
    # 19: {"battery_ipu": ["0-0-1"]},
    # 11: {"motor_thalamus": ["0-0-5", "3-0-6", "0-0-11", "1-0-15"]},
    # 12: {"motor_thalamus": ["2-0-1", "1-0-4", "3-0-5", "0-0-14", "0-0-1", "1-0-10"]},
    # 13: {"motor_memory": ["0-0-0", "0-0-1", "0-0-2", "0-0-5", "0-0-6"]},
    # 14: {"motor_memory": ["0-0-7", "0-0-5", "0-0-2", "0-0-1", "0-0-0"]},
    # 15: {"motor_ipu": ["3-0-1", "0-0-2", "1-0-3", "1-0-4", "2-0-5"]},
    # 16: {"motor_ipu": ["0-0-5", "1-0-4", "2-0-3", "3-0-2"]},
    # 17: {"motor_opu": ["0-0-1", "0-0-15", "0-0-5", "0-0-0", "0-0-1"]},
    # 18: {"motor_opu": ["2-0-15", "0-0-5", "0-0-11", "3-0-0", "1-0-9"]},
    # 19: {"motor_thalamus": ["0-0-5", "3-0-6", "0-0-11", "1-0-15"]},
    # 20: {"motor_thalamus": ["2-0-1", "1-0-4", "3-0-5", "0-0-14", "0-0-1", "1-0-10"]},
    # 21: {"motor_memory": ["0-0-0", "0-0-1", "0-0-2", "0-0-5", "0-0-6"]},
    # 22: {"motor_memory": ["0-0-7", "0-0-5", "0-0-2", "0-0-1", "0-0-0"]},
    # 23: {"motor_ipu": ["3-0-1", "0-0-2", "1-0-3", "1-0-4", "2-0-5"]},
    # 24: {"motor_ipu": ["0-0-5", "1-0-4", "2-0-3", "3-0-2"]},
    # 25: {"motor_opu": ["0-0-1", "0-0-15", "0-0-5", "0-0-0", "0-0-1"]},
    # 26: {"motor_opu": ["2-0-15", "0-0-5", "0-0-11", "3-0-0", "1-0-9"]},
    # 27: {"motor_thalamus": ["0-0-5", "3-0-6", "0-0-11", "1-0-15"]},
    # 28: {"motor_thalamus": ["2-0-1", "1-0-4", "3-0-5", "0-0-14", "0-0-1", "1-0-10"]},
    # 29: {"motor_memory": ["0-0-0", "0-0-1", "0-0-2", "0-0-5", "0-0-6"]},
    # 30: {"motor_memory": ["0-0-7", "0-0-5", "0-0-2", "0-0-1", "0-0-0"]},
    # 31: {"motor_ipu": ["3-0-1", "0-0-2", "1-0-3", "1-0-4", "2-0-5"]},
    # 32: {"motor_ipu": ["0-0-5", "1-0-4", "2-0-3", "3-0-2"]},
    # 33: {"motor_opu": ["0-0-1", "0-0-15", "0-0-5", "0-0-0", "0-0-1"]},
    # 34: {"motor_opu": ["2-0-15", "0-0-5", "0-0-11", "3-0-0", "1-0-9"]},
    # 35: {"motor_thalamus": ["0-0-5", "3-0-6", "0-0-11", "1-0-15"]},
    # 36: {"motor_thalamus": ["2-0-1", "1-0-4", "3-0-5", "0-0-14", "0-0-1", "1-0-10"]},
    # 37: {"motor_memory": ["0-0-0", "0-0-1", "0-0-2", "0-0-5", "0-0-6"]},
    # 38: {"motor_memory": ["0-0-7", "0-0-5", "0-0-2", "0-0-1", "0-0-0"]},
    # 39: {"motor_ipu": ["3-0-1", "0-0-2", "1-0-3", "1-0-4", "2-0-5"]},
    # 40: {"motor_ipu": ["0-0-5", "1-0-4", "2-0-3", "3-0-2"]},
    # 41: {"motor_opu": ["0-0-1", "0-0-15", "0-0-5", "0-0-0", "0-0-1"]},
    # 42: {"motor_opu": ["2-0-15", "0-0-5", "0-0-11", "3-0-0", "1-0-9"]},
    # 43: {"motor_thalamus": ["0-0-5", "3-0-6", "0-0-11", "1-0-15"]},
    # 44: {"motor_thalamus": ["2-0-1", "1-0-4", "3-0-5", "0-0-14", "0-0-1", "1-0-10"]},
    # 45: {"motor_memory": ["0-0-0", "0-0-1", "0-0-2", "0-0-5", "0-0-6"]},
    # 46: {"motor_memory": ["0-0-7", "0-0-5", "0-0-2", "0-0-1", "0-0-0"]},
    # 47: {"motor_ipu": ["3-0-1", "0-0-2", "1-0-3", "1-0-4", "2-0-5"]},
    # 48: {"motor_ipu": ["0-0-5", "1-0-4", "2-0-3", "3-0-2"]},
    # 49: {"motor_opu": ["0-0-1", "0-0-15", "0-0-5", "0-0-0", "0-0-1"]},
    # 50: {"motor_opu": ["2-0-15", "0-0-5", "0-0-11", "3-0-0", "1-0-9"]},
    # 11: {"motor_thalamus": ["0-0-5", "3-0-4", "0-0-0", "1-2-3"]},
    # 12: {"motor_thalamus": ["2-0-1", "1-0-4", "3-1-1", "4-0-4", "0-0-1", "1-0-3"]},
    # 13: {"motor_thalamus": ["0-0-5", "3-0-4", "0-0-0", "1-2-3"]},
    # 14: {"motor_thalamus": ["2-0-1", "1-0-4", "3-1-1", "4-0-4", "0-0-1", "1-0-3"]},
    # 15: {"motor_thalamus": ["0-0-5", "3-0-4", "0-0-0", "1-2-3"]},
    # 16: {"motor_thalamus": ["2-0-1", "1-0-4", "3-1-1", "4-0-4", "0-0-1", "1-0-3"]},
    # 17: {"motor_thalamus": ["0-0-5", "3-0-4", "0-0-0", "1-2-3"]},
    # 18: {"motor_thalamus": ["2-0-1", "1-0-4", "3-1-1", "4-0-4", "0-0-1", "1-0-3"]},
    # 19: {"motor_thalamus": ["0-0-5", "3-0-4", "0-0-0", "1-2-3"]},
    # 20: {"motor_thalamus": ["2-0-1", "1-0-4", "3-1-1", "4-0-4", "0-0-1", "1-0-3"]},
    11: {"motor_thalamus": ["2-0-1", "1-0-4", "3-1-1", "4-0-4", "0-0-1", "1-0-3"]},
    12: {"motor_thalamus": ["2-0-1", "1-0-4", "3-1-1", "4-0-4", "0-0-1", "1-0-3"]},
    13: {"motor_thalamus": ["2-0-1", "1-0-4", "3-1-1", "4-0-4", "0-0-1", "1-0-3"]},
    14: {"motor_thalamus": ["2-0-1", "1-0-4", "3-1-1", "4-0-4", "0-0-1", "1-0-3"]},
    15: {"motor_thalamus": ["2-0-1", "1-0-4", "3-1-1", "4-0-4", "0-0-1", "1-0-3"]},
    16: {"motor_thalamus": ["2-0-1", "1-0-4", "3-1-1", "4-0-4", "0-0-1", "1-0-3"]},
    17: {"motor_thalamus": ["0-0-2"]},
    18: {"motor_thalamus": ["0-0-3"]},
    19: {"motor_thalamus": ["0-0-0"]},
    20: {"motor_thalamus": ["0-0-1"]},
    21: {"motor_thalamus": ["0-0-2"]},
    22: {"motor_thalamus": ["0-0-3"]},
    23: {"motor_thalamus": ["0-0-0"]},
    24: {"motor_thalamus": ["0-0-1"]},
    25: {"motor_thalamus": ["0-0-2"]},
    26: {"motor_thalamus": ["0-0-3"]},
    27: {"motor_thalamus": ["0-0-0"]},
    28: {"motor_thalamus": ["0-0-1"]},
    29: {"motor_thalamus": ["0-0-2"]},
    30: {"motor_thalamus": ["0-0-3"]},
    141: {"motor_opu": ["0-0-15", "1-0-15", "2-0-15", "3-0-15"]},
    142: {"motor_opu": ["0-0-15", "1-0-15", "2-0-15", "3-0-15"]},
    143: {"motor_opu": ["0-0-15", "1-0-15", "2-0-15", "3-0-15"]},
    144: {"motor_opu": ["0-0-15", "1-0-15", "2-0-15", "3-0-15"]},
    145: {"motor_opu": ["0-0-15", "1-0-15", "2-0-15", "3-0-15"]},
    146: {"motor_opu": ["0-0-15", "1-0-15", "2-0-15", "3-0-15"]},
    147: {"motor_opu": ["0-0-15", "1-0-15", "2-0-15", "3-0-15"]},
    148: {"motor_opu": ["0-0-15", "1-0-15", "2-0-15", "3-0-15"]},
    149: {"motor_opu": ["0-0-15", "1-0-15", "2-0-15", "3-0-15"]},
    150: {"motor_opu": ["0-0-15", "1-0-15", "2-0-15", "3-0-15"]},
    151: {"motor_opu": ["0-0-15", "1-0-15", "2-0-15", "3-0-15"]},
    152: {"motor_opu": ["0-0-15", "1-0-15", "2-0-15", "3-0-15"]},
    153: {"motor_opu": ["0-0-15", "1-0-15", "2-0-15", "3-0-15"]},
    154: {"motor_opu": ["0-0-15", "1-0-15", "2-0-15", "3-0-15"]},
    155: {"motor_opu": ["0-0-15", "1-0-15", "2-0-15", "3-0-15"]},
    156: {"motor_opu": ["0-0-15", "1-0-15", "2-0-15", "3-0-15"]},
    157: {"motor_opu": ["0-0-15", "1-0-15", "2-0-15", "3-0-15"]},
    158: {"motor_opu": ["0-0-15", "1-0-15", "2-0-15", "3-0-15"]},
    159: {"motor_opu": ["0-0-15", "1-0-15", "2-0-15", "3-0-15"]},
    160: {"motor_opu": ["0-0-15", "1-0-15", "2-0-15", "3-0-15"]},
    161: {"motor_opu": ["0-0-15", "1-0-15", "2-0-15", "3-0-15"]},
    162: {"motor_opu": ["0-0-15", "1-0-15", "2-0-15", "3-0-15"]},
    163: {"motor_opu": ["0-0-15", "1-0-15", "2-0-15", "3-0-15"]},
    164: {"motor_opu": ["0-0-15", "1-0-15", "2-0-15", "3-0-15"]},
    165: {"motor_opu": ["0-0-15", "1-0-15", "2-0-15", "3-0-15"]},
    166: {"motor_opu": ["0-0-15", "1-0-15", "2-0-15", "3-0-15"]},
    167: {"motor_opu": ["0-0-15", "1-0-15", "2-0-15", "3-0-15"]},
    168: {"motor_opu": ["0-0-15", "1-0-15", "2-0-15", "3-0-15"]},
    169: {"motor_opu": ["0-0-15", "1-0-15", "2-0-15", "3-0-15"]},
    170: {"motor_opu": ["0-0-15", "1-0-15", "2-0-15", "3-0-15"]},
    171: {"motor_opu": ["0-0-15", "1-0-15", "2-0-15", "3-0-15"]},
    172: {"motor_opu": ["0-0-15", "1-0-15", "2-0-15", "3-0-15"]},
    173: {"motor_opu": ["0-0-15", "1-0-15", "2-0-15", "3-0-15"]},
    174: {"motor_opu": ["0-0-15", "1-0-15", "2-0-15", "3-0-15"]},
    175: {"motor_opu": ["0-0-15", "1-0-15", "2-0-15", "3-0-15"]},
    176: {"motor_opu": ["0-0-15", "1-0-15", "2-0-15", "3-0-15"]},
    177: {"motor_opu": ["0-0-15", "1-0-15", "2-0-15", "3-0-15"]},
    }
