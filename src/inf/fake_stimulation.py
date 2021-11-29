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
    11: {"motor_thalamus": ["0-0-5"]},
    12: {"motor_thalamus": ["0-0-15"]},
    13: {"motor_memory": ["0-0-0"]},
    14: {"motor_memory": ["0-0-7"]},
    15: {"motor_ipu": ["0-0-5"]},
    16: {"motor_ipu": ["0-0-15"]},
    17: {"motor_opu": ["0-0-5"]},
    18: {"motor_opu": ["0-0-15"]},
    }
