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
    # 10: {"ir_ipu": ["0-0-0", "1-1-1"],
    #     "proximity_ipu": ["2-1-13", "0-0-3", "0-0-10", "0-0-20"],
    #     "led_opu": ["5-0-1", "7-0-0"]},
    2: {},
    3: {},
    4: {"led_opu": ["5-0-1"]},
    # 15: {"proximity_ipu": ["2-1-13"], "ir_ipu": ["0-0-0", "1-1-1"]},
    # 16: {"proximity_ipu": ["2-1-13"], "ir_ipu": ["0-0-0", "1-1-1"]},
    # 17: {"proximity_ipu": ["2-1-13"], "ir_ipu": ["0-0-0", "1-1-1"]},
    # 18: {"proximity_ipu": ["2-1-13"], "ir_ipu": ["0-0-0", "1-1-1"]},
    19: {"proximity_ipu": ["2-1-13"]},
    20: {"proximity_ipu": ["2-1-13"]},
    21: {"proximity_ipu": ["2-1-13"]},
    22: {"proximity_ipu": ["2-1-13"]},
    23: {"proximity_ipu": ["2-1-13"]},
    400: {"motor_opu": ["3-0-10", "3-0-11", "3-0-12"]},
    401: {"motor_opu": ["1-0-2", "1-0-3", "1-0-4"]},
    402: {"motor_opu": ["2-0-12", "2-0-13"]},
    403: {"motor_opu": ["3-0-13", "3-0-1", "3-0-2"]},
    }
