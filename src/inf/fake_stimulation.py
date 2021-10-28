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
    11: {"servo_opu": ["0-0-90", "1-0-0"]},
    15: {"servo_opu": ["0-0-90", "1-0-90"]},
    20: {"servo_opu": ["0-0-90", "1-0-180"]},
    25: {"servo_opu": ["0-0-90", "1-0-90"]},
    # 30: {"servo_opu": ["0-0-0", "1-0-90"]},
    # 35: {"servo_opu": ["0-0-90", "1-0-90"]},
    # 40: {"servo_opu": ["0-0-180", "1-0-90"]},
    }
