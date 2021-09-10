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
    10: {"ir_ipu": ["0-0-0", "1-1-1"],
        "proximity_ipu": ["0-0-0", "0-0-3", "0-0-10", "0-0-20"],
        "led_opu": ["5-0-1", "7-0-0"]},
    2: {},
    3: {},
    4: {"led_opu": ["5-0-1"]}
    }
