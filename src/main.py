# -*- coding: utf-8 -*-
"""
Copyright (c) 2019 Mohammad Nadji-Tehrani <m.nadji.tehrani@gmail.com>

"""

import logging.config
import json
from art import text2art

logging_config_file = './logging_config.json'

with open(logging_config_file, 'r') as data_file:
    LOGGING_CONFIG = json.load(data_file)
    logging.config.dictConfig(LOGGING_CONFIG)


def splash_screen():
    # FEAGI Word Art
    print(text2art("FEAGI", font='block'))


if __name__ == '__main__':
    from inf import initialize
    from evo import neuroembryogenesis, death
    from npu import consciousness
    from life import adventures
    from shutil import copyfile
    import sys
    import os

    splash_screen()

    # Initialize the environment
    feagi_config = initialize.init_parameters()
    # print(feagi_config.get('Logging', 'logging_config_file'))
    initialize.init_data_sources()
    initialize.init_ipu()
    initialize.init_opu()

    exit_condition = False

    # This while loop simulates a single cycle of life for the artificial brain
    while not exit_condition:
        # Process of artificial neuroembryogenesis that leads to connectome development
        neuroembryogenesis.develop_brain()
        # All brain activities occur in between consciousness start and stop
        consciousness.start()
        # A set of experiences will be outlined under life adventures that leads to learning
        adventures.tbd()
        consciousness.stop()
        # Death process eliminates the brain instance and captures associated performance details
        death.register()
