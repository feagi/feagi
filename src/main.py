# -*- coding: utf-8 -*-
"""
Copyright (c) 2019 Mohammad Nadji-Tehrani <m.nadji.tehrani@gmail.com>

FEAGI is a brain inspired evolutionary framework capable of growing and artificial brain from a
genome and helping it evolve over generations.

This main module is responsible for driving the lifecycle of a single generation of an
artificial brain at a time. To scale up the system to many parallel generations, FEAGI
is intended to run within a container and scale up to many container instances.

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
    initialize.init_data_sources()
    initialize.init_ipu()
    initialize.init_opu()

    exit_condition = False

    # This while loop simulates a single cycle of life for the artificial brain
    while not exit_condition:
        # Process of artificial neuroembryogenesis that leads to connectome development
        neuroembryogenesis.develop_brain(reincarnation_mode=feagi_config.get('Brain_Development', 'reincarnation_mode'))
        # All brain activities occur in between consciousness start and stop
        consciousness.start()
        # A set of experiences will be outlined under life adventures that leads to learning
        adventures.tbd()
        consciousness.stop()
        # Death process eliminates the brain instance and captures associated performance details
        death.register()

    print('FEAGI instance has been terminated!')

# todo: redo brain development with no db dependency
# todo: build Mongodb / local handling for brain development
# todo: test dev
