# -*- coding: utf-8 -*-
"""
Copyright (c) 2019 Mohammad Nadji-Tehrani <m.nadji.tehrani@gmail.com>

FEAGI is a brain inspired evolutionary framework capable of growing and artificial brain from a
genome and helping it evolve over generations.

This main module is responsible for driving the lifecycle of a single generation of an
artificial brain at a time. To scale up the system to many parallel generations, FEAGI
is intended to run within a container and scale up to many container instances.

todo: redo brain development with no db dependency
todo: build Mongodb / local handling for brain development
todo: test dev
todo: perf optimization: ...
todo: create a process for burst_engine
"""

if __name__ == '__main__':
    import logging.config
    import json
    from art import text2art
    from threading import Thread
    from queue import Queue
    from inf import initialize
    from evo import neuroembryogenesis, death
    from npu import consciousness, burst_engine
    from inf import runtime_data
    from ipu.folder_monitor import push_data_to_ipu, folder_mon
    from life import trainer, evaluator

    logging_config_file = '/Users/mohammadnadji-tehrani/code/feagi/feagi/src/logging_config.json'

    with open(logging_config_file, 'r') as data_file:
        LOGGING_CONFIG = json.load(data_file)
        logging.config.dictConfig(LOGGING_CONFIG)

    def splash_screen():
        # FEAGI Word Art
        print(text2art("FEAGI", font='block'))

    splash_screen()

    exit_condition = False

    # This while loop simulates a single cycle of life for the artificial brain
    while not runtime_data.exit_condition:

        # Initialize the environment
        initialize.initialize()

        # Process of artificial neuroembryogenesis that leads to connectome development
        neuroembryogenesis.develop_brain(reincarnation_mode=
                                         runtime_data.parameters['Brain_Development']['reincarnation_mode'])

        # create queue
        runtime_data.watchdog_queue = Queue()

        # Set up a worker thread to process IPU folder reads
        ipu_folder_handler = Thread(target=push_data_to_ipu,
                                    args=(runtime_data.watchdog_queue,), name="ipu_folder_handler", daemon=True)
        ipu_folder_handler.start()

        ipu_thread = Thread(target=folder_mon,
                            args=(runtime_data.working_directory + '/ipu', ['*.png', '*.txt'],
                                  runtime_data.watchdog_queue, ),
                            name='IPU_folder_monitor', daemon=True)
        ipu_thread.start()

        # Staring the burst_manager engine
        burst_engine.burst_manager()

        # All brain activities occur in between consciousness start and stop
        # todo: define what consciousness would mean in the context of this framework!!
        consciousness.start()

        # A set of experiences will be outlined under life adventures that leads to learning
        # adventures.tbd()

        consciousness.stop()

        # closing the threads
        ipu_thread.join()

        # Death process eliminates the brain instance and captures associated performance details
        death.register()
        runtime_data.exit_condition = True

    print('FEAGI instance has been terminated!')
