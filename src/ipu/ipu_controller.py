"""
This module manages all IPU related modules
"""
from queue import Queue
from threading import Thread
from inf import runtime_data
from ipu.folder_monitor import folder_mon, push_data_to_ipu


def initialize():
    """
    This function will monitor the IPU folder and other possible input devices such as a camera or mic for data and
    pass them along to the IPU module to have them converted to neuronal activity that in turn can be passed to cortical
    areas via FCL injection.

    """

    # log.info("All IPUs have been initialized.")
    # pattern = ["*.png"]
    # event_handler = FileLoaderWatchdog(runtime_data.watchdog_queue, patterns=pattern)
    # observer = Observer()
    # path = runtime_data.working_directory + '/ipu'
    # observer.schedule(event_handler, path, recursive=True)
    # observer.start()

    # create queue used for IPU folder monitor
    runtime_data.watchdog_queue = Queue()

    ipu_folder_handler = Thread(target=push_data_to_ipu,
                                args=(runtime_data.watchdog_queue,), name="ipu_folder_handler", daemon=True)
    ipu_folder_handler.start()

    ipu_thread = Thread(target=folder_mon,
                        args=(runtime_data.working_directory + '/ipu', ['*.png', '*.txt'],
                              runtime_data.watchdog_queue,),
                        name='IPU_folder_monitor', daemon=True)
    ipu_thread.start()
