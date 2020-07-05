"""
Source: https://camcairns.github.io/python/2017/09/06/python_watchdog_jobs_queue.html

This class inherits from the Watchdog PatternMatchingEventHandler class. In this code our watchdog will only be
triggered if a file is moved to have a .trigger extension. Once triggered the watchdog places the event object on the
queue, ready to be picked up by the worker thread
"""

import queue
import string
import time
from watchdog.observers import Observer
from watchdog.events import PatternMatchingEventHandler
from inf import runtime_data


def folder_mon(folder_path, pattern, q):
    event_handler = FolderWatchdog(queue_=q, patterns=pattern)
    observer = Observer()
    observer.schedule(event_handler, folder_path, recursive=True)
    observer.start()
    while not runtime_data.exit_condition:
        time.sleep(2)


class FolderWatchdog(PatternMatchingEventHandler):
    """ Watches a nominated directory and when a trigger file is
        moved to take the ".trigger" extension it proceeds to execute
        the commands within that trigger file.
    """

    def __init__(self, queue_, patterns):
        PatternMatchingEventHandler.__init__(self, patterns=patterns)
        self.queue = queue_

    def process(self, event):
        """
        event.event_type
            'modified' | 'created' | 'moved' | 'deleted'
        event.is_directory
            True | False
        event.src_path
            path/to/observed/file
        """
        self.queue.put(event)

    # def on_moved(self, event):
    #     self.process(event)

    def on_created(self, event):
        push_data_to_ipu(event.src_path)


# todo: most likely this function needs to run on its own thread and not block other operations...maybe!
def push_data_to_ipu(file_path):
    if str(file_path).endswith('.png'):
        pass
    if str(file_path).endswith('.txt'):
        with open(file_path, 'r') as txt_file:
            for _ in txt_file.read():
                print(_)
