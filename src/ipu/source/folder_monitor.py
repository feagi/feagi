"""
Source: https://camcairns.github.io/python/2017/09/06/python_watchdog_jobs_queue.html

This class inherits from the Watchdog PatternMatchingEventHandler class. In this code our watchdog will only be
triggered if a file is moved to have a .trigger extension. Once triggered the watchdog places the event object on the
queue, ready to be picked up by the worker thread
"""

import string
import time
from queue import Queue
from threading import Thread
from watchdog.observers import Observer
from watchdog.events import PatternMatchingEventHandler
from inf import runtime_data
from ipu.processor import utf


# todo: combine all of this module into a single class
def initialize():
    runtime_data.watchdog_queue = Queue()
    ipu_folder_monitor = Thread(target=folder_mon, args=(runtime_data.working_directory + '/ipu', ['*.png', '*.txt'],
                                                         runtime_data.watchdog_queue,), name='IPU_folder_monitor',
                                daemon=True)
    ipu_folder_monitor.start()
    print(">> >> >> Folder monitoring thread has started..")


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

    def on_moved(self, event):
        print("IPU detected a modification to ", event.src_path)
        self.process(event)

    def on_created(self, event):
        file_path = event.src_path
        if str(file_path).endswith('.png'):
            print('IPU detected the presence of a new --PNG-- file @', file_path)
            pass
        if str(file_path).endswith('.txt'):
            print('Detected the presence of a new --TXT-- file @', file_path)
            with open(file_path, 'r') as txt_file:
                for _ in txt_file.read():
                    print(_)
                    runtime_data.fire_candidate_list['utf8'].update(utf.convert_char_to_fire_list(_))
