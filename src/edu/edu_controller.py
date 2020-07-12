"""
This module will run as an independent thread and acts as a wrapper to orchestrate the training, testing, etc.

Supervised training is coordinated here
"""
from queue import Queue
from threading import Thread
from inf import runtime_data


def initialize():
    return


class Controller:
    def __init__(self):
        # setup a new thread here
        return

    def trainer_mnist(self):
        return

    def trainer_fashion_mnist(self):
        return

    def tester_mnist(self):
        return

    def tester_fashion_mnist(self):
        return
