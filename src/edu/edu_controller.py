
# Copyright 2016-2022 The FEAGI Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================

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
