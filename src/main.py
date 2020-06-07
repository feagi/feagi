# -*- coding: utf-8 -*-
"""
Copyright (c) 2019 Mohammad Nadji-Tehrani <m.nadji.tehrani@gmail.com>

"""

from configparser import ConfigParser
import logging
import logging.config
import json

parser = ConfigParser()
parser.read('../configuration.ini')

with open('./inf/logging_config.json', 'r') as data_file:
    LOGGING_CONFIG = json.load(data_file)

logging.config.dictConfig(LOGGING_CONFIG)

if __name__ == '__main__':
    from configparser import ConfigParser
    from logging import Logger
    from art import text2art
    from shutil import copyfile
    import sys
    import os
    from inf import initialize

    # FEAGI Word Art
    print(text2art("FEAGI", font='block'))


    initialize.init_parameters()
    initialize.init_dataSources()
    initialize.init_IPUs()
    initialize.init_OPUs()










