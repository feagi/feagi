""""""

import logging


def setup_custom_logger(name, log_level):
    logger = logging.getLogger(__name__)
    logger.setLevel(logging.DEBUG)

    formatter = logging.Formatter('%(asctime)s:%(levelname)s:%(name)s:%(funcName)s:%(lineno)d:%(message)s')

    file_handler = logging.FileHandler('sample.log')
    file_handler.setLevel(logging.ERROR)
    file_handler.setFormatter(formatter)

    logger.addHandler(file_handler)
    return logger

