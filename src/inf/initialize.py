
from inf import runtime_data, disk_ops
from configparser import ConfigParser
import logging

log = logging.getLogger(__name__)


def init_parameters():
    """To load all the key configuration parameters"""
    feagi_config = ConfigParser()
    feagi_config.read('./feagi_configuration.ini')
    runtime_data.parameters = {s: dict(feagi_config.items(s)) for s in feagi_config.sections()}
    print(type(runtime_data.parameters))
    print(runtime_data.parameters)
    log.info("All parameters have been initialized.")


def init_data_sources():
    """To validate and initialize all data sources and databases"""
    log.info("All data sources have been initialized.")
    return


def init_ipu():
    """To validate and initialize all the Input Processing Units"""
    log.info("All IPUs have been initialized.")
    return


def init_opu():
    """To validate and initialize all the Output Processing Units"""
    log.info("All OPUs have been initialized.")
    return


