
import logging

log = logging.getLogger(__name__)


def init_parameters():
    """To load all the key configuration parameters"""
    from configparser import ConfigParser
    feagi_config = ConfigParser()
    feagi_config.read('./feagi_configuration.ini')
    log.info("All parameters have been initialized.")
    return feagi_config


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


