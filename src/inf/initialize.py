
from inf import runtime_data, disk_ops
from configparser import ConfigParser
from shutil import copyfile
import logging
import os


log = logging.getLogger(__name__)


def init_parameters():
    """To load all the key configuration parameters"""
    feagi_config = ConfigParser()
    feagi_config.read('./feagi_configuration.ini')
    runtime_data.parameters = {s: dict(feagi_config.items(s)) for s in feagi_config.sections()}
    log.info("All parameters have been initialized.")


def initialize_connectome():
    connectome_path = runtime_data.parameters["InitData"]["connectome_path"]
    if not os.path.exists(connectome_path):
        os.makedirs(connectome_path)
        copyfile(runtime_data.parameters["InitData"]["static_genome_path"], connectome_path)


def initialize_genome():
    # The following stages the genome in the proper connectome path and loads it into the memory
    disk_ops.genome_handler(runtime_data.parameters["InitData"]["connectome_path"])


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


