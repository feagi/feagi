"""
Copyright (c) 2019 Mohammad Nadji-Tehrani <m.nadji.tehrani@gmail.com>

This module handles connectome related operations.

"""

import logging
from inf import runtime_data


log = logging.getLogger(__name__)


class ConnectomeOps:
    def __init__(self):
        return

    # Resets the in-memory brain for each cortical area
    def reset_brain():
        for item in runtime_data.cortical_list:
            runtime_data.brain[item] = {}

    @staticmethod
    def reuse():
        """
        Placeholder for a function to reuse an existing connectome.

        Returns:

        """
        log.info("Reusing an old connectome")
        connectome_path = ''
        return

    @staticmethod
    def develop():
        """
        Reads instructions from genome and develops the connectome based on instructions.
        Args:
            genome_instructions:

        Returns:

        """
        log.info("Brain development has begone")

        return

    @staticmethod
    def connectome_backup(src, dst):
        """
        Backs up an existing connectome to preserve all the structures and associated memory data for future use.
        Args:
            src (String): Location of the source connectome folder
            dst (String): Destination folder for storing connectome backup

        Returns:

        """
        import shutil
        import errno
        try:
            shutil.copytree(src, dst)
        except OSError as exc:
            if exc.errno == errno.ENOTDIR:
                shutil.copy(src, dst)
            else:
                raise


class ConnectomeInsights:
    def __init__(self):
        return

    def neuron_count(self):
        return

    @staticmethod
    def synapse_count(cortical_area_src, cortical_area_dst):
        brain = runtime_data.brain
        synapse__count = 0
        for neuron in brain[cortical_area_src]:
            for synapse in brain[cortical_area_src][neuron]['neighbors']:
                if brain[cortical_area_src][neuron]['neighbors'][synapse]['cortical_area'] == cortical_area_dst:
                    synapse__count += 1
        return synapse__count

    @staticmethod
    def connectome_structural_fitness():
        """
        To conduct a set of validations and calculate a structural fitness for the developed connectome. The returned value
        can be used to eliminate a premature structure.

        Returns: Structural fitness factor

        """
        #     vision_v2_it_synapse_cnt = synapse_count('vision_v2', 'vision_IT')
        #     vision_it_mem_synapse_cnt = synapse_count('vision_IT', 'vision_memory')
        #
        #     print("Synapse count vision_v2 >> vision_IT == ", vision_v2_it_synapse_cnt)
        #     print("Synapse count vision_IT >> vision_memory == ", vision_it_mem_synapse_cnt)
        #
        #     if vision_v2_it_synapse_cnt < 50 or vision_it_mem_synapse_cnt < 50:
        #         fitness = 0
        #     else:
        #         fitness = 1
        #     return fitness
        return
