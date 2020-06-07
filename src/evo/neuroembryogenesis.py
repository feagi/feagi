"""
Copyright (c) 2019 Mohammad Nadji-Tehrani <m.nadji.tehrani@gmail.com>

This module is responsible for reading instructions from genome as a genotype and translating them into connectome as
a phenotype.

Instructions in this module are biologically inspired and a high level model after the neuroembryogenesis process that
translates genome to a fully developed brain starting from the embryo's neural tube.

In this framework, genome contains a list of all cortical layers and sub-layers for the brain. Each layer or sub-layer
has properties embedded within genome. Such properties are translated using this module to perform neurogenesis that
creates neurons followed by the process of synaptogenesis that is responsible for the creation of the connectivity
among neurons. All of the structures generated as a result of this process is stored in a data structure called
connectome.
"""

import evo.connectome as connectome
import evo.genome as genome
import logging

log = logging.getLogger(__name__)


def develop_brain(reincarnation_mode=False):
    """
    This function operates in two modes. If run with reincarnation mode as false then it will develop a new brain by
    reading instructions from genome and developing it step by step. If run with reincarnation mode as True then the
    last available connectome data will be used to operate as a new brain. Reincarnation set to True will preserve the
    memories and learning from the previous generation while starting a new brain instance.

    Args:
        reincarnation_mode (bool): If true, an existing connectome will be used to reuse an existing brain structure.

    Returns:

    """

    if reincarnation_mode:
        connectome.reuse()
    else:
        genome_instructions = genome.selection()
        connectome.develop(genome_instructions)



