
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
Provides a series of methods to validate genome syntax

Supporting Genome Versions: 2.0
"""

from evo.genome_properties import *
from inf import settings


def cortical_list_gen(blueprint):
    cortical_list_ = set()
    for gene in blueprint:
        cortical_area = gene.split(genome_properties["structure"]["segment_seperator"])[1]
        if cortical_area not in cortical_list_:
            cortical_list_.add(cortical_area)
    return cortical_list_


def blueprint_validator(genome):
    blueprint = genome["blueprint"]
    neuron_morphologies = genome["neuron_morphologies"]
    cortical_list = cortical_list_gen(blueprint)

    valid_genome = True

    def gene_segments(gene_):
        guide = \
            genome_properties["structure"]["segment_guide"].split(genome_properties["structure"]["segment_seperator"])
        segments = gene_.split(genome_properties["structure"]["segment_seperator"])
        if len(segments) != genome_properties["structure"]["segment_count"]:
            print(gene_)
            print("\t\t\tError: Gene structure should only have %i segments seperated by a %s"
                  % (genome_properties["structure"]["segment_count"],
                     genome_properties["structure"]["segment_seperator"]))
            return False

        for index in range(len(guide)):
            if len(guide[index]) != len(segments[index]):
                print(gene_)
                print("\t\t\tError: Incorrect segment size for ", segments[index])
                return False

        return True

    def destination_rules(gene_):
        segments = gene_.split(genome_properties["structure"]["segment_seperator"])
        if segments[3] == "dstmap":
            for destination in blueprint[gene_]:
                # Check for a valid destination cortical area reference
                if destination not in cortical_list:
                    print(gene_)
                    print("\t\t\tError: Destination cortical area associated with neuron morphology not found:",
                          destination)
                    return False

                # Check for a valid destination rule usage
                for rule in blueprint[gene_][destination]:
                    if rule[0] not in neuron_morphologies:
                        print(gene_)
                        print("\t\t\tError: Referenced neuron morphology is not defined!\n\t\t\t Rule:", rule)
                        return False
        return True

    for gene in blueprint:
        if not gene_segments(gene):
            valid_genome = False
            print(gene)
        if not destination_rules(gene):
            print(gene)
            valid_genome = False

    if valid_genome:
        print("\n" + settings.Bcolors.OKGREEN + "* -- * " * 20 + settings.Bcolors.ENDC)
        print("\n" + settings.Bcolors.OKGREEN + "* -- * " * 20 + settings.Bcolors.ENDC)
        print("\n\t\t\t\t\t\t\t\tGenome validation completed successfully!!")
        print("\n" + settings.Bcolors.OKGREEN + "* -- * " * 20 + settings.Bcolors.ENDC)
        print("\n" + settings.Bcolors.OKGREEN + "* -- * " * 20 + settings.Bcolors.ENDC)
        return True
    else:
        print("\n" + settings.Bcolors.RED + "! ! " * 30 + settings.Bcolors.ENDC)
        print("\n" + settings.Bcolors.RED + "! ! " * 30 + settings.Bcolors.ENDC)
        print("\n\t\t\t\t\t\t\t\tErrors detected during genome validation!!")
        print("\n" + settings.Bcolors.RED + "! ! " * 30 + settings.Bcolors.ENDC)
        print("\n" + settings.Bcolors.RED + "! ! " * 30 + settings.Bcolors.ENDC)
        return False


