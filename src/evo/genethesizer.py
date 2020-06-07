# Copyright (c) 2019 Mohammad Nadji-Tehrani <m.nadji.tehrani@gmail.com>
""" This module contains functions capable of modifying and shaping the Genome"""

import datetime
import random
import string
from misc import db_handler
from math import floor
from configuration import settings


print(settings.Bcolors.YELLOW + "Module loaded: genethesizer" + settings.Bcolors.ENDC)


def gene_anomoly_detector():
    verdict = False

    #todo: implement the logic for anomoly detection here.

    return verdict


def select_a_genome():
    """
    This function randomly selects a genetic operation from following options to produce a genome:
    1. Crossover two randomly selected genomes
    2. Random selection from available genomes
    3. Latest genome
    4. Best fitness ever
    5. Mutate genome with highest fitness
    6. TBD
    """
    random_selector = random.randrange(1, 10, 1)
    # random_selector = 4

    if random_selector == 1:
        print("Crossover is happening...")
        genome, original_genome_id = crossover()

    elif random_selector == 2:
        print("A random genome is being selected...")
        # genome = random_genome()
        genome, original_genome_id = highest_fitness_genome()

    elif random_selector == 3:
        print("Most recent genome is being selected...")
        genome, original_genome_id = latest_genome()
        # genome, original_genome_id = highest_fitness_genome()

    elif random_selector == 4:
        print("The genome with highest fitness so far has been selected...")
        genome, original_genome_id = highest_fitness_genome()

    elif random_selector >= 5:
        print("Gene mutation has occurred...")
        candidate_genome, original_genome_id = highest_fitness_genome()
        genome = mutate(candidate_genome)

    # elif random_selector == 6:
    #     genome =
    print(">> >> >> >> The genome id used for brain generation is: ", original_genome_id)
    return genome, original_genome_id



# class GeneModifier2:


class GeneModifier3:
    def __init__(self):
        pass

    def random_gene_selector(self, genome_segment):
        while type(genome_segment) == dict:
            random_key = random.choice(list(genome_segment))
            previous_segment = genome_segment
            genome_segment = genome_segment[random_key]
        # todo: need to return the full path!
        return random_key, previous_segment[random_key]

    def mutate(self, genome):
        # 1. index all the genome leaves
        # 2. modify the value of that selected leave
        # 3. return: new genome, what gene changed, initial value, new value

        mutation_keys = ('blueprint', 'location_tolerance', 'image_color_intensity_tolerance')

        return mutated_genome, mutated_gene, old_value, new_value


class GeneModifier:
    @staticmethod
    def change_cortical_neuron_count(genome, cortical_area, change_percentage):
        """ Function to increase or decrease the neuron density aka Cortical Neuron Count in a given cortical area"""
        genome['blueprint'][cortical_area]['cortical_neuron_count'] += \
            floor(genome['blueprint'][cortical_area]['cortical_neuron_count'] * change_percentage)
        if genome['blueprint'][cortical_area]['cortical_neuron_count'] < 0:
            genome['blueprint'][cortical_area]['cortical_neuron_count'] = 0
        return genome

    @staticmethod
    def change_cortical_dimensions(genome, cortical_area, change_percentage):
        """ Function to increase or decrease the size of a cortical area's dimension"""
        genome['blueprint'][cortical_area]['neuron_params']['geometric_boundaries']["x"][1] += \
            genome['blueprint'][cortical_area]['neuron_params']['geometric_boundaries']["x"][1] * change_percentage
        if genome['blueprint'][cortical_area]['neuron_params']['geometric_boundaries']["x"][1] < 10:
            genome['blueprint'][cortical_area]['neuron_params']['geometric_boundaries']["x"][1] = 10
        genome['blueprint'][cortical_area]['neuron_params']['geometric_boundaries']["x"][1] = \
            floor(genome['blueprint'][cortical_area]['neuron_params']['geometric_boundaries']["x"][1])

        genome['blueprint'][cortical_area]['neuron_params']['geometric_boundaries']["y"][1] += \
            genome['blueprint'][cortical_area]['neuron_params']['geometric_boundaries']["y"][1] * change_percentage
        if genome['blueprint'][cortical_area]['neuron_params']['geometric_boundaries']["y"][1] < 10:
            genome['blueprint'][cortical_area]['neuron_params']['geometric_boundaries']["y"][1] = 10
        genome['blueprint'][cortical_area]['neuron_params']['geometric_boundaries']["y"][1] = \
            floor(genome['blueprint'][cortical_area]['neuron_params']['geometric_boundaries']["y"][1])

        genome['blueprint'][cortical_area]['neuron_params']['geometric_boundaries']["z"][1] += \
            genome['blueprint'][cortical_area]['neuron_params']['geometric_boundaries']["z"][1] * change_percentage
        if genome['blueprint'][cortical_area]['neuron_params']['geometric_boundaries']["z"][1] < 10:
            genome['blueprint'][cortical_area]['neuron_params']['geometric_boundaries']["z"][1] = 10
        genome['blueprint'][cortical_area]['neuron_params']['geometric_boundaries']["z"][1] = \
            floor(genome['blueprint'][cortical_area]['neuron_params']['geometric_boundaries']["z"][1])
        return genome


    @staticmethod
    def change_firing_threshold(genome, cortical_area, change_percentage):
        """ Function to increase or decrease the neuron firing threshold in a given cortical area"""
        genome['blueprint'][cortical_area]['neuron_params']['firing_threshold'] += \
            genome['blueprint'][cortical_area]['neuron_params']['firing_threshold'] * change_percentage
        if genome['blueprint'][cortical_area]['neuron_params']['firing_threshold'] < 0:
            genome['blueprint'][cortical_area]['neuron_params']['firing_threshold'] = 0
        return genome

    @staticmethod
    def change_consecutive_fire_cnt_max(genome, cortical_area, change_percentage):
        """ Function to increase or decrease the neuron firing threshold in a given cortical area"""
        genome['blueprint'][cortical_area]['neuron_params']['consecutive_fire_cnt_max'] += \
            genome['blueprint'][cortical_area]['neuron_params']['consecutive_fire_cnt_max'] * change_percentage
        if genome['blueprint'][cortical_area]['neuron_params']['consecutive_fire_cnt_max'] <= 1:
            genome['blueprint'][cortical_area]['neuron_params']['consecutive_fire_cnt_max'] = 1
        genome['blueprint'][cortical_area]['neuron_params']['consecutive_fire_cnt_max'] = \
            floor(genome['blueprint'][cortical_area]['neuron_params']['consecutive_fire_cnt_max'])
        return genome

    @staticmethod
    def change_depolarization_timer_threshold(genome, cortical_area, change_percentage):
        """ Function to increase or decrease the neuron timer threshold in a given cortical area"""
        genome['blueprint'][cortical_area]['neuron_params']['depolarization_threshold'] += \
            genome['blueprint'][cortical_area]['neuron_params']['depolarization_threshold'] * change_percentage
        if genome['blueprint'][cortical_area]['neuron_params']['depolarization_threshold'] < 0:
            genome['blueprint'][cortical_area]['neuron_params']['depolarization_threshold'] = 0
        return genome

    @staticmethod
    def change_consecutive_fire_cnt_max(genome, cortical_area, change_percentage):
        """ Function to increase or decrease the neuron consecutive_fire_cnt_max in a given cortical area"""
        genome['blueprint'][cortical_area]['neuron_params']['consecutive_fire_cnt_max'] += \
            floor(genome['blueprint'][cortical_area]['neuron_params']['consecutive_fire_cnt_max'] * change_percentage)
        if genome['blueprint'][cortical_area]['neuron_params']['consecutive_fire_cnt_max'] < 0:
            genome['blueprint'][cortical_area]['neuron_params']['consecutive_fire_cnt_max'] = 0
        return genome

    @staticmethod
    def change_snooze_length(genome, cortical_area, change_percentage):
        """ Function to increase or decrease the neuron snooze_length in a given cortical area"""
        genome['blueprint'][cortical_area]['neuron_params']['snooze_length'] += \
            genome['blueprint'][cortical_area]['neuron_params']['snooze_length'] * change_percentage
        if genome['blueprint'][cortical_area]['neuron_params']['snooze_length'] < 0:
            genome['blueprint'][cortical_area]['neuron_params']['snooze_length'] = 0
        return genome

    @staticmethod
    def change_vision_plasticity_constant(genome, change_percentage):
        """ Function to increase or decrease the neuron snooze_length in a given cortical area"""
        genome['blueprint']['vision_memory']['plasticity_constant'] += \
            genome['blueprint']['vision_memory']['plasticity_constant'] * change_percentage
        if genome['blueprint']['vision_memory']['plasticity_constant'] < 0:
            genome['blueprint']['vision_memory']['plasticity_constant'] = 0
        return genome


    @staticmethod
    def change_growth_rule_4_param_2(genome, change_percentage):
        """ Function to increase or decrease the neuron snooze_length in a given cortical area"""
        genome['neighbor_locator_rule']['rule_4']['param_2'] += \
            genome['neighbor_locator_rule']['rule_4']['param_2'] * change_percentage
        if genome['neighbor_locator_rule']['rule_4']['param_2'] < 5:
            genome['neighbor_locator_rule']['rule_4']['param_2'] = 5
        genome['neighbor_locator_rule']['rule_4']['param_2'] = \
            floor(genome['neighbor_locator_rule']['rule_4']['param_2'])
        return genome


def genome_id_gen(size=6, chars=string.ascii_uppercase + string.digits):
    """
    This function generates a unique id which will be associated with each GEnome

    """
    # Rand gen source partially from:
    # http://stackoverflow.com/questions/2257441/random-string-generation-with-upper-case-letters-and-digits-in-python
    return (str(datetime.datetime.now()).replace(' ', '_')).replace('.', '_')+'_'+(''.join(random.choice(chars) for _ in range(size)))+'_G'


def generation_assessment():
    """ A collection of assessments to evaluate the performance of the Genome"""
    return


def genethesize():
    """ Responsible for generating a complete set of Genome"""

    genome = {}
    genome["firing_patterns"] = {
      "A": {
         "frequency": "100",
         "magnitude": "80"
      },
      "B": {
         "frequency": "20",
         "magnitude": "100"
      }
   }
    genome["neighbor_locator_rule"] = {
      "rule_0": {
         "param_1": 5,
         "param_2": 0
      },
      "rule_1": {
         "param_1": 5,
         "param_2": 5
      },
      "rule_2": {
         "param_1": 5,
         "param_2": 5,
         "param_3": 10
      },
      "rule_3": {
         "param_1": 0,
         "param_2": 0
      }
   }
    return genome


def mutate(genome):
    # todo: refactor this function to use parameters/genome to drive

    dice = random.randrange(0,8)
    selector = [0,0,0,0,0,0,0,0]
    selector[dice] = 1

    factor_1 = selector[0] * random.randrange(-20, 20, 1) / 100
    factor_2 = selector[1] * random.randrange(-20, 20, 1) / 100
    factor_3 = selector[2] * random.randrange(-20, 20, 1) / 100
    factor_4 = selector[3] * random.randrange(-20, 20, 1) / 100
    factor_5 = selector[4] * random.randrange(-20, 20, 1) / 100
    factor_6 = selector[5] * random.randrange(-20, 20, 1) / 100
    factor_7 = selector[6] * random.randrange(-20, 20, 1) / 100
    factor_8 = selector[7] * random.randrange(-20, 20, 1) / 100

    blueprint = genome["blueprint"]
    cortical_list = []
    for key in blueprint:
        # Condition to black-list select regions from mutation such as UTF
        if genome["blueprint"][key]["group_id"] == 'vision' or \
                (genome["blueprint"][key]["group_id"] == 'Memory' and
                 genome["blueprint"][key]["sub_group_id"] == 'vision'):
            cortical_list.append(key)

    print("#@#@#@# $$$ Mutation is about to take place on the following cortical regions:\n", cortical_list)

    for cortical_area in cortical_list:
        # Anatomical changes
        genome = GeneModifier.change_cortical_dimensions(genome, cortical_area, factor_6)
        genome = GeneModifier.change_cortical_neuron_count(genome, cortical_area, factor_2)
        genome = GeneModifier.change_growth_rule_4_param_2(genome, factor_7)
        # Physiological changes
        genome = GeneModifier.change_consecutive_fire_cnt_max(genome, cortical_area, factor_1)
        genome = GeneModifier.change_depolarization_timer_threshold(genome, cortical_area, factor_3)
        genome = GeneModifier.change_firing_threshold(genome, cortical_area, factor_4)
        genome = GeneModifier.change_snooze_length(genome, cortical_area, factor_5)
        genome = GeneModifier.change_vision_plasticity_constant(genome, factor_8)
    return genome


def get_genome_candidate():
    """ Scans genome db for a high performing genome and returns one from top 10% by random """
    genome = db_handler.MongoManagement.top_n_genome(1)
    return genome


def crossover():
    """
    To corssover genome 1 and 2, first list of keys from one genome is read and the content of that key
    is swapped with the other genome.

    todo: Given genome is hierarchical, crossover need to account for different levels

    """
    db = db_handler.MongoManagement()

    genome_1, genome_2 = db.id_list_2_genome_list(db.random_m_from_top_n(2, 5))

    original_genome_id = []
    original_genome_id.append(genome_1['genome_id'])
    original_genome_id.append(genome_2['genome_id'])

    genome_1 = genome_1["properties"]
    genome_2 = genome_2["properties"]

    genome_1_keys = []
    for key in genome_1["blueprint"].keys():
        genome_1_keys.append(key)

    # Select a random key
    random_key = genome_1_keys[random.randrange(len(genome_1_keys))]

    print("Crossing over: ", random_key)

    # Cross over
    genome_2["blueprint"][random_key] = genome_1["blueprint"][random_key]

    print("--- Gene crossover has occurred ---")

    return genome_2, original_genome_id


def random_genome():
    db = db_handler.MongoManagement()
    genomes = db.id_list_2_genome_list(db.random_m_from_top_n(1, 5))
    for item in genomes:
        genome = item
    # print("this is the random genome", genome)
    original_genome_id = []
    original_genome_id.append(genome['genome_id'])
    return genome['properties'], original_genome_id


def latest_genome():
    db = db_handler.MongoManagement()
    genome = db.latest_genome()
    for key in genome:
        print(">.> ", key)
    original_genome_id = []
    try:
        original_genome_id.append(genome['genome_id'])
    except KeyError:
        print("\n\n\nERROR: KeyError while appending genome_id to original_genome_id\n\n\n")
    return genome['properties'], original_genome_id


def highest_fitness_genome():
    db = db_handler.MongoManagement()
    genome = db.highest_fitness_genome()
    original_genome_id = []
    original_genome_id.append(genome['genome_id'])
    return genome['properties'], original_genome_id


def translate_genotype2phenotype():
    return


def calculate_brain_cognitive_fitness(test_stats):
    """
    Calculate the effectiveness of a given genome:
    1. Fitness value will be a number between 0 and 100 with 100 the highest fitness possible (how can there be limit?)
    2. Brain activeness should be considered as a factor. Brain that only guessed one number and that one correctly
          should not be considered better than a brain that guessed 100s of numbers 95% correct.
    3. Number of guess attempts vs. number of correct guesses is a factor

    Fitness calculation formula:

    TE = Total number of times brain has been exposed to a character
    TC = Total number of times brain has comprehended a number correctly
    AF = Activity factor that is measured by a threshold.
        - < 10 exposures leads to 0
        - > 10 & < 50 exposures leads to n / 50 factor
        - > 50 exposures leads to 1

    TC / TE = Percentage of correct comprehensions

    Genome fitness factor = AF * TC / TE

    todo: Investigate cases where looking at one number stimulates multiple numbers together

    """
    total_exposure, total_comprehended = genome_stats_analytics(test_stats)

    if total_exposure < 10:
        activity_factor = 0
    elif 10 <= total_exposure <= 50:
        activity_factor = total_exposure / 50
    else:
        activity_factor = 1

    if total_exposure == 0:
        fitness = 0
    else:
        fitness = total_comprehended / total_exposure
        # fitness = activity_factor * total_comprehended / total_exposure

    return fitness


def genome_stats_analytics(test_stats):
    print("Test stats:", test_stats)
    exposure_total = 0
    comprehended_total = 0
    for _ in range(10):
        exposure_total += test_stats[0][_]['exposed']
        comprehended_total += test_stats[0][_]['comprehended']

    return exposure_total, comprehended_total


def calculate_survival_prob():
    return


def compare_genomes():
    return


def synthesize_new_gen():
    return


def selection():
    return


def spin_new_generation():
    return

#
if __name__ == "__main__":
    gene_modifier = GeneModifier3()

    sample_genome = {
  "firing_patterns" : {
      "A" : {
          "frequency" : "100",
          "magnitude" : "80"
      },
      "B" : {
          "frequency" : "20",
          "magnitude" : "100"
      }
  },
  "neighbor_locator_rule" : {
      "rule_0" : {
          "param_1" : 0,
          "param_2" : 0
      },
      "rule_1" : {
          "param_1" : 1,
          "param_2" : 1
      },
      "rule_2" : {
          "param_1" : 5,
          "param_2" : 5,
          "param_3" : 10
      },
      "rule_3" : {
          "param_1" : 0,
          "param_2" : 0
      },
      "rule_4" : {
          "param_1" : 25,
          "param_2" : 77
      },
      "rule_5" : {
          "param_1" : 700,
          "param_2" : 700,
          "param_3" : 700,
          "param_4" : 700,
          "param_5" : 700,
          "param_6" : 700,
          "param_7" : 700
      },
      "rule_6" : {
          "param_1" : 1,
          "param_2" : 1
      }
  },
  "IPU_vision_filters" : {
      "3" : {
          "-" : [
              [
                  -1,
                  -1,
                  -1
              ],
              [
                  1,
                  1,
                  1
              ],
              [
                  -1,
                  -1,
                  -1
              ]
          ],
          "|" : [
              [
                  -1,
                  1,
                  -1
              ],
              [
                  -1,
                  1,
                  -1
              ],
              [
                  -1,
                  1,
                  -1
              ]
          ],
          " " : [
              [
                  0,
                  0,
                  0
              ],
              [
                  0,
                  0,
                  0
              ],
              [
                  0,
                  0,
                  0
              ]
          ],
          "o" : [
              [
                  -1,
                  -1,
                  -1
              ],
              [
                  -1,
                  8,
                  -1
              ],
              [
                  -1,
                  -1,
                  -1
              ]
          ],
          "/" : [
              [
                  -1,
                  -1,
                  1
              ],
              [
                  -1,
                  1,
                  -1
              ],
              [
                  1,
                  -1,
                  -1
              ]
          ],
          "\\" : [
              [
                  1,
                  -1,
                  -1
              ],
              [
                  -1,
                  1,
                  -1
              ],
              [
                  -1,
                  -1,
                  1
              ]
          ]
      },
      "5" : {
          "-" : [
              [
                  -1,
                  -1,
                  -1,
                  -1,
                  -1
              ],
              [
                  -1,
                  -1,
                  -1,
                  -1,
                  -1
              ],
              [
                  1,
                  1,
                  1,
                  1,
                  1
              ],
              [
                  -1,
                  -1,
                  -1,
                  -1,
                  -1
              ],
              [
                  -1,
                  -1,
                  -1,
                  -1,
                  -1
              ]
          ],
          " " : [
              [
                  0,
                  0,
                  0,
                  0,
                  0
              ],
              [
                  0,
                  0,
                  0,
                  0,
                  0
              ],
              [
                  0,
                  0,
                  0,
                  0,
                  0
              ],
              [
                  0,
                  0,
                  0,
                  0,
                  0
              ],
              [
                  0,
                  0,
                  0,
                  0,
                  0
              ]
          ],
          "|" : [
              [
                  -1,
                  -1,
                  1,
                  -1,
                  -1
              ],
              [
                  -1,
                  -1,
                  1,
                  -1,
                  -1
              ],
              [
                  -1,
                  -1,
                  1,
                  -1,
                  -1
              ],
              [
                  -1,
                  -1,
                  1,
                  -1,
                  -1
              ],
              [
                  -1,
                  -1,
                  1,
                  -1,
                  -1
              ]
          ],
          "o" : [
              [
                  -1,
                  -1,
                  -1,
                  -1,
                  -1
              ],
              [
                  -1,
                  -1,
                  -1,
                  -1,
                  -1
              ],
              [
                  -1,
                  -1,
                  -1,
                  -1,
                  -1
              ],
              [
                  -1,
                  -1,
                  -1,
                  -1,
                  -1
              ],
              [
                  -1,
                  -1,
                  -1,
                  -1,
                  -1
              ]
          ],
          "/" : [
              [
                  -1,
                  -1,
                  -1,
                  -1,
                  1
              ],
              [
                  -1,
                  -1,
                  -1,
                  1,
                  -1
              ],
              [
                  -1,
                  -1,
                  1,
                  -1,
                  -1
              ],
              [
                  -1,
                  1,
                  -1,
                  -1,
                  -1
              ],
              [
                  1,
                  -1,
                  -1,
                  -1,
                  -1
              ]
          ],
          "\\" : [
              [
                  1,
                  -1,
                  -1,
                  -1,
                  -1
              ],
              [
                  -1,
                  1,
                  -1,
                  -1,
                  -1
              ],
              [
                  -1,
                  -1,
                  1,
                  -1,
                  -1
              ],
              [
                  -1,
                  -1,
                  -1,
                  1,
                  -1
              ],
              [
                  -1,
                  -1,
                  -1,
                  -1,
                  1
              ]
          ]
      },
      "7" : {
          "-" : [
              [
                  -1,
                  -1,
                  -1,
                  -1,
                  -1,
                  -1,
                  -1
              ],
              [
                  -1,
                  -1,
                  -1,
                  -1,
                  -1,
                  -1,
                  -1
              ],
              [
                  -1,
                  -1,
                  -1,
                  -1,
                  -1,
                  -1,
                  -1
              ],
              [
                  1,
                  1,
                  1,
                  1,
                  1,
                  1,
                  1
              ],
              [
                  -1,
                  -1,
                  -1,
                  -1,
                  -1,
                  -1,
                  -1
              ],
              [
                  -1,
                  -1,
                  -1,
                  -1,
                  -1,
                  -1,
                  -1
              ],
              [
                  -1,
                  -1,
                  -1,
                  -1,
                  -1,
                  -1,
                  -1
              ]
          ],
          " " : [
              [
                  0,
                  0,
                  0,
                  0,
                  0,
                  0,
                  0
              ],
              [
                  0,
                  0,
                  0,
                  0,
                  0,
                  0,
                  0
              ],
              [
                  0,
                  0,
                  0,
                  0,
                  0,
                  0,
                  0
              ],
              [
                  0,
                  0,
                  0,
                  0,
                  0,
                  0,
                  0
              ],
              [
                  0,
                  0,
                  0,
                  0,
                  0,
                  0,
                  0
              ],
              [
                  0,
                  0,
                  0,
                  0,
                  0,
                  0,
                  0
              ],
              [
                  0,
                  0,
                  0,
                  0,
                  0,
                  0,
                  0
              ]
          ],
          "I" : [
              [
                  -1,
                  -1,
                  -1,
                  1,
                  -1,
                  -1,
                  -1
              ],
              [
                  -1,
                  -1,
                  -1,
                  1,
                  -1,
                  -1,
                  -1
              ],
              [
                  -1,
                  -1,
                  -1,
                  1,
                  -1,
                  -1,
                  -1
              ],
              [
                  -1,
                  -1,
                  -1,
                  1,
                  -1,
                  -1,
                  -1
              ],
              [
                  -1,
                  -1,
                  -1,
                  1,
                  -1,
                  -1,
                  -1
              ],
              [
                  -1,
                  -1,
                  -1,
                  1,
                  -1,
                  -1,
                  -1
              ],
              [
                  -1,
                  -1,
                  -1,
                  1,
                  -1,
                  -1,
                  -1
              ]
          ],
          "i" : [
              [
                  -1,
                  -1,
                  -1,
                  -1,
                  -1,
                  -1,
                  -1
              ],
              [
                  -1,
                  -1,
                  -1,
                  1,
                  -1,
                  -1,
                  -1
              ],
              [
                  -1,
                  -1,
                  -1,
                  1,
                  -1,
                  -1,
                  -1
              ],
              [
                  -1,
                  -1,
                  -1,
                  1,
                  -1,
                  -1,
                  -1
              ],
              [
                  -1,
                  -1,
                  -1,
                  1,
                  -1,
                  -1,
                  -1
              ],
              [
                  -1,
                  -1,
                  -1,
                  1,
                  -1,
                  -1,
                  -1
              ],
              [
                  -1,
                  -1,
                  -1,
                  -1,
                  -1,
                  -1,
                  -1
              ]
          ],
          "o" : [
              [
                  -1,
                  -1,
                  -1,
                  -1,
                  -1,
                  -1,
                  -1
              ],
              [
                  -1,
                  -1,
                  -1,
                  -1,
                  -1,
                  -1,
                  -1
              ],
              [
                  -1,
                  -1,
                  -1,
                  -1,
                  -1,
                  -1,
                  -1
              ],
              [
                  -1,
                  -1,
                  -1,
                  -1,
                  -1,
                  -1,
                  -1
              ],
              [
                  -1,
                  -1,
                  -1,
                  -1,
                  -1,
                  -1,
                  -1
              ],
              [
                  -1,
                  -1,
                  -1,
                  -1,
                  -1,
                  -1,
                  -1
              ],
              [
                  -1,
                  -1,
                  -1,
                  -1,
                  -1,
                  -1,
                  -1
              ]
          ],
          "/" : [
              [
                  -1,
                  -1,
                  -1,
                  -1,
                  -1,
                  -1,
                  1
              ],
              [
                  -1,
                  -1,
                  -1,
                  -1,
                  -1,
                  1,
                  -1
              ],
              [
                  -1,
                  -1,
                  -1,
                  -1,
                  1,
                  -1,
                  -1
              ],
              [
                  -1,
                  -1,
                  -1,
                  1,
                  -1,
                  -1,
                  -1
              ],
              [
                  -1,
                  -1,
                  1,
                  -1,
                  -1,
                  -1,
                  -1
              ],
              [
                  -1,
                  1,
                  -1,
                  -1,
                  -1,
                  -1,
                  -1
              ],
              [
                  1,
                  -1,
                  -1,
                  -1,
                  -1,
                  -1,
                  -1
              ]
          ],
          "\\" : [
              [
                  1,
                  -1,
                  -1,
                  -1,
                  -1,
                  -1,
                  -1
              ],
              [
                  -1,
                  1,
                  -1,
                  -1,
                  -1,
                  -1,
                  -1
              ],
              [
                  -1,
                  -1,
                  1,
                  -1,
                  -1,
                  -1,
                  -1
              ],
              [
                  -1,
                  -1,
                  -1,
                  1,
                  -1,
                  -1,
                  -1
              ],
              [
                  -1,
                  -1,
                  -1,
                  -1,
                  1,
                  -1,
                  -1
              ],
              [
                  -1,
                  -1,
                  -1,
                  -1,
                  -1,
                  1,
                  -1
              ],
              [
                  -1,
                  -1,
                  -1,
                  -1,
                  -1,
                  -1,
                  1
              ]
          ]
      }
  },
  "location_tolerance" : 2,
  "image_color_intensity_tolerance" : 100,
  "max_burst_count" : 3,
  "evolution_burst_count" : 50,
  "performance_stats" : {
      "mnist_view_cnt" : 0,
      "mnist_correct_detection_cnt" : 0
  },
  "blueprint" : {
      "vision_v1-1" : {
          "growth_path" : "",
          "direction_sensitivity" : "/",
          "group_id" : "vision",
          "sub_group_id" : "vision_v1",
          "plot_index" : 1,
          "layer_index" : 1,
          "total_layer_count" : 7,
          "orientation_selectivity_pattern" : "",
          "location" : "",
          "kernel_size" : 5,
          "cortical_neuron_count" : 1597,
          "location_generation_type" : "random",
          "synapse_attractivity" : 100,
          "init_synapse_needed" : False,
          "postsynaptic_current" : 20,
          "plasticity_constant" : 0,
          "postsynaptic_current_max" : 1000,
          "neighbor_locator_rule_id" : "rule_1",
          "neighbor_locator_rule_param_id" : "param_1",
          "cortical_mapping_dst" : {
              "vision_v2" : {
                  "neighbor_locator_rule_id" : "rule_5",
                  "neighbor_locator_rule_param_id" : "param_1"
              }
          },
          "neuron_params" : {
              "activation_function_id" : "",
              "depolarization_threshold" : 1.633575495825,
              "orientation_selectivity_id" : "",
              "firing_threshold" : 2.89235403072,
              "firing_pattern_id" : "",
              "refractory_period" : 0,
              "axon_avg_length" : "",
              "leak_coefficient" : 10,
              "axon_avg_connections" : "",
              "axon_orientation function" : "",
              "consecutive_fire_cnt_max" : 0,
              "snooze_length" : 1.659231437448,
              "block_boundaries" : [
                  28,
                  28,
                  1
              ],
              "geometric_boundaries" : {
                  "x" : [
                      0,
                      32
                  ],
                  "y" : [
                      0,
                      32
                  ],
                  "z" : [
                      0,
                      10
                  ]
              }
          }
      },
      "vision_v1-2" : {
          "growth_path" : "",
          "group_id" : "vision",
          "sub_group_id" : "vision_v1",
          "plot_index" : 4,
          "layer_index" : 2,
          "total_layer_count" : 7,
          "direction_sensitivity" : "\\",
          "orientation_selectivity_pattern" : "",
          "location" : "",
          "kernel_size" : 5,
          "cortical_neuron_count" : 1403,
          "location_generation_type" : "random",
          "synapse_attractivity" : 100,
          "init_synapse_needed" : False,
          "postsynaptic_current" : 20,
          "plasticity_constant" : 0,
          "postsynaptic_current_max" : 1000,
          "neighbor_locator_rule_id" : "rule_1",
          "neighbor_locator_rule_param_id" : "param_1",
          "cortical_mapping_dst" : {
              "vision_v2" : {
                  "neighbor_locator_rule_id" : "rule_5",
                  "neighbor_locator_rule_param_id" : "param_2"
              }
          },
          "neuron_params" : {
              "activation_function_id" : "",
              "depolarization_threshold" : 1.574682375,
              "orientation_selectivity_id" : "",
              "firing_threshold" : 3.4898094,
              "firing_pattern_id" : "",
              "refractory_period" : 0,
              "axon_avg_length" : "",
              "leak_coefficient" : 10,
              "axon_avg_connections" : "",
              "axon_orientation function" : "",
              "consecutive_fire_cnt_max" : 0,
              "snooze_length" : 1.83787266,
              "block_boundaries" : [
                  28,
                  28,
                  1
              ],
              "geometric_boundaries" : {
                  "x" : [
                      0,
                      29
                  ],
                  "y" : [
                      0,
                      29
                  ],
                  "z" : [
                      0,
                      10
                  ]
              }
          }
      },
      "vision_v1-3" : {
          "growth_path" : "",
          "group_id" : "vision",
          "sub_group_id" : "vision_v1",
          "plot_index" : 7,
          "layer_index" : 3,
          "total_layer_count" : 7,
          "direction_sensitivity" : "|",
          "orientation_selectivity_pattern" : "",
          "location" : "",
          "kernel_size" : 5,
          "cortical_neuron_count" : 1403,
          "location_generation_type" : "random",
          "synapse_attractivity" : 100,
          "init_synapse_needed" : False,
          "postsynaptic_current" : 20,
          "plasticity_constant" : 0,
          "postsynaptic_current_max" : 1000,
          "neighbor_locator_rule_id" : "rule_1",
          "neighbor_locator_rule_param_id" : "param_1",
          "cortical_mapping_dst" : {
              "vision_v2" : {
                  "neighbor_locator_rule_id" : "rule_5",
                  "neighbor_locator_rule_param_id" : "param_3"
              }
          },
          "neuron_params" : {
              "activation_function_id" : "",
              "depolarization_threshold" : 1.574682375,
              "orientation_selectivity_id" : "",
              "firing_threshold" : 3.4898094,
              "firing_pattern_id" : "",
              "refractory_period" : 0,
              "axon_avg_length" : "",
              "leak_coefficient" : 10,
              "axon_avg_connections" : "",
              "axon_orientation function" : "",
              "consecutive_fire_cnt_max" : 0,
              "snooze_length" : 1.83787266,
              "block_boundaries" : [
                  28,
                  28,
                  1
              ],
              "geometric_boundaries" : {
                  "x" : [
                      0,
                      29
                  ],
                  "y" : [
                      0,
                      29
                  ],
                  "z" : [
                      0,
                      10
                  ]
              }
          }
      },
      "vision_v1-4" : {
          "growth_path" : "",
          "group_id" : "vision",
          "sub_group_id" : "vision_v1",
          "plot_index" : 10,
          "layer_index" : 4,
          "total_layer_count" : 7,
          "direction_sensitivity" : "-",
          "orientation_selectivity_pattern" : "",
          "location" : "",
          "kernel_size" : 5,
          "cortical_neuron_count" : 1403,
          "location_generation_type" : "random",
          "synapse_attractivity" : 100,
          "init_synapse_needed" : False,
          "postsynaptic_current" : 20,
          "plasticity_constant" : 0,
          "postsynaptic_current_max" : 1100,
          "neighbor_locator_rule_id" : "rule_1",
          "neighbor_locator_rule_param_id" : "param_1",
          "cortical_mapping_dst" : {
              "vision_v2" : {
                  "neighbor_locator_rule_id" : "rule_5",
                  "neighbor_locator_rule_param_id" : "param_4"
              }
          },
          "neuron_params" : {
              "activation_function_id" : "",
              "depolarization_threshold" : 1.574682375,
              "orientation_selectivity_id" : "",
              "firing_threshold" : 3.4898094,
              "firing_pattern_id" : "",
              "refractory_period" : 0,
              "axon_avg_length" : "",
              "leak_coefficient" : 10,
              "axon_avg_connections" : "",
              "axon_orientation function" : "",
              "consecutive_fire_cnt_max" : 0,
              "snooze_length" : 1.83787266,
              "block_boundaries" : [
                  28,
                  28,
                  1
              ],
              "geometric_boundaries" : {
                  "x" : [
                      0,
                      29
                  ],
                  "y" : [
                      0,
                      29
                  ],
                  "z" : [
                      0,
                      10
                  ]
              }
          }
      },
      "vision_v1-5" : {
          "growth_path" : "",
          "group_id" : "vision",
          "sub_group_id" : "vision_v1",
          "plot_index" : 13,
          "layer_index" : 5,
          "total_layer_count" : 7,
          "direction_sensitivity" : "I",
          "orientation_selectivity_pattern" : "",
          "location" : "",
          "kernel_size" : 3,
          "cortical_neuron_count" : 1403,
          "location_generation_type" : "random",
          "synapse_attractivity" : 100,
          "init_synapse_needed" : False,
          "postsynaptic_current" : 20,
          "plasticity_constant" : 0,
          "postsynaptic_current_max" : 1000,
          "neighbor_locator_rule_id" : "rule_1",
          "neighbor_locator_rule_param_id" : "param_1",
          "cortical_mapping_dst" : {
              "vision_v2" : {
                  "neighbor_locator_rule_id" : "rule_5",
                  "neighbor_locator_rule_param_id" : "param_5"
              }
          },
          "neuron_params" : {
              "activation_function_id" : "",
              "depolarization_threshold" : 1.574682375,
              "orientation_selectivity_id" : "",
              "firing_threshold" : 3.4898094,
              "firing_pattern_id" : "",
              "refractory_period" : 0,
              "axon_avg_length" : "",
              "leak_coefficient" : 10,
              "axon_avg_connections" : "",
              "axon_orientation function" : "",
              "consecutive_fire_cnt_max" : 0,
              "snooze_length" : 1.83787266,
              "block_boundaries" : [
                  28,
                  28,
                  1
              ],
              "geometric_boundaries" : {
                  "x" : [
                      0,
                      29
                  ],
                  "y" : [
                      0,
                      29
                  ],
                  "z" : [
                      0,
                      10
                  ]
              }
          }
      },
      "vision_v1-6" : {
          "growth_path" : "",
          "group_id" : "vision",
          "sub_group_id" : "vision_v1",
          "plot_index" : 16,
          "layer_index" : 6,
          "total_layer_count" : 7,
          "direction_sensitivity" : "o",
          "orientation_selectivity_pattern" : "",
          "location" : "",
          "kernel_size" : 3,
          "cortical_neuron_count" : 1404,
          "location_generation_type" : "random",
          "synapse_attractivity" : 100,
          "init_synapse_needed" : False,
          "postsynaptic_current" : 20,
          "plasticity_constant" : 0,
          "postsynaptic_current_max" : 1000,
          "neighbor_locator_rule_id" : "rule_1",
          "neighbor_locator_rule_param_id" : "param_1",
          "cortical_mapping_dst" : {
              "vision_v2" : {
                  "neighbor_locator_rule_id" : "rule_5",
                  "neighbor_locator_rule_param_id" : "param_6"
              }
          },
          "neuron_params" : {
              "activation_function_id" : "",
              "depolarization_threshold" : 1.574682375,
              "orientation_selectivity_id" : "",
              "firing_threshold" : 3.4898094,
              "firing_pattern_id" : "",
              "refractory_period" : 0,
              "axon_avg_length" : "",
              "leak_coefficient" : 10,
              "axon_avg_connections" : "",
              "axon_orientation function" : "",
              "consecutive_fire_cnt_max" : 0,
              "snooze_length" : 1.83787266,
              "block_boundaries" : [
                  28,
                  28,
                  1
              ],
              "geometric_boundaries" : {
                  "x" : [
                      0,
                      29
                  ],
                  "y" : [
                      0,
                      29
                  ],
                  "z" : [
                      0,
                      10
                  ]
              }
          }
      },
      "vision_v1-7" : {
          "growth_path" : "",
          "group_id" : "vision",
          "sub_group_id" : "vision_v1",
          "plot_index" : 19,
          "layer_index" : 7,
          "total_layer_count" : 7,
          "direction_sensitivity" : " ",
          "orientation_selectivity_pattern" : "",
          "location" : "",
          "kernel_size" : 5,
          "cortical_neuron_count" : 1403,
          "location_generation_type" : "random",
          "synapse_attractivity" : 100,
          "init_synapse_needed" : False,
          "postsynaptic_current" : 20,
          "plasticity_constant" : 0,
          "postsynaptic_current_max" : 1000,
          "neighbor_locator_rule_id" : "rule_1",
          "neighbor_locator_rule_param_id" : "param_1",
          "cortical_mapping_dst" : {
              "vision_v2" : {
                  "neighbor_locator_rule_id" : "rule_5",
                  "neighbor_locator_rule_param_id" : "param_7"
              }
          },
          "neuron_params" : {
              "activation_function_id" : "",
              "depolarization_threshold" : 1.574682375,
              "orientation_selectivity_id" : "",
              "firing_threshold" : 3.4898094,
              "firing_pattern_id" : "",
              "refractory_period" : 0,
              "axon_avg_length" : "",
              "leak_coefficient" : 10,
              "axon_avg_connections" : "",
              "axon_orientation function" : "",
              "consecutive_fire_cnt_max" : 0,
              "snooze_length" : 1.83787266,
              "block_boundaries" : [
                  28,
                  28,
                  1
              ],
              "geometric_boundaries" : {
                  "x" : [
                      0,
                      29
                  ],
                  "y" : [
                      0,
                      29
                  ],
                  "z" : [
                      0,
                      10
                  ]
              }
          }
      },
      "vision_v2" : {
          "growth_path" : "",
          "group_id" : "vision",
          "sub_group_id" : "vision_v2",
          "plot_index" : 2,
          "orientation_selectivity_pattern" : "",
          "location" : "",
          "kernel_size" : 3,
          "cortical_neuron_count" : 14321,
          "location_generation_type" : "random",
          "synapse_attractivity" : 100,
          "init_synapse_needed" : False,
          "postsynaptic_current" : 150,
          "plasticity_constant" : 0,
          "postsynaptic_current_max" : 200,
          "neighbor_locator_rule_id" : "rule_1",
          "neighbor_locator_rule_param_id" : "param_2",
          "cortical_mapping_dst" : {
              "vision_memory" : {
                  "neighbor_locator_rule_id" : "rule_6",
                  "neighbor_locator_rule_param_id" : "param_2"
              }
          },
          "neuron_params" : {
              "activation_function_id" : "",
              "depolarization_threshold" : 1.574682375,
              "firing_threshold" : 1.1632698,
              "firing_pattern_id" : "",
              "refractory_period" : 0,
              "orientation_selectivity_id" : "",
              "axon_avg_length" : "",
              "leak_coefficient" : 5,
              "axon_avg_connections" : "",
              "axon_orientation function" : "",
              "consecutive_fire_cnt_max" : 0,
              "snooze_length" : 0.0,
              "block_boundaries" : [
                  28,
                  28,
                  7
              ],
              "geometric_boundaries"  : {
                  "x" : [
                      0,
                      74
                  ],
                  "y" : [
                      0,
                      74
                  ],
                  "z" : [
                      0,
                      74
                  ]
              }
          }
      },
      "vision_IT" : {
          "growth_path" : "",
          "group_id" : "vision",
          "sub_group_id" : "vision_IT",
          "plot_index" : 3,
          "orientation_selectivity_pattern" : "",
          "location" : "",
          "kernel_size" : 7,
          "cortical_neuron_count" : 16836,
          "location_generation_type" : "random",
          "synapse_attractivity" : 80,
          "init_synapse_needed" : False,
          "postsynaptic_current" : 100,
          "plasticity_constant" : 0.5,
          "postsynaptic_current_max" : 300,
          "neighbor_locator_rule_id" : "rule_1",
          "neighbor_locator_rule_param_id" : "param_2",
          "cortical_mapping_dst" : {
              "vision_memory" : {
                  "neighbor_locator_rule_id" : "rule_6",
                  "neighbor_locator_rule_param_id" : "param_2"
              }
          },
          "neuron_params" : {
              "activation_function_id" : "",
              "orientation_selectivity_id" : "",
              "depolarization_threshold" : 1.574682375,
              "firing_threshold" : 0.882,
              "firing_pattern_id" : "",
              "refractory_period" : 0,
              "axon_avg_length" : "",
              "leak_coefficient" : 5,
              "axon_avg_connections" : "",
              "axon_orientation function" : "",
              "consecutive_fire_cnt_max" : 0,
              "snooze_length" : 1.02467266,
              "block_boundaries" : [
                  28,
                  28,
                  1
              ],
              "geometric_boundaries" : {
                  "x" : [
                      0,
                      74
                  ],
                  "y" : [
                      0,
                      74
                  ],
                  "z" : [
                      0,
                      74
                  ]
              }
          }
      },
      "vision_memory" : {
          "growth_path" : "",
          "group_id" : "Memory",
          "sub_group_id" : "vision",
          "plot_index" : 1,
          "orientation_selectivity_pattern" : "",
          "location" : "",
          "kernel_size" : 7,
          "cortical_neuron_count" : 50001,
          "location_generation_type" : "random",
          "synapse_attractivity" : 10,
          "init_synapse_needed" : False,
          "postsynaptic_current" : 0.9,
          "plasticity_constant" : 1,
          "postsynaptic_current_max" : 60,
          "neighbor_locator_rule_id" : "rule_0",
          "neighbor_locator_rule_param_id" : "param_1",
          "cortical_mapping_dst" : {},
          "neuron_params" : {
              "activation_function_id" : "",
              "orientation_selectivity_id" : "",
              "depolarization_threshold" : 5,
              "firing_threshold" : 1.5,
              "firing_pattern_id" : "",
              "refractory_period" : 0,
              "axon_avg_length" : "",
              "leak_coefficient" : 5,
              "axon_avg_connections" : "",
              "axon_orientation function" : "",
              "consecutive_fire_cnt_max" : 2,
              "snooze_length" : 8,
              "block_boundaries" : [
                  28,
                  28,
                  1
              ],
              "geometric_boundaries" : {
                  "x" : [
                      0,
                      201
                  ],
                  "y" : [
                      0,
                      201
                  ],
                  "z" : [
                      0,
                      201
                  ]
              }
          }
      },
      "utf8" : {
          "growth_path" : "",
          "group_id" : "IPU",
          "sub_group_id" : "IPU_utf8",
          "plot_index" : 1,
          "orientation_selectivity_pattern" : "",
          "location" : "",
          "kernel_size" : 7,
          "cortical_neuron_count" : 300,
          "location_generation_type" : "sequential",
          "synapse_attractivity" : 100,
          "init_synapse_needed" : False,
          "postsynaptic_current" : 501,
          "plasticity_constant" : 0.05,
          "postsynaptic_current_max" : 501,
          "neighbor_locator_rule_id" : "rule_0",
          "neighbor_locator_rule_param_id" : "param_1",
          "cortical_mapping_dst" : {
              "utf8_memory" : {
                  "neighbor_locator_rule_id" : "rule_3",
                  "neighbor_locator_rule_param_id" : "param_2"
              }
          },
          "neuron_params" : {
              "activation_function_id" : "",
              "orientation_selectivity_id" : "",
              "depolarization_threshold" : 5,
              "firing_threshold" : 1,
              "firing_pattern_id" : "",
              "refractory_period" : 0,
              "axon_avg_length" : "",
              "leak_coefficient" : 10,
              "axon_avg_connections" : "",
              "axon_orientation function" : "",
              "consecutive_fire_cnt_max" : 3,
              "snooze_length" : 0,
              "block_boundaries" : [
                  1,
                  1,
                  300
              ],
              "geometric_boundaries" : {
                  "x" : [
                      0,
                      1
                  ],
                  "y" : [
                      0,
                      1
                  ],
                  "z" : [
                      0,
                      300
                  ]
              }
          }
      },
      "utf8_memory" : {
          "growth_path" : "",
          "group_id" : "Memory",
          "sub_group_id" : "utf8",
          "plot_index" : 2,
          "orientation_selectivity_pattern" : "",
          "location" : "",
          "kernel_size" : 7,
          "cortical_neuron_count" : 300,
          "location_generation_type" : "sequential",
          "synapse_attractivity" : 100,
          "init_synapse_needed" : False,
          "postsynaptic_current" : 11.2,
          "plasticity_constant" : 1,
          "postsynaptic_current_max" : 11.2,
          "neighbor_locator_rule_id" : "rule_0",
          "neighbor_locator_rule_param_id" : "param_1",
          "cortical_mapping_dst" : {
              "utf8_out" : {
                  "neighbor_locator_rule_id" : "rule_3",
                  "neighbor_locator_rule_param_id" : "param_2"
              }
          },
          "neuron_params" : {
              "activation_function_id" : "",
              "orientation_selectivity_id" : "",
              "depolarization_threshold" : 20,
              "firing_threshold" : 20,
              "firing_pattern_id" : "",
              "refractory_period" : 0,
              "axon_avg_length" : "",
              "leak_coefficient" : 5,
              "axon_avg_connections" : "",
              "axon_orientation function" : "",
              "consecutive_fire_cnt_max" : 100000,
              "snooze_length" : 2,
              "block_boundaries" : [
                  1,
                  1,
                  300
              ],
              "geometric_boundaries" : {
                  "x" : [
                      0,
                      1
                  ],
                  "y" : [
                      0,
                      1
                  ],
                  "z" : [
                      0,
                      300
                  ]
              }
          }
      },
      "utf8_out" : {
          "growth_path" : "",
          "group_id" : "OPU",
          "sub_group_id" : "OPU_utf8",
          "plot_index" : 1,
          "orientation_selectivity_pattern" : "",
          "location" : "",
          "kernel_size" : 7,
          "cortical_neuron_count" : 300,
          "location_generation_type" : "sequential",
          "synapse_attractivity" : 100,
          "init_synapse_needed" : False,
          "postsynaptic_current" : 0.51,
          "plasticity_constant" : 0.05,
          "postsynaptic_current_max" : 1,
          "neighbor_locator_rule_id" : "rule_0",
          "neighbor_locator_rule_param_id" : "param_1",
          "cortical_mapping_dst" : {},
          "neuron_params" : {
              "activation_function_id" : "",
              "orientation_selectivity_id" : "",
              "depolarization_threshold" : 20,
              "firing_threshold" : 10,
              "firing_pattern_id" : "",
              "refractory_period" : 0,
              "axon_avg_length" : "",
              "leak_coefficient" : 1,
              "axon_avg_connections" : "",
              "axon_orientation function" : "",
              "consecutive_fire_cnt_max" : 1,
              "snooze_length" : 0,
              "block_boundaries" : [
                  1,
                  1,
                  300
              ],
              "geometric_boundaries" : {
                  "x" : [
                      0,
                      1
                  ],
                  "y" : [
                      0,
                      1
                  ],
                  "z" : [
                      0,
                      300
                  ]
              }
          }
      },
      "pain" : {
          "growth_path" : "",
          "group_id" : "PAIN",
          "sub_group_id" : "pain",
          "plot_index" : 1,
          "orientation_selectivity_pattern" : "",
          "location" : "",
          "kernel_size" : 7,
          "cortical_neuron_count" : 5,
          "location_generation_type" : "sequential",
          "synapse_attractivity" : 100,
          "init_synapse_needed" : False,
          "postsynaptic_current" : 0.51,
          "plasticity_constant" : 0.05,
          "postsynaptic_current_max" : 1,
          "neighbor_locator_rule_id" : "rule_0",
          "neighbor_locator_rule_param_id" : "param_1",
          "cortical_mapping_dst" : {},
          "neuron_params" : {
              "activation_function_id" : "",
              "orientation_selectivity_id" : "",
              "depolarization_threshold" : 20,
              "firing_threshold" : 10,
              "firing_pattern_id" : "",
              "refractory_period" : 0,
              "axon_avg_length" : "",
              "leak_coefficient" : 1,
              "axon_avg_connections" : "",
              "axon_orientation function" : "",
              "consecutive_fire_cnt_max" : 1,
              "snooze_length" : 0,
              "block_boundaries" : [
                  1,
                  1,
                  1
              ],
              "geometric_boundaries" : {
                  "x" : [
                      0,
                      1
                  ],
                  "y" : [
                      0,
                      1
                  ],
                  "z" : [
                      0,
                      5
                  ]
              }
          }
      }
  }
}

    print(gene_modifier.random_gene_selector(sample_genome))


#     # print(genethesize())


    # genome_1 = {"a": 3, "b": {"name": "mohammad", "last": "nadji"}, "c": 5}
    # genome_2 = {"a": 1, "b": {"name": "jafar", "last": "gholi"}, "c": 6}
    #
    # print(crossover(genome_1, genome_2))

    # a = crossover()
    # for _ in a:
    #     print(_)
    # print(calculate_fitness())

