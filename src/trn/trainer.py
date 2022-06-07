
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
Responsible for orchestrating the exposure of the brain to various information for learning purposes.

Learning strategies:
- Expose multiple stimuli at the same time with the purpose of building association between
them.
- Reinforcement learning
- ...

todo: build a switcher so different learning methods can be selected
todo: a system that can trigger multiple data feeder instances at the same time


"""

from inf import runtime_data
from evo.voxels import neurons_in_the_block


def shock_manager():
    shock_cortical_area = 'ishock'
    shock_register = list()

    shock_register.append(shock_scenario_1())
    shock_register.append(shock_scenario_2())

    max_shock_level = max(shock_register)
    shock_voxel = '0-0-' + str(max_shock_level - 1)

    if max_shock_level in range(1, 11):
        shock_neurons = neurons_in_the_block(shock_cortical_area, shock_voxel)
        for neuron in shock_neurons:
            runtime_data.fire_candidate_list[shock_cortical_area].add(neuron)


def shock_scenario_1():
    """
    Each shock scenario returns a shock level from 0 to 10 where 0 corresponds to no shock and 10 the highest level

    Example: A scenario can be defined as if the robot enters a specific region of the environment to be shocked or
    if the robot steps on a dark surface to be shocked.


    IR centric shock. based on the level of exposure to lightness or darkness, different levels of shock to be applied

    """
    shock_level = 0
    for neuron in runtime_data.fire_candidate_list["i__inf"]:
        shock_level += 3

    return shock_level


def shock_scenario_2():
    """
    TBD
    """
    shock_level = 0
    return shock_level







# class Trainer:
#     def __init__(self):
#         self.training_epochs = runtime_data.training_epochs
#
#     def training_manager(self):
#         # for epoch in self.training_epochs:
#         #     for
#         return
#
#
# class Trainer_old:
#     def __init__(self):
#         self.injector_img_flag = False
#         self.injector_utf_flag = False
#         self.injector_injection_has_begun = False
#         self.injector_variation_handler = True
#         self.injector_exposure_handler = True
#         self.injector_utf_handler = True
#         self.injector_variation_counter = int(runtime_data.parameters["Auto_injector"]["variation_default"])
#         self.injector_exposure_default = int(runtime_data.parameters["Auto_injector"]["exposure_default"])
#         runtime_data.exposure_counter_actual = self.injector_exposure_default
#         self.injector_utf_default = int(runtime_data.parameters["Auto_injector"]["utf_default"])
#         self.injector_utf_counter_actual = self.injector_utf_default
#         self.injector_injection_start_time = datetime.now()
#         self.injector_num_to_inject = ''
#         self.injector_utf_to_inject = ''
#         self.injector_injection_mode = ''
#         self.injector_exit_flag = False
#         self.injector_burst_skip_flag = False
#         self.injector_burst_skip_counter = 0
#         self.mnist = MNIST()
#
#     def training_manager(self, injection_mode, injection_param):
#         """
#         This function has three modes l1, l2, r and c.
#         Mode l1: Assist in learning numbers from 0 to 9
#         Mode l2: Assist in learning variations of the same number
#         Mode r: Assist in exposing a single image to the brain for a number of bursts
#         Mode c: Assist in exposing a single utf8 char to the brain for a number of bursts
#         """
#         print("\nInjection Manager...")
#         try:
#             if injection_mode == 'l1':
#                 self.injector_injection_mode = "l1"
#                 print("Automatic learning for 0..9 has been turned ON!")
#                 self.injector_img_flag = True
#                 self.injector_utf_flag = True
#                 self.injector_exit_flag = False
#                 self.injector_burst_skip_flag = False
#                 self.injector_utf_handler = True
#                 self.injector_variation_handler = True
#                 # self.injector_exposure_counter_actual = runtime_data.parameters["Auto_injector"]["exposure_default"]
#                 self.injector_variation_counter = int(runtime_data.parameters["Auto_injector"]["variation_default"])
#                 runtime_data.variation_counter_actual = \
#                     int(runtime_data.parameters["Auto_injector"]["variation_default"])
#                 self.injector_utf_default = int(runtime_data.parameters["Auto_injector"]["utf_default"])
#                 self.injector_utf_counter_actual = int(runtime_data.parameters["Auto_injector"]["utf_default"])
#                 self.injector_num_to_inject = self.injector_utf_default
#                 self.injector_burst_skip_counter = \
#                     runtime_data.parameters["Auto_injector"]["injector_burst_skip_counter"]
#
#             elif injection_mode == 'l2':
#                 self.injector_injection_mode = "l2"
#                 self.injector_img_flag = True
#                 self.injector_utf_flag = True
#                 self.injector_burst_skip_flag = False
#                 self.injector_utf_handler = False
#                 self.injector_variation_handler = True
#                 self.injector_variation_counter = runtime_data.parameters["Auto_injector"]["variation_default"]
#                 runtime_data.variation_counter_actual = runtime_data.parameters["Auto_injector"]["variation_default"]
#                 self.injector_utf_default = 1
#                 self.injector_utf_counter_actual = 1
#                 self.injector_num_to_inject = int(injection_param)
#                 self.injector_burst_skip_counter = \
#                     runtime_data.parameters["Auto_injector"]["injector_burst_skip_counter"]
#                 print("   <<<   Automatic learning for variations of number << %s >> has been turned ON!   >>>"
#                       % injection_param)
#
#             elif injection_mode == 'r':
#                 self.injector_injection_mode = "r"
#                 self.injector_variation_handler = False
#                 self.injector_img_flag = True
#                 self.injector_utf_flag = False
#                 self.injector_burst_skip_flag = False
#                 self.injector_variation_counter = 0
#                 runtime_data.variation_counter_actual = 0
#                 self.injector_utf_default = -1
#                 self.injector_utf_counter_actual = -1
#                 self.injector_num_to_inject = injection_param
#
#             elif injection_mode == 'c':
#                 self.injector_injection_mode = "c"
#                 self.injector_variation_handler = False
#                 self.injector_utf_handler = False
#                 self.injector_img_flag = False
#                 self.injector_utf_flag = True
#                 self.injector_burst_skip_flag = False
#                 self.injector_utf_to_inject = injection_param
#                 self.injector_variation_counter = 0
#                 runtime_data.variation_counter_actual = 0
#                 self.injector_utf_default = -1
#                 self.injector_utf_counter_actual = -1
#
#             else:
#                 print("Error detecting the injection mode...")
#                 return
#
#         finally:
#             print("Injection Manager... Finally!")
#             toggle_injection_mode()
#             self.injector_injection_has_begun = True
#
#     def auto_train(self):
#         if runtime_data.parameters["Auto_injector"]["injector_status"]:
#             if self.injector_injection_has_begun:
#                 # Beginning of a injection process
#                 print("----------------------------------------Data injection has begun-------------------------------")
#                 self.injector_injection_has_begun = False
#                 self.injector_injection_start_time = datetime.now()
#                 self.image_feeder2(num=self.injector_num_to_inject,
#                                    seq=runtime_data.variation_counter_actual,
#                                    mnist_type='training')
#
#             # Mechanism to skip a number of bursts between each injections to clean-up FCL
#             if not self.injector_burst_skip_flag:
#
#                 if self.injector_img_flag:
#                     self.img_neuron_list_feeder()
#                 if self.injector_utf_flag:
#                     self.utf8_feeder()
#
#                 # Exposure counter
#                 if not self.injector_burst_skip_flag:
#                     runtime_data.exposure_counter_actual -= 1
#
#                 print('  -------------------------------------------------------------------------------------    ### ',
#                       runtime_data.variation_counter_actual, self.injector_utf_counter_actual,
#                       runtime_data.exposure_counter_actual, ' ###')
#
#                 # Check if exit condition has been met
#                 if self.injection_exit_condition() or runtime_data.variation_counter_actual < 1:
#                     self.injection_exit_process()
#
#                 # Counter logic
#                 if runtime_data.exposure_counter_actual < 1:
#
#                     # Effectiveness check
#                     if runtime_data.parameters["Switches"]["evaluation_based_termination"]:
#                         upstream_neuron_count_for_digits = \
#                             list_upstream_neuron_count_for_digits(digit=self.injector_utf_counter_actual)
#                         print('## ## ###:', upstream_neuron_count_for_digits)
#                         if upstream_neuron_count_for_digits[0][1] == 0:
#                             print(settings.Bcolors.RED +
#                                   "\n\n\n\n\n\n!!!!! !! !Terminating the brain due to low training capability! !! !!!" +
#                                   settings.Bcolors.ENDC)
#                             runtime_data.termination_flag = True
#                             exit_burst_process()
#                             self.injector_exit_flag = True
#
#                     if not self.injector_exit_flag:
#                         # Resetting exposure counter
#                         runtime_data.exposure_counter_actual = self.injector_exposure_default
#
#                         # UTF counter
#                         self.injector_utf_counter_actual -= 1
#
#                         # Turning on the skip flag to allow FCL to clear
#                         self.injector_burst_skip_flag = True
#
#                         if self.injector_utf_counter_actual < 0:
#                             # Resetting counters to their default value
#                             self.injector_utf_counter_actual = self.injector_utf_default
#                             # Variation counter
#                             runtime_data.variation_counter_actual -= 1
#
#                         self.injector_num_to_inject = max(self.injector_utf_counter_actual, 0)
#                         runtime_data.flag_ready_to_inject_image = True
#
#                         # Saving brain to disk
#                         # todo: assess the impact of the following disk operation
#                         if runtime_data.parameters["Switches"]["save_connectome_to_disk"]:
#                             for cortical_area in runtime_data.cortical_list:
#                                 with open(runtime_data.parameters['InitData']['connectome_path'] +
#                                           cortical_area + '.json', "r+") as data_file:
#                                     data = runtime_data.brain[cortical_area]
#                                     for _ in data:
#                                         data[_]['activity_history'] = ""
#                                     data_file.seek(0)  # rewind
#                                     data_file.write(json.dumps(data, indent=3))
#                                     data_file.truncate()
#
#             else:
#                 print("Skipping the injection for this round...")
#                 self.injector_burst_skip_counter -= 1
#                 if self.injector_burst_skip_counter <= 0 or \
#                         candidate_list_counter(runtime_data.fire_candidate_list) < 1:
#                     self.injector_burst_skip_counter = runtime_data.parameters["Auto_injector"][
#                         "injector_burst_skip_counter"]
#                     self.injector_burst_skip_flag = False
#
#             if runtime_data.flag_ready_to_inject_image and not self.injector_burst_skip_flag:
#                 print("self.num_to_inject: ", self.injector_num_to_inject)
#                 self.image_feeder2(num=self.injector_num_to_inject,
#                                    seq=runtime_data.variation_counter_actual,
#                                    mnist_type='training')
#                 runtime_data.flag_ready_to_inject_image = False
#
#     def trainer_exit_condition(self):
#         if (self.injector_utf_handler and
#             self.injector_utf_counter_actual < 1 and
#             runtime_data.variation_counter_actual < 1 and
#             runtime_data.exposure_counter_actual < 1) or \
#                 (not self.injector_utf_handler and
#                  self.injector_variation_handler and
#                  runtime_data.variation_counter_actual < 1 and
#                  runtime_data.exposure_counter_actual < 1) or \
#                 (not self.injector_utf_handler and
#                  not self.injector_variation_handler and
#                  runtime_data.exposure_counter_actual < 1):
#             exit_condition = True
#             print(">> Injection exit condition has been met <<")
#         else:
#             exit_condition = False
#         return exit_condition
#
#     def trainer_exit_process(self):
#         runtime_data.parameters["Auto_injector"]["injector_status"] = False
#         self.injector_num_to_inject = ''
#         runtime_data.exposure_counter_actual = int(runtime_data.parameters["Auto_injector"]["exposure_default"])
#         runtime_data.variation_counter_actual = int(runtime_data.parameters["Auto_injector"]["variation_default"])
#         self.injector_utf_counter_actual = int(runtime_data.parameters["Auto_injector"]["utf_default"])
#         injection_duration = datetime.now() - self.injector_injection_start_time
#         print("----------------------------All injection rounds has been completed-----------------------------")
#         print("Total injection duration was: ", injection_duration)
#         print("-----------------------------------------------------------------------------------------------")
#         if runtime_data.parameters["Auto_injector"]["epochs"] == 0:
#             if runtime_data.parameters["Switches"]["live_mode"] and \
#                     runtime_data.live_mode_status == 'learning':
#                 runtime_data.live_mode_status = 'testing'
#                 print(settings.Bcolors.RED + "\n\n\n\n\n"
#                                              "Starting automated testing process \n\n\n\n\nXXX XXX XXX XXX XXX\n\n\n\n"
#                                              "-----------------------------------------------------------------\n"
#                                              "-----------------------------------------------------------------" +
#                       settings.Bcolors.ENDC)
#                 self.test_manager(test_mode="t1", test_param="")
#         else:
#             runtime_data.parameters["Auto_injector"]["epochs"] -= 1
#             self.injection_manager(injection_mode="l1", injection_param="")
#             print(text2art("EPOCH_" + str(runtime_data.parameters["Auto_injector"]["epochs"]), font='block'))
#
#
# def toggle_training_mode():
#     if runtime_data.parameters["Auto_injector"]["injector_status"]:
#         runtime_data.parameters["Auto_injector"]["injector_status"] = False
#         print("Auto_train mode is Turned OFF!")
#     else:
#         runtime_data.parameters["Auto_injector"]["injector_status"] = True
#         print("Auto_train mode is Turned On!")
