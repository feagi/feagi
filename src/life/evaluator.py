"""
Set of functions to evaluate the fitness of the brain from various perspectives
"""
import string
import random
from datetime import datetime
from inf import runtime_data, settings
from ipu.vision import MNIST, retina
from inf.initialize import exit_burst_process
from evo.stats import candidate_list_counter, list_upstream_neuron_count_for_digits


class Tester:

    def __init__(self):
        self.tester_img_flag = False
        self.tester_utf_flag = False
        self.tester_testing_has_begun = False
        self.tester_variation_handler = True
        self.tester_exposure_handler = True
        self.tester_utf_handler = True
        self.tester_variation_counter = runtime_data.parameters["Auto_tester"]["variation_default"]
        self.tester_exposure_default = runtime_data.parameters["Auto_tester"]["exposure_default"]
        self.tester_utf_default = runtime_data.parameters["Auto_tester"]["utf_default"]
        self.tester_utf_counter_actual = self.tester_utf_default
        self.tester_test_start_time = datetime.now()
        self.tester_num_to_inject = ''
        self.tester_test_mode = ''
        self.tester_comprehension_counter = 0
        self.tester_test_attempt_counter = 0
        self.tester_no_response_counter = 0
        # self.tester_temp_stats = []
        self.tester_test_id = ""
        self.tester_exit_flag = False
        self.tester_burst_skip_flag = False
        self.tester_burst_skip_counter = 0
        print("-------------------------++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ Injector init")
        self.mnist = MNIST()

    def test_manager(self, test_mode, test_param):
        """
        This function has three modes t1, t2.
        Mode t1: Assist in learning numbers from 0 to 9
        Mode t2: Assist in learning variations of the same number
        """
        try:
            if test_mode == 't1':
                self.tester_test_mode = "t1"
                print("Automatic learning for 0..9 has been turned ON!")
                self.tester_img_flag = True
                self.tester_utf_flag = True
                self.tester_exit_flag = False
                self.tester_burst_skip_flag = False
                self.tester_utf_handler = True
                self.tester_variation_handler = True
                self.tester_variation_counter = runtime_data.parameters["Auto_tester"]["variation_default"]
                runtime_data.variation_counter_actual = runtime_data.parameters["Auto_tester"]["variation_default"]
                self.tester_utf_default = runtime_data.parameters["Auto_tester"]["utf_default"]
                self.tester_utf_counter_actual = runtime_data.parameters["Auto_tester"]["utf_default"]
                self.tester_num_to_inject = self.tester_utf_default
                self.tester_burst_skip_counter = runtime_data.parameters["Auto_tester"]["tester_burst_skip_counter"]

            elif test_mode == 't2':
                self.tester_test_mode = "t2"
                self.tester_img_flag = True
                self.tester_utf_flag = True
                self.tester_exit_flag = False
                self.tester_burst_skip_flag = False
                self.tester_utf_handler = False
                self.tester_variation_handler = True
                self.tester_variation_counter = runtime_data.parameters["Auto_tester"]["variation_default"]
                runtime_data.variation_counter_actual = runtime_data.parameters["Auto_tester"]["variation_default"]
                self.tester_utf_default = -1
                self.tester_utf_counter_actual = -1
                self.tester_num_to_inject = int(test_param)
                self.tester_burst_skip_counter = runtime_data.parameters["Auto_tester"]["tester_burst_skip_counter"]
                print("   <<<   Automatic learning for variations of number << %s >> has been turned ON!   >>>"
                      % test_param)

            else:
                print("Error detecting the test mode...")
                return

        finally:
            toggle_test_mode()
            self.tester_test_id = test_id_gen()
            runtime_data.tester_test_stats["genome_id"] = runtime_data.genome_id
            print('Genome_id = ', runtime_data.genome_id)
            runtime_data.tester_test_stats["test_id"] = self.tester_test_id
            self.tester_testing_has_begun = True

    @staticmethod
    def create_test_stat_template():
        template = {}
        for i in range(10):
            template[i] = {
                "exposed": 0,
                "no_response": 0,
                "comprehended": 0
            }
        return template

    @staticmethod
    def test_stat_counter_incrementer(digit, stat_type):
        runtime_data.tester_test_stats[digit][stat_type] += 1

    def auto_tester(self):
        if runtime_data.parameters["Auto_tester"]["tester_status"]:
            """
            Test approach:

            - Ask user for number of times testing every digit call it x
            - Inject each number x rounds with each round conclusion being a "Comprehensions"
            - Count the number of True vs. False Comprehensions
            - Collect stats for each number and report at the end of testing

            """
            if self.tester_testing_has_begun:
                # Beginning of a injection process
                print("----------------------------------------Testing has begun------------------------------------")
                self.tester_testing_has_begun = False
                runtime_data.tester_test_stats = self.create_test_stat_template()
                self.tester_test_start_time = datetime.now()
                if self.tester_img_flag:
                    # todo: temporarily changing test data set to training instead <<< CHANGE IT BACK!!! >>>
                    self.image_feeder2(num=self.tester_num_to_inject,
                                       seq=runtime_data.variation_counter_actual,
                                       mnist_type='test')

            # Mechanism to skip a number of bursts between each injections to clean-up FCL
            if not self.tester_burst_skip_flag:

                print("                                              .... .. .. .. ... New Exposure ... ... ... .. .. ")

                # Injecting test image to the FCL
                if self.tester_img_flag:
                    self.img_neuron_list_feeder()
                    # print("Test image data just got injected into FCL")

                # Exposure counter
                runtime_data.exposure_counter_actual -= 1

                print('  -------------------------------------------------------------------------------------    ### ',
                      runtime_data.variation_counter_actual,
                      self.tester_utf_counter_actual,
                      runtime_data.exposure_counter_actual, ' ###')

                if runtime_data.exposure_counter_actual < 1:
                    # Turning on the skip flag to allow FCL to clear
                    self.tester_burst_skip_flag = True

            else:
                print("Skipping the injection for this round...")
                self.tester_burst_skip_counter -= 1
                if self.tester_burst_skip_counter <= 0 or candidate_list_counter(runtime_data.fire_candidate_list) < 1:
                    self.tester_burst_skip_counter = runtime_data.parameters["Auto_tester"]["tester_burst_skip_counter"]
                    self.tester_burst_skip_flag = False

                    # Final phase of a single test instance is to evaluate the comprehension when FCL is cleaned up
                    self.test_comprehension_logic()
                    self.tester_test_attempt_counter += 1

                    print("Number to inject:", self.tester_num_to_inject)
                    print("Default counters:", self.tester_variation_counter,
                          self.tester_utf_default,
                          self.tester_exposure_default)

                    print("Current Counters:", runtime_data.variation_counter_actual,
                          self.tester_utf_counter_actual,
                          runtime_data.exposure_counter_actual)

                    # Resetting exposure counter
                    runtime_data.exposure_counter_actual = self.tester_exposure_default

                    # UTF counter
                    self.tester_utf_counter_actual -= 1
                    self.test_stat_counter_incrementer(digit=self.tester_num_to_inject, stat_type='exposed')

                    # self.test_stats_report()

                    if self.tester_utf_flag:
                        self.tester_num_to_inject -= 1
                        print('#-#-# Current test UTF counter is ', self.tester_utf_counter_actual)
                        print("#-#-# Current number to inject is :", self.tester_num_to_inject)

                        print(".... .. .. .. ... .... .. .. . ... ... ... .. .. ")
                        print(".... .. .. .. ... .... .. .. . ... ... ... .. .. ")
                        print(".... .. .. .. ... New UTF .. . ... ... ... .. .. ")
                        print(".... .. .. .. ... ... .. .. .  ... ... ... .. .. ")
                        print(".... .. .. .. ... .... .. .. . ... ... ... .. .. ")

                    if self.tester_utf_counter_actual < 0:
                        runtime_data.variation_counter_actual -= 1

                        # Variation logic
                        if runtime_data.variation_counter_actual < 1:
                            print(">> Test exit condition has been met <<")
                            self.tester_exit_flag = True
                            self.test_exit_process()

                        print(".... .. .. .. ... .... .. .. . ... ... ... .. .. ")
                        print(".... .. .. .. ... New Variation... ... ... .. .. ")
                        print(".... .. .. .. ... ... .. .. .  ... ... ... .. .. ")

                        # Resetting counters
                        runtime_data.exposure_counter_actual = self.tester_exposure_default
                        self.tester_utf_counter_actual = self.tester_utf_default
                        self.tester_num_to_inject = self.tester_utf_default
                        self.tester_test_attempt_counter = 0
                        self.tester_comprehension_counter = 0
                        self.tester_no_response_counter = 0

                    if self.tester_img_flag and not self.tester_exit_flag:
                        print('#-#-# Current number that is about to be tested is ', self.tester_num_to_inject)
                        # todo: temporarily changing test data set to training instead <<< CHANGE IT BACK!!! >>>
                        self.image_feeder2(num=self.tester_num_to_inject,
                                           seq=runtime_data.variation_counter_actual,
                                           mnist_type='test')

    # def update_test_stats(self):
    #     # Initialize parameters
    #     utf_exposed = str(self.tester_num_to_inject) + '_exposed'
    #     utf_comprehended = str(self.tester_num_to_inject) + '_comprehended'
    #     utf_no_response = str(self.tester_num_to_inject) + '_no_response'
    #     if utf_exposed not in self.tester_test_stats:
    #         self.tester_test_stats[utf_exposed] = runtime_data.parameters["Auto_tester"]["utf_default"]
    #     if utf_comprehended not in self.tester_test_stats:
    #         self.tester_test_stats[utf_comprehended] = 0
    #     if utf_no_response not in self.tester_test_stats:
    #         self.tester_test_stats[utf_no_response] = 0
    #
    #     # Add current stats to the list
    #     self.tester_test_stats[utf_exposed] = self.tester_test_attempt_counter
    #     self.tester_test_stats[utf_comprehended] = self.tester_comprehension_counter
    #     self.tester_test_stats[utf_no_response] = self.tester_no_response_counter
    #     print('no_response_counter: ', self.tester_no_response_counter)
    #     print('comprehension_counter: ', self.tester_comprehension_counter)
    #     print('attempted_counter: ', self.tester_test_attempt_counter)

    def test_comprehension_logic(self):
        # Comprehension logic
        print("\n****************************************")
        print("Comprehended char> ", runtime_data.parameters["Input"]["comprehended_char"], "  Injected char> ",
              self.tester_num_to_inject)
        print("****************************************\n")
        if runtime_data.parameters["Input"]["comprehended_char"] in ['', '-']:
            self.tester_no_response_counter += 1
            self.test_stat_counter_incrementer(digit=self.tester_num_to_inject, stat_type='no_response')
            print("() () () No response was logged () () () > Currently no-response counter is",
                  self.tester_no_response_counter)
            runtime_data.parameters["Input"]["comprehended_char"] = ''
        elif runtime_data.parameters["Input"]["comprehended_char"] == str(self.tester_num_to_inject):
            self.test_stat_counter_incrementer(digit=self.tester_num_to_inject, stat_type='comprehended')
            print(settings.Bcolors.HEADER +
                  "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^"
                  + settings.Bcolors.ENDC)
            print(settings.Bcolors.HEADER +
                  ">> >> >>                                                   << << <<"
                  + settings.Bcolors.ENDC)
            print(settings.Bcolors.HEADER +
                  ">> >> >> The Brain successfully identified the image as > %s < !!!!"
                  % runtime_data.parameters["Input"]["comprehended_char"] + settings.Bcolors.ENDC)
            print(settings.Bcolors.HEADER +
                  ">> >> >>                                                   << << <<"
                  + settings.Bcolors.ENDC)
            print(settings.Bcolors.HEADER +
                  "vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv"
                  + settings.Bcolors.ENDC)
            self.tester_comprehension_counter += 1
            runtime_data.parameters["Input"]["comprehended_char"] = ''

    @staticmethod
    def test_stats_report():
        # print(runtime_data.tester_test_stats)
        print("\n-----------------------------------------------------------------------------------------------")
        print("Test statistics:")
        print("-----------------------------------------------------------------------------------------------")

        print(settings.Bcolors.OKGREEN + '               ', end='')
        for _ in range(1 + runtime_data.parameters["Auto_tester"]["utf_default"]):
            print(_, '    ', end='')

        print(settings.Bcolors.ENDC + '\nExposed:       ', end='')
        for _ in range(10):
            print(runtime_data.tester_test_stats[_]['exposed'], '    ', end='')

        print('\nNo Response:   ', end='')
        for _ in range(10):
            print(runtime_data.tester_test_stats[_]['no_response'], '    ', end='')

        print('\nComprehended:  ', end='')
        for _ in range(10):
            print(runtime_data.tester_test_stats[_]['comprehended'], '    ', end='')

        print("\n-----------------------------------------------------------------------------------------------")
        print("-----------------------------------------------------------------------------------------------")

    def test_exit_process(self):
        runtime_data.parameters["Auto_tester"]["tester_status"] = False
        # runtime_data.exposure_counter_actual = runtime_data.parameters["Auto_tester"]["exposure_default"]
        # runtime_data.variation_counter_actual = runtime_data.parameters["Auto_tester"]["variation_default"]
        # self.tester_utf_counter_actual = runtime_data.parameters["Auto_tester"]["utf_default"]
        test_duration = datetime.now() - self.tester_test_start_time
        print("----------------------------All testing rounds has been completed-----------------------------")
        print("Total test duration was: ", test_duration)
        print("-----------------------------------------------------------------------------------------------")

        self.test_stats_report()

        print("test_stats:\n", runtime_data.tester_test_stats)

        self.tester_test_attempt_counter = 0
        self.tester_comprehension_counter = 0
        self.tester_no_response_counter = 0
        # logging stats into Genome
        runtime_data.genome_test_stats.append(runtime_data.tester_test_stats.copy())
        runtime_data.tester_test_stats = self.create_test_stat_template()
        runtime_data.live_mode_status = 'idle'
        print(settings.Bcolors.RED + "Burst exit triggered by the automated workflow >< >< >< >< >< " +
              settings.Bcolors.ENDC)
        exit_burst_process()


def toggle_test_mode():
    if runtime_data.parameters["Auto_tester"]["tester_status"]:
        runtime_data.parameters["Auto_tester"]["tester_status"] = False
        print("Auto_test mode is Turned OFF!")
    else:
        runtime_data.parameters["Auto_tester"]["tester_status"] = True
        print("Auto_test mode is Turned On!")


def test_id_gen(size=6, chars=string.ascii_uppercase + string.digits):
    """
    This function generates a unique id which will be associated with each neuron
    :param size:
    :param chars:
    :return:
    """
    # Rand gen source partially from:
    # http://stackoverflow.com/questions/2257441/random-string-generation-with-upper-case-letters-and-digits-in-python
    return (str(datetime.now()).replace(' ', '_')).replace('.', '_') + '_' + (''.join(random.choice(chars)
                                                                                      for _ in range(size))) + '_T'
