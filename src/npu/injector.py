
import json
import string
from datetime import datetime
from inf import runtime_data, settings
from ipu.vision import MNIST, retina
from ipu.utf import convert_char_to_fire_list
from art import text2art
from inf.initialize import burst_exit_process
from evo.stats import candidate_list_counter, list_upstream_neuron_count_for_digits


class Injector:

    def __init__(self):
        self.injector_img_flag = False
        self.injector_utf_flag = False
        self.injector_injection_has_begun = False
        self.injector_variation_handler = True
        self.injector_exposure_handler = True
        self.injector_utf_handler = True
        self.injector_variation_counter = int(runtime_data.parameters["Auto_injector"]["variation_default"])
        self.injector_exposure_default = int(runtime_data.parameters["Auto_injector"]["exposure_default"])
        runtime_data.exposure_counter_actual = self.injector_exposure_default
        self.injector_utf_default = int(runtime_data.parameters["Auto_injector"]["utf_default"])
        self.injector_utf_counter_actual = self.injector_utf_default
        self.injector_injection_start_time = datetime.now()
        self.injector_num_to_inject = ''
        self.injector_utf_to_inject = ''
        self.injector_injection_mode = ''
        self.injector_exit_flag = False
        self.injector_burst_skip_flag = False
        self.injector_burst_skip_counter = 0

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
        print("*** *** ***\n\n\n*** *AA BB CC ** ***\n\n\n*** *** ***")

    def utf8_feeder(self):

        # inject label to FCL

        runtime_data.training_neuron_list_utf = set()

        if self.injector_injection_mode == 'c':
            runtime_data.training_neuron_list_utf = \
                convert_char_to_fire_list(self.injector_utf_to_inject)
        else:
            runtime_data.training_neuron_list_utf = \
                convert_char_to_fire_list(str(runtime_data.labeled_image[1]))
            print("!!! Image label: ", runtime_data.labeled_image[1])

        runtime_data.fire_candidate_list['utf8'].update(runtime_data.training_neuron_list_utf)

    @staticmethod
    def img_neuron_list_feeder():

        # inject neuron activity to FCL
        if runtime_data.training_neuron_list_img:
            for cortical_area in runtime_data.v1_members:
                if runtime_data.training_neuron_list_img[cortical_area]:
                    # print("Before FCL injection:", candidate_list_counter(runtime_data.fire_candidate_list),
                    # len(runtime_data.training_neuron_list_img[cortical_area]))
                    runtime_data.fire_candidate_list[cortical_area]. \
                        update(runtime_data.training_neuron_list_img[cortical_area])
                    # print("After FCL injection:", candidate_list_counter(runtime_data.fire_candidate_list))

    @staticmethod
    def image_feeder2(num, seq, mnist_type):

        runtime_data.labeled_image = ['', num]
        runtime_data.training_neuron_list_img = retina(num=num, seq=seq, mnist_type=mnist_type, random_num=False)

    def injection_manager(self, injection_mode, injection_param):
        """
        This function has three modes l1, l2, r and c.
        Mode l1: Assist in learning numbers from 0 to 9
        Mode l2: Assist in learning variations of the same number
        Mode r: Assist in exposing a single image to the brain for a number of bursts
        Mode c: Assist in exposing a single utf8 char to the brain for a number of bursts
        """
        print("\nInjection Manager...")
        try:
            if injection_mode == 'l1':
                self.injector_injection_mode = "l1"
                print("Automatic learning for 0..9 has been turned ON!")
                self.injector_img_flag = True
                self.injector_utf_flag = True
                self.injector_exit_flag = False
                self.injector_burst_skip_flag = False
                self.injector_utf_handler = True
                self.injector_variation_handler = True
                # self.injector_exposure_counter_actual = runtime_data.parameters["Auto_injector"]["exposure_default"]
                self.injector_variation_counter = int(runtime_data.parameters["Auto_injector"]["variation_default"])
                runtime_data.variation_counter_actual = \
                    int(runtime_data.parameters["Auto_injector"]["variation_default"])
                self.injector_utf_default = int(runtime_data.parameters["Auto_injector"]["utf_default"])
                self.injector_utf_counter_actual = int(runtime_data.parameters["Auto_injector"]["utf_default"])
                self.injector_num_to_inject = self.injector_utf_default
                self.injector_burst_skip_counter = \
                    runtime_data.parameters["Auto_injector"]["injector_burst_skip_counter"]

            elif injection_mode == 'l2':
                self.injector_injection_mode = "l2"
                self.injector_img_flag = True
                self.injector_utf_flag = True
                self.injector_burst_skip_flag = False
                self.injector_utf_handler = False
                self.injector_variation_handler = True
                self.injector_variation_counter = runtime_data.parameters["Auto_injector"]["variation_default"]
                runtime_data.variation_counter_actual = runtime_data.parameters["Auto_injector"]["variation_default"]
                self.injector_utf_default = 1
                self.injector_utf_counter_actual = 1
                self.injector_num_to_inject = int(injection_param)
                self.injector_burst_skip_counter = \
                    runtime_data.parameters["Auto_injector"]["injector_burst_skip_counter"]
                print("   <<<   Automatic learning for variations of number << %s >> has been turned ON!   >>>"
                      % injection_param)

            elif injection_mode == 'r':
                self.injector_injection_mode = "r"
                self.injector_variation_handler = False
                self.injector_img_flag = True
                self.injector_utf_flag = False
                self.injector_burst_skip_flag = False
                self.injector_variation_counter = 0
                runtime_data.variation_counter_actual = 0
                self.injector_utf_default = -1
                self.injector_utf_counter_actual = -1
                self.injector_num_to_inject = injection_param

            elif injection_mode == 'c':
                self.injector_injection_mode = "c"
                self.injector_variation_handler = False
                self.injector_utf_handler = False
                self.injector_img_flag = False
                self.injector_utf_flag = True
                self.injector_burst_skip_flag = False
                self.injector_utf_to_inject = injection_param
                self.injector_variation_counter = 0
                runtime_data.variation_counter_actual = 0
                self.injector_utf_default = -1
                self.injector_utf_counter_actual = -1

            else:
                print("Error detecting the injection mode...")
                return

        finally:
            print("Injection Manager... Finally!")
            toggle_injection_mode()
            self.injector_injection_has_begun = True

    def auto_injector(self):

        if self.injector_injection_has_begun:
            # Beginning of a injection process
            print("----------------------------------------Data injection has begun-----------------------------------")
            self.injector_injection_has_begun = False
            self.injector_injection_start_time = datetime.now()
            self.image_feeder2(num=self.injector_num_to_inject,
                               seq=runtime_data.variation_counter_actual,
                               mnist_type='training')

        # Mechanism to skip a number of bursts between each injections to clean-up FCL
        if not self.injector_burst_skip_flag:

            if self.injector_img_flag:
                self.img_neuron_list_feeder()
            if self.injector_utf_flag:
                self.utf8_feeder()

            # Exposure counter
            if not self.injector_burst_skip_flag:
                runtime_data.exposure_counter_actual -= 1

            print('  ----------------------------------------------------------------------------------------    ### ',
                  runtime_data.variation_counter_actual, self.injector_utf_counter_actual,
                  runtime_data.exposure_counter_actual, ' ###')

            # Check if exit condition has been met
            if self.injection_exit_condition() or runtime_data.variation_counter_actual < 1:
                self.injection_exit_process()

            # Counter logic
            if runtime_data.exposure_counter_actual < 1:

                # Effectiveness check
                if runtime_data.parameters["Switches"]["evaluation_based_termination"]:
                    upstream_neuron_count_for_digits = \
                        list_upstream_neuron_count_for_digits(digit=self.injector_utf_counter_actual)
                    print('## ## ###:', upstream_neuron_count_for_digits)
                    if upstream_neuron_count_for_digits[0][1] == 0:
                        print(settings.Bcolors.RED +
                              "\n\n\n\n\n\n!!!!! !! !Terminating the brain due to low training capability! !! !!!" +
                              settings.Bcolors.ENDC)
                        runtime_data.termination_flag = True
                        burst_exit_process()
                        self.injector_exit_flag = True

                if not self.injector_exit_flag:
                    # Resetting exposure counter
                    runtime_data.exposure_counter_actual = self.injector_exposure_default

                    # UTF counter
                    self.injector_utf_counter_actual -= 1

                    # Turning on the skip flag to allow FCL to clear
                    self.injector_burst_skip_flag = True

                    if self.injector_utf_counter_actual < 0:
                        # Resetting counters to their default value
                        self.injector_utf_counter_actual = self.injector_utf_default
                        # Variation counter
                        runtime_data.variation_counter_actual -= 1

                    self.injector_num_to_inject = max(self.injector_utf_counter_actual, 0)
                    runtime_data.flag_ready_to_inject_image = True

                    # Saving brain to disk
                    # todo: assess the impact of the following disk operation
                    if runtime_data.parameters["Switches"]["save_connectome_to_disk"]:
                        for cortical_area in runtime_data.cortical_list:
                            with open(runtime_data.parameters['InitData']['connectome_path'] +
                                      cortical_area + '.json', "r+") as data_file:
                                data = runtime_data.brain[cortical_area]
                                for _ in data:
                                    data[_]['activity_history'] = ""
                                data_file.seek(0)  # rewind
                                data_file.write(json.dumps(data, indent=3))
                                data_file.truncate()

        else:
            print("Skipping the injection for this round...")
            self.injector_burst_skip_counter -= 1
            if self.injector_burst_skip_counter <= 0 or candidate_list_counter(runtime_data.fire_candidate_list) < 1:
                self.injector_burst_skip_counter = runtime_data.parameters["Auto_injector"][
                    "injector_burst_skip_counter"]
                self.injector_burst_skip_flag = False

        if runtime_data.flag_ready_to_inject_image and not self.injector_burst_skip_flag:
            print("self.num_to_inject: ", self.injector_num_to_inject)
            self.image_feeder2(num=self.injector_num_to_inject,
                               seq=runtime_data.variation_counter_actual,
                               mnist_type='training')
            runtime_data.flag_ready_to_inject_image = False

    def injection_exit_condition(self):
        if (self.injector_utf_handler and
            self.injector_utf_counter_actual < 1 and
            runtime_data.variation_counter_actual < 1 and
            runtime_data.exposure_counter_actual < 1) or \
                (not self.injector_utf_handler and
                 self.injector_variation_handler and
                 runtime_data.variation_counter_actual < 1 and
                 runtime_data.exposure_counter_actual < 1) or \
                (not self.injector_utf_handler and
                 not self.injector_variation_handler and
                 runtime_data.exposure_counter_actual < 1):
            exit_condition = True
            print(">> Injection exit condition has been met <<")
        else:
            exit_condition = False
        return exit_condition

    def injection_exit_process(self):

        runtime_data.parameters["Auto_injector"]["injector_status"] = False
        self.injector_num_to_inject = ''
        runtime_data.exposure_counter_actual = int(runtime_data.parameters["Auto_injector"]["exposure_default"])
        runtime_data.variation_counter_actual = int(runtime_data.parameters["Auto_injector"]["variation_default"])
        self.injector_utf_counter_actual = int(runtime_data.parameters["Auto_injector"]["utf_default"])
        injection_duration = datetime.now() - self.injector_injection_start_time
        print("----------------------------All injection rounds has been completed-----------------------------")
        print("Total injection duration was: ", injection_duration)
        print("-----------------------------------------------------------------------------------------------")
        if runtime_data.parameters["Auto_injector"]["epochs"] == 0:
            if runtime_data.parameters["Switches"]["live_mode"] and \
                    runtime_data.live_mode_status == 'learning':
                runtime_data.live_mode_status = 'testing'
                print(settings.Bcolors.RED + "\n\n\n\n\n"
                                             "Starting automated testing process \n\n\n\n\nXXX XXX XXX XXX XXX\n\n\n\n"
                                             "-----------------------------------------------------------------\n"
                                             "-----------------------------------------------------------------" +
                      settings.Bcolors.ENDC)
                self.test_manager(test_mode="t1", test_param="")
        else:
            runtime_data.parameters["Auto_injector"]["epochs"] -= 1
            self.injection_manager(injection_mode="l1", injection_param="")
            print(text2art("EPOCH_" + str(runtime_data.parameters["Auto_injector"]["epochs"]), font='block'))

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

            print('  ----------------------------------------------------------------------------------------    ### ',
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
        burst_exit_process()


def toggle_injection_mode():
    if runtime_data.parameters["Auto_injector"]["injector_status"]:
        runtime_data.parameters["Auto_injector"]["injector_status"] = False
        print("Auto_train mode is Turned OFF!")
    else:
        runtime_data.parameters["Auto_injector"]["injector_status"] = True
        print("Auto_train mode is Turned On!")


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
    return (str(datetime.now()).replace(' ', '_')).replace('.', '_')+'_'+(''.join(random.choice(chars)
                                                                                  for _ in range(size)))+'_T'



