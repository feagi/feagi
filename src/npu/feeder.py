"""
This module plays the role of the information gateway to the cortical neurons. After physical stimuli is processed by
the Input Processing Unit (IPU), it will be translated to neuronal activation and fed to Fire Candidate List (FCL).

Expectations:
- To receive the list of activated IPU neurons from the corresponding IPU module and inject them into FCL
- To monitor flow of information and act as needed

Q) Why the need for seperate module? Can this be merged with the burst-engine? or, each IPU can directly inject to FCL!
"""
from queue import Queue
from inf import runtime_data, settings
from ipu.vision import retina
from ipu.mnist import MNIST
from ipu.utf import convert_char_to_fire_list
from inf.initialize import exit_burst_process
from evo.stats import candidate_list_counter, list_upstream_neuron_count_for_digits



class Feeder:

    def __init__(self):

    def eval(self):
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
                exit_burst_process()
                self.injector_exit_flag = True

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
                    # print("After FCL injection:", candidate_list_counter(runtime_data.fire_candidate_list))0

    @staticmethod
    def mnist_feeder(num, seq, mnist_type):
        runtime_data.labeled_image = ['', num]

        # todo: define a function to return mnist image (should already have one)
        image = {}

        runtime_data.training_neuron_list_img = retina(image=image, polarize=True)


