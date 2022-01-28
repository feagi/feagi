
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

parameters = {}
genome = {}
genome_stats = {}
genome_test_stats = []
brain = {}
cortical_list = []
cortical_map = {}
intercortical_mapping = []
block_dic = {}
upstream_neurons = {}
memory_list = {}
activity_stats = {}
temp_neuron_list = []
original_genome_id = []
fire_list = []
termination_flag = False
variation_counter_actual = 0
exposure_counter_actual = 0
mnist_training = {}
mnist_testing = {}
top_10_utf_memory_neurons = {}
top_10_utf_neurons = {}
v1_members = []
prunning_candidates = set()
genome_id = ""
event_id = '_'
blueprint = ""
comprehension_queue = ''
working_directory = ''
connectome_path = ''
paths = {}
watchdog_queue = ''
exit_condition = False
fcl_queue = ''
proximity_queue = ''
last_ipu_activity = ''
last_alertness_trigger = ''
influxdb = ''
mongodb = ''
running_in_container = False
hardware = ''
gazebo = False
stimulation_data = {}
hw_controller_path = ''
hw_controller = None
opu_pub = None
brain_activity_pub = None
brain_activity_pub_freq = 1
router_address = None
burst_timer = None
genome2 = {}
genome_ver = None
fire_queue = {}
controller_config = None


# rules = ""
brain_is_running = False

# live_mode_status can have modes of idle, learning, testing, tbd
live_mode_status = 'idle'
fcl_history = {}
brain_run_id = ""
burst_detection_list = {}
burst_count = 0
fire_candidate_list = {}
previous_fcl = {}
future_fcl = {}
labeled_image = []
training_neuron_list_utf = {}
training_neuron_list_img = {}
empty_fcl_counter = 0
neuron_mp_list = []
pain_flag = False
cumulative_neighbor_count = 0
time_neuron_update = ''
time_apply_plasticity_ext = ''
plasticity_time_total = None
plasticity_time_total_p1 = None
plasticity_dict = {}

tester_test_stats = {}

# Flags
flag_ready_to_inject_image = False
