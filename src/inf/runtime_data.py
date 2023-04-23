
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


# System
parameters = {}
event_id = '_'
last_ipu_activity = ''
last_alertness_trigger = ''
influxdb = ''
mongodb = ''
running_in_container = False
hardware = ''
gazebo = False
hw_controller_path = ''
hw_controller = None
opu_pub = None
brain_activity_pub = None
brain_activity_pub_freq = 1
controller_config = None
feagi_thread = None
beacon_sub = set()
beacon_flag = False
feagi_state = {
    "state": "idle"
}
working_directory = ''
connectome_path = ''
paths = {}


# Evolutionary
genome = {}
genome_stats = {}
genome_test_stats = []
genome_orig = {}
genome2 = {}
genome_ver = None
genome_id = ""
genome_file_name = None
genome_counter = 0
autopilot = False
generation_dict = None
current_generation_dict_id = None
generation_id = None
genome_reset_flag = False
last_genome_modification_time = None
genome_validity = False
original_genome_id = []

# Burst Engine
burst_publisher = None
burst_activities = {}
burst_timer = None
exit_condition = False
new_genome = False
live_mode_status = 'idle'  # live_mode_status can have modes of idle, learning, testing, tbd
fcl_history = {}
brain_run_id = ""
burst_detection_list = {}
burst_count = 0
fire_candidate_list = {}
previous_fcl = {}
future_fcl = {}
empty_fcl_counter = 0
activity_stats = {}
temp_neuron_list = []
fire_list = []
prunning_candidates = set()


# Stats Collection
neuron_mp_collection_scope = {}
neuron_psp_collection_scope = {}


# Queues
api_queue = None
watchdog_queue = ''
fcl_queue = ''
proximity_queue = ''
fire_queue = {}
comprehension_queue = ''


# cumulative_neighbor_count = 0
time_neuron_update = ''
time_apply_plasticity_ext = ''
plasticity_time_total = None
plasticity_time_total_p1 = None
tester_test_stats = {}


# Stimulation
stimulation_script = {}
stimulation_index = {}

# Brain
current_age = 0
death_flag = False
stats = {}
opu_data = {}
cortical_dimensions = {}
voxel_dict = {}
cortical_types = {}
cortical_defaults = None
plasticity_dict = {}
ipu_list = set()
opu_list = set()
mem_list = set()
core_list = set()
brain = {}
cortical_list = []
intercortical_mapping = []
brain_is_running = False

# Training
shock_scenarios_options = tuple
shock_admin = False
shock_scenarios = set()
pain_flag = False

# Robot
robot_id = None
robot_controller = {}
robot_model = {}
robot_file = None
robot_file_path = None

# Environment
environment_id = None
environment_map_file = None
environment_map_path = None

# ZMQ
godot_listener = None
embodiment_listener = None

# Agents
"""
agent_registry = {
    "agent_ip": {
     ...   
    }

}
"""
agent_registry = {}
