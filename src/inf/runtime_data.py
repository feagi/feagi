parameters = {}
genome = {}
genome_stats = {}
genome_test_stats = []
brain = {}
cortical_list = []
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

tester_test_stats = {}

# Flags
flag_ready_to_inject_image = False
