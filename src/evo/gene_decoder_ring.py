gene_decoder = {
    "_______b-_____s-__-__name-t": "species_name",
    "_______c-______-cx-__name-t": "cortical_name",
    "_______c-______-cx-_n_cnt-i": "neuron_count",
    "_______c-______-cx-gd_vis-b": "godot_visualization",
    "_______c-______-cx-rcordx-i": "relative_coordinate_x",
    "_______c-______-cx-rcordy-i": "relative_coordinate_y",
    "_______c-______-cx-rcordz-i": "relative_coordinate_z",
    "_______c-______-cx-___bbx-i": "block_boundary_x",
    "_______c-______-cx-___bby-i": "block_boundary_y",
    "_______c-______-cx-___bbz-i": "block_boundary_z",
    "_______c-______-cx-__rand-b": "random_neuron_locations",
    "_______c-______-cx-synatt-i": "synapse_attractivity",
    "_______c-______-cx-init_s-b": "init_synapse_needed",
    "_______c-______-nx-pstcr_-f": "postsynaptic_current",
    "_______c-______-nx-pstcrm-f": "postsynaptic_current_max",
    "_______c-______-nx-plst_c-f": "plasticity_constant",
    "_______c-______-nx-locr__-t": "neighbor_locator_rule_id",
    "_______c-______-nx-locrp_-t": "neighbor_locator_rule_param_id",
    "_______c-______-nx-fire_t-f": "firing_threshold",
    "_______c-______-nx-refrac-i": "refractory_period",
    "_______c-______-nx-leak_c-f": "leak_coefficient",
    "_______c-______-nx-c_fr_c-i": "consecutive_fire_cnt_max",
    "_______c-______-nx-snooze-f": "snooze_length"
}

genome_1_template = {
          "cortical_neuron_count": None,
          "location_generation_type": None,
          "synapse_attractivity": None,
          "init_synapse_needed": None,
          "postsynaptic_current": None,
          "plasticity_constant": None,
          "postsynaptic_current_max": None,
          "neighbor_locator_rule_id": None,
          "neighbor_locator_rule_param_id": None,
          "cortical_mapping_dst": {},
          "neuron_params": {
              "activation_function_id": None,
              "orientation_selectivity_id": None,
              "depolarization_threshold": None,
              "firing_threshold": None,
              "firing_pattern_id": None,
              "refractory_period": None,
              "axon_avg_length": None,
              "leak_coefficient": None,
              "axon_avg_connections": None,
              "axon_orientation function": None,
              "consecutive_fire_cnt_max": None,
              "snooze_length": None,
              "block_boundaries": [
                  None,
                  None,
                  None
              ],
              "relative_coordinate": [
                None,
                  None,
                  None
              ],
              "visualization": None,
              "geometric_boundaries": {
                  "x": [
                      None,
                      None
                  ],
                  "y": [
                      None,
                      None
                  ],
                  "z": [
                      None,
                      None
                  ]
              }
          }
      }

genome_2_to_1 = {
    "cx-_n_cnt-i": "cortical_neuron_count",
    "cx-gd_vis-b": "visualization",
    "nx-rcordx-i": "relative_coordinate",
    "nx-rcordy-i": "relative_coordinate",
    "nx-rcordz-i": "relative_coordinate",
    "nx-___bbx-i": "block_boundaries",
    "nx-___bby-i": "block_boundaries",
    "nx-___bbz-i": "block_boundaries",
    "cx-__rand-b": "location_generation_type",
    "cx-synatt-i": "synapse_attractivity",
    "cx-init_s-b": "init_synapse_needed",
    "cx-pstcr_-f": "postsynaptic_current",
    "cx-pstcrm-f": "postsynaptic_current_max",
    "cx-plst_c-f": "plasticity_constant",
    "nx-locr__-t": "neighbor_locator_rule_id",
    "nx-locrp_-t": "neighbor_locator_rule_param_id",
    "nx-fire_t-f": "firing_threshold",
    "nx-refrac-i": "refractory_period",
    "nx-leak_c-f": "leak_coefficient",
    "nx-c_fr_c-i": "consecutive_fire_cnt_max",
    "nx-snooze-f": "snooze_length",
    "nx-geo_x0-f": "geometric_boundaries",
    "nx-geo_x1-f": "geometric_boundaries",
    "nx-geo_y0-f": "geometric_boundaries",
    "nx-geo_y1-f": "geometric_boundaries",
    "nx-geo_z0-f": "geometric_boundaries",
    "nx-geo_z1-f": "geometric_boundaries"
}

genome_1_to_2 = {
    "cortical_neuron_count": "cx-_n_cnt-i",
    "visualization": "cx-gd_vis-b",
    "relative_coordinate": "cx-rcord_-i",
    "block_boundaries": "cx-___bb_-i",
    "location_generation_type": "cx-__rand-b",
    "synapse_attractivity": "cx-synatt-i",
    "init_synapse_needed": "cx-init_s-b",
    "postsynaptic_current": "nx-pstcr_-f",
    "postsynaptic_current_max": "nx-pstcrm-f",
    "plasticity_constant": "nx-plst_c-f",
    "neighbor_locator_rule_id": "nx-locr__-t",
    "neighbor_locator_rule_param_id": "nx-locrp_-t",
    "firing_threshold": "nx-fire_t-f",
    "refractory_period": "nx-refrac-i",
    "leak_coefficient": "nx-leak_c-f",
    "consecutive_fire_cnt_max": "nx-c_fr_c-i",
    "snooze_length": "nx-snooze-f",
    "geometric_boundaries": "nx-geo___-i"
}
