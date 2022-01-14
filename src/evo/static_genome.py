genome = {
    'genome_2.0': {
        "_____10b-_____s-__-__name-t": "smart_car",
        "_____10c-i__bat-cx-__name-t": "battery_opu",
        "_____10c-i__bat-cx-_n_cnt_i": 100,
        "_____10c-i__bat-cx-gd_vis-b": True,
        "_____10c-i__bat-nx-rcordx-i": 100,
        "_____10c-i__bat-nx-rcordy-i": 0,
        "_____10c-i__bat-nx-rcordz-i": 0,
        "_____10c-i__bat-nx-___bbx-i": 1,
        "_____10c-i__bat-nx-___bby-i": 1,
        "_____10c-i__bat-nx-___bbz-i": 10,
        "_____10c-i__bat-cx-__rand-b": True,
        "_____10c-i__bat-cx-synatt-i": 100,
        "_____10c-i__bat-cx-init_s-b": False,
        "_____10c-i__bat-cx-pstcr_-f": -500,
        "_____10c-i__bat-cx-pstcrm-f": 35,
        "_____10c-i__bat-cx-plst_c-f": 0.05,
        "_____10c-i__bat-nx-locr__-t": "rule_5",
        "_____10c-i__bat-nx-locrp_-t": "param_3",
        "_____10c-i__bat-nx-fire_t-f": 1,
        "_____10c-i__bat-nx-refrac-i": 0,
        "_____10c-i__bat-nx-leak_c-f": 10,
        "_____10c-i__bat-nx-c_fr_c-i": 3,
        "_____10c-i__bat-nx-snooze-f": 0,
        "_____10c-i__bat-nx-geo_x0-f": 0,
        "_____10c-i__bat-nx-geo_x1-f": 0,
        "_____10c-i__bat-nx-geo_y0-f": 0,
        "_____10c-i__bat-nx-geo_y1-f": 0,
        "_____10c-i__bat-nx-geo_z0-f": 0,
        "_____10c-i__bat-nx-geo_z1-f": 0
    },
    "max_burst_count": 3,
    "evolution_burst_count": 50,
    'neighbor_locator_rule': {'rule_0': {'param_1': 0, 'param_2': 0},
                              'rule_1': {'param_1': 1, 'param_2': 1},
                              'rule_10': {'param_1': 1, 'param_2': 1},
                              'rule_11': {'param_1': 1, 'param_2': 1},
                              'rule_12': {'param_1': 1, 'param_2': 1},
                              'rule_13': {'param_1': 1, 'param_2': 1},
                              'rule_14': {'param_1': 1, 'param_2': 1},
                              'rule_15': {'param_1': 1, 'param_2': 1},
                              'rule_16': {'param_1': 1, 'param_2': 1},
                              'rule_17': {'param_1': 1, 'param_2': 1},
                              'rule_18': {'param_1': 1, 'param_2': 1},
                              'rule_19': {'param_1': 1, 'param_2': 1},
                              'rule_2': {'param_1': 5, 'param_2': 5, 'param_3': 3},
                              'rule_3': {'param_1': 3, 'param_2': 3},
                              'rule_4': {'param_1': 3, 'param_2': 3},
                              'rule_5': {'param_1': 3,
                                         'param_2': 3,
                                         'param_3': 3,
                                         'param_4': 3,
                                         'param_5': 3,
                                         'param_6': 3,
                                         'param_7': 3},
                              'rule_6': {'param_1': 1, 'param_2': 1},
                              'rule_7': {'param_1': 1, 'param_2': 1},
                              'rule_9': {'param_1': 1, 'param_2': 1}},
    "species": {
      "class": "toy",
      "brand": "gazebo",
      "model": "smart_car"
    },
    "blueprint": {
      "battery_ipu": {
          "orientation_selectivity_pattern": "",
          "location": "",
          "kernel_size": 7,
          "cortical_neuron_count": 100,
          "location_generation_type": "random",
          "synapse_attractivity": 100,
          "init_synapse_needed": False,
          "postsynaptic_current": -500,
          "plasticity_constant": 0.05,
          "postsynaptic_current_max": 35,
          "neighbor_locator_rule_id": "rule_5",
          "neighbor_locator_rule_param_id": "param_3",
          "cortical_mapping_dst": {
              "motor_thalamus": {
                  "neighbor_locator_rule_id": "rule_6",
                  "neighbor_locator_rule_param_id": "param_2"
              }
          },
          "neuron_params": {
              "activation_function_id": "",
              "orientation_selectivity_id": "",
              "depolarization_threshold": 5,
              "firing_threshold": 1,
              "firing_pattern_id": "",
              "refractory_period": 0,
              "axon_avg_length": "",
              "leak_coefficient": 10,
              "axon_avg_connections": "",
              "axon_orientation function": "",
              "consecutive_fire_cnt_max": 3,
              "snooze_length": 0,
              "block_boundaries": [
                  1,
                  1,
                  10
              ],
              "relative_coordinate": [
                100,
                  0,
                  0
              ],
              "visualization": True,
              "geometric_boundaries": {
                  "x": [
                      0,
                      10
                  ],
                  "y": [
                      0,
                      10
                  ],
                  "z": [
                      0,
                      100
                  ]
              }
          }
      }
    }
}
