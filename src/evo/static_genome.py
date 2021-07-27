genome = {
  "firing_patterns": {
      "A": {
          "frequency": "100",
          "magnitude": "80"
      },
      "B": {
          "frequency": "20",
          "magnitude": "100"
      }
  },
  "neighbor_locator_rule": {
      "rule_0": {
          "param_1": 0,
          "param_2": 0
      },
      "rule_1": {
          "param_1": 1,
          "param_2": 1
      },
      "rule_2": {
          "param_1": 5,
          "param_2": 5,
          "param_3": 3
      },
      "rule_3": {
          "param_1": 3,
          "param_2": 3
      },
      "rule_4": {
          "param_1": 3,
          "param_2": 3
      },
      "rule_5": {
          "param_1": 3,
          "param_2": 3,
          "param_3": 3,
          "param_4": 3,
          "param_5": 3,
          "param_6": 3,
          "param_7": 3
      },
      "rule_6": {
          "param_1": 1,
          "param_2": 1
      }
  },
  "orientation_blocks": {
      "1-": [[-1, 0], [0, 0], [1, 0]],
      "1|": [[0, -1], [0, 0], [0, 1]],
      "1/": [[-1, -1], [0, 0], [1, 1]],
      "1\\": [[-1, 1], [0, 0], [1, -1]],
      "2-": [[-2, 0], [-1, 0], [0, 0], [1, 0], [2, 0]],
      "2|": [[0, -2], [0, -1], [0, 0], [0, 1], [0, 2]],
      "2/": [[-2, -2], [-1, -1], [0, 0], [1, 1], [2, 2]],
      "2\\": [[-2, 2], [-1, 1], [0, 0], [1, -1], [2, -2]],
      "3-": [[-3, 0], [-2, 0], [-1, 0], [0, 0], [1, 0], [2, 0], [3, 0]],
      "3|": [[0, -3], [0, -2], [0, -1], [0, 0], [0, 1], [0, 2], [0, 3]],
      "3/": [[-3, -3], [-2, -2], [-1, -1], [0, 0], [1, 1], [2, 2], [3, 3]],
      "3\\": [[-3, 3], [-2, 2], [-1, 1], [0, 0], [1, -1], [2, -2], [3, -3]]
  },
  "IPU_vision_filters": {
      "3": {
          "-": [
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
          "|": [
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
          "": [
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
          "o": [
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
          "/": [
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
          "\\": [
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
      "5": {
          "a": [
              [
                  1,
                  -1,
                  1,
                  -1,
                  1
              ],
              [
                  1,
                  -1,
                  1,
                  -1,
                  1
              ],
              [
                  1,
                  -1,
                  1,
                  -1,
                  1
              ],
              [
                  1,
                  -1,
                  1,
                  -1,
                  1
              ],
              [
                  1,
                  -1,
                  1,
                  -1,
                  1
              ]
          ],
          "b": [
              [
                  0,
                  2,
                  -1,
                  2,
                  0
              ],
              [
                  0,
                  1,
                  -1,
                  1,
                  0
              ],
              [
                  0,
                  -1,
                  -1,
                  -1,
                  0
              ],
              [
                  0,
                  1,
                  -1,
                  1,
                  0
              ],
              [
                  0,
                  2,
                  -1,
                  2,
                  0
              ]
          ],
          "c": [
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
                  1,
                  1,
                  1,
                  1,
                  1
              ]
          ],
          "d": [
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
                  1,
                  1,
                  1,
                  1,
                  1
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
              ]
          ],
          "e": [
              [
                  2,
                  1,
                  -1,
                  1,
                  2
              ],
              [
                  -2,
                  1,
                  -1,
                  1,
                  -2
              ],
              [
                  -2,
                  1,
                  -1,
                  1,
                  -2
              ],
              [
                  -2,
                  1,
                  -1,
                  1,
                  -2
              ],
              [
                  2,
                  1,
                  -1,
                  1,
                  2
              ]
          ],
          "f": [
              [
                  2,
                  -2,
                  -2,
                  -2,
                  2
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
                  1,
                  1,
                  1,
                  1,
                  1
              ],
              [
                  2,
                  -2,
                  -2,
                  -2,
                  2
              ]
          ],
          "g": [
              [
                  -1,
                  1,
                  1,
                  1,
                  -1
              ],
              [
                  -1,
                  1,
                  1,
                  1,
                  -1
              ],
              [
                  1,
                  1,
                  -2,
                  1,
                  1
              ],
              [
                  -1,
                  1,
                  1,
                  1,
                  -1
              ],
              [
                  -1,
                  1,
                  1,
                  1,
                  -1
              ]
          ],
          "h": [
              [
                  1,
                  -1,
                  -1,
                  -1,
                  1
              ],
              [
                  2,
                  -1,
                  -1,
                  -1,
                  2
              ],
              [
                  -1,
                  3,
                  3,
                  3,
                  -1
              ],
              [
                  2,
                  -1,
                  -1,
                  -1,
                  2
              ],
              [
                  1,
                  -1,
                  -1,
                  -1,
                  1
              ]
          ],
          "i": [
              [
                  -2,
                  -1,
                  5,
                  -1,
                  -2
              ],
              [
                  -1,
                  -2,
                  5,
                  -2,
                  -1
              ],
              [
                  -1,
                  -1,
                  5,
                  -1,
                  -1
              ],
              [
                  -1,
                  -2,
                  5,
                  -2,
                  -1
              ],
              [
                  -2,
                  -1,
                  5,
                  -1,
                  -2
              ]
          ],
          "j": [
              [
                  -1,
                  -1,
                  2,
                  -1,
                  -1
              ],
              [
                  -1,
                  -1,
                  4,
                  -1,
                  -1
              ],
              [
                  1,
                  2,
                  -1,
                  2,
                  1
              ],
              [
                  -1,
                  -1,
                  4,
                  -1,
                  -1
              ],
              [
                  -1,
                  -1,
                  3,
                  -1,
                  -1
              ]
          ],
          "-": [
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
                  4,
                  4,
                  4,
                  4,
                  4
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
          "": [
              [
                  -3,
                  -3,
                  -1,
                  -1,
                  -1
              ],
              [
                  -3,
                  -1,
                  -1,
                  -1,
                  -1
              ],
              [
                  -2,
                  -1,
                  -1,
                  -1,
                  2
              ],
              [
                  -1,
                  -1,
                  -1,
                  -1,
                  2
              ],
              [
                  -1,
                  -1,
                  2,
                  2,
                  3
              ]
          ],
          "|": [
              [
                  -1,
                  -1,
                  4,
                  -1,
                  -1
              ],
              [
                  -1,
                  -1,
                  4,
                  -1,
                  -1
              ],
              [
                  -1,
                  -1,
                  4,
                  -1,
                  -1
              ],
              [
                  -1,
                  -1,
                  4,
                  -1,
                  -1
              ],
              [
                  -1,
                  -1,
                  4,
                  -1,
                  -1
              ]
          ],
          "o": [
              [
                  3,
                  2,
                  2,
                  -1,
                  -1
              ],
              [
                  2,
                  -1,
                  -1,
                  -1,
                  -1
              ],
              [
                  2,
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
                  -2
              ],
              [
                  -1,
                  -1,
                  -1,
                  -2,
                  -3
              ]
          ],
          "/": [
              [
                  -1,
                  -1,
                  -1,
                  -1,
                  3
              ],
              [
                  -1,
                  -1,
                  -1,
                  3,
                  -1
              ],
              [
                  -1,
                  -1,
                  3,
                  -1,
                  -1
              ],
              [
                  -1,
                  3,
                  -1,
                  -1,
                  -1
              ],
              [
                  3,
                  -1,
                  -1,
                  -1,
                  -1
              ]
          ],
          "\\": [
              [
                  3,
                  -1,
                  -1,
                  -1,
                  -1
              ],
              [
                  -1,
                  3,
                  -1,
                  -1,
                  -1
              ],
              [
                  -1,
                  -1,
                  3,
                  -1,
                  -1
              ],
              [
                  -1,
                  -1,
                  -1,
                  3,
                  -1
              ],
              [
                  -1,
                  -1,
                  -1,
                  -1,
                  3
              ]
          ]
      },
      "7": {
          "-": [
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
          "": [
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
          "I": [
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
          "i": [
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
          "o": [
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
          "/": [
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
          "\\": [
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
  "location_tolerance": 2,
  "image_color_intensity_tolerance": 100,
  "max_burst_count": 3,
  "evolution_burst_count": 50,
  "performance_stats": {
      "mnist_view_cnt": 0,
      "mnist_correct_detection_cnt": 0
  },
  "blueprint": {
      "thalamus_vision_1": {
          "growth_path": "",
          "direction_sensitivity": "/",
          "group_id": "thalamus",
          "sub_group_id": "t_vision",
          "plot_index": 1,
          "total_layer_count": 2,
          "orientation_selectivity_pattern": "",
          "location": "",
          "kernel_size": 5,
          "cortical_neuron_count": 200,
          "location_generation_type": "random",
          "synapse_attractivity": 100,
          "init_synapse_needed": False,
          "postsynaptic_current": 20,
          "plasticity_constant": 0,
          "postsynaptic_current_max": 1000,
          "neighbor_locator_rule_id": "rule_1",
          "neighbor_locator_rule_param_id": "param_1",
          "cortical_mapping_dst": {
              "vision_v1-1": {
                  "neighbor_locator_rule_id": "rule_1",
                  "neighbor_locator_rule_param_id": "param_1"
              }
          },
          "neuron_params": {
              "activation_function_id": "",
              "depolarization_threshold": 1.633575495825,
              "orientation_selectivity_id": "",
              "firing_threshold": 2.89235403072,
              "firing_pattern_id": "",
              "refractory_period": 0,
              "axon_avg_length": "",
              "leak_coefficient": 10,
              "axon_avg_connections": "",
              "axon_orientation function": "",
              "consecutive_fire_cnt_max": 0,
              "snooze_length": 1.659231437448,
              "block_boundaries": [
                  28,
                  28,
                  1
              ],
              "geometric_boundaries": {
                  "x": [
                      0,
                      32
                  ],
                  "y": [
                      0,
                      32
                  ],
                  "z": [
                      0,
                      10
                  ]
              }
          }
      },
      "thalamus_vision_2": {
          "growth_path": "",
          "direction_sensitivity": "/",
          "group_id": "thalamus",
          "sub_group_id": "t_vision",
          "plot_index": 1,
          "total_layer_count": 2,
          "orientation_selectivity_pattern": "",
          "location": "",
          "kernel_size": 5,
          "cortical_neuron_count": 200,
          "location_generation_type": "random",
          "synapse_attractivity": 100,
          "init_synapse_needed": False,
          "postsynaptic_current": 20,
          "plasticity_constant": 0,
          "postsynaptic_current_max": 1000,
          "neighbor_locator_rule_id": "rule_1",
          "neighbor_locator_rule_param_id": "param_1",
          "cortical_mapping_dst": {
              "vision_v1-2": {
                  "neighbor_locator_rule_id": "rule_1",
                  "neighbor_locator_rule_param_id": "param_1"
              }
          },
          "neuron_params": {
              "activation_function_id": "",
              "depolarization_threshold": 1.633575495825,
              "orientation_selectivity_id": "",
              "firing_threshold": 2.89235403072,
              "firing_pattern_id": "",
              "refractory_period": 0,
              "axon_avg_length": "",
              "leak_coefficient": 10,
              "axon_avg_connections": "",
              "axon_orientation function": "",
              "consecutive_fire_cnt_max": 0,
              "snooze_length": 1.659231437448,
              "block_boundaries": [
                  28,
                  28,
                  1
              ],
              "geometric_boundaries": {
                  "x": [
                      0,
                      32
                  ],
                  "y": [
                      0,
                      32
                  ],
                  "z": [
                      0,
                      10
                  ]
              }
          }
      },
      "thalamus_vision_3": {
          "growth_path": "",
          "direction_sensitivity": "/",
          "group_id": "thalamus",
          "sub_group_id": "t_vision",
          "plot_index": 1,
          "total_layer_count": 2,
          "orientation_selectivity_pattern": "",
          "location": "",
          "kernel_size": 5,
          "cortical_neuron_count": 200,
          "location_generation_type": "random",
          "synapse_attractivity": 100,
          "init_synapse_needed": False,
          "postsynaptic_current": 20,
          "plasticity_constant": 0,
          "postsynaptic_current_max": 1000,
          "neighbor_locator_rule_id": "rule_1",
          "neighbor_locator_rule_param_id": "param_1",
          "cortical_mapping_dst": {
              "vision_v1-3": {
                  "neighbor_locator_rule_id": "rule_1",
                  "neighbor_locator_rule_param_id": "param_1"
              }
          },
          "neuron_params": {
              "activation_function_id": "",
              "depolarization_threshold": 1.633575495825,
              "orientation_selectivity_id": "",
              "firing_threshold": 2.89235403072,
              "firing_pattern_id": "",
              "refractory_period": 0,
              "axon_avg_length": "",
              "leak_coefficient": 10,
              "axon_avg_connections": "",
              "axon_orientation function": "",
              "consecutive_fire_cnt_max": 0,
              "snooze_length": 1.659231437448,
              "block_boundaries": [
                  28,
                  28,
                  1
              ],
              "geometric_boundaries": {
                  "x": [
                      0,
                      32
                  ],
                  "y": [
                      0,
                      32
                  ],
                  "z": [
                      0,
                      10
                  ]
              }
          }
      },
      "thalamus_vision_4": {
          "growth_path": "",
          "direction_sensitivity": "/",
          "group_id": "thalamus",
          "sub_group_id": "t_vision",
          "plot_index": 1,
          "total_layer_count": 2,
          "orientation_selectivity_pattern": "",
          "location": "",
          "kernel_size": 5,
          "cortical_neuron_count": 200,
          "location_generation_type": "random",
          "synapse_attractivity": 100,
          "init_synapse_needed": False,
          "postsynaptic_current": 20,
          "plasticity_constant": 0,
          "postsynaptic_current_max": 1000,
          "neighbor_locator_rule_id": "rule_1",
          "neighbor_locator_rule_param_id": "param_1",
          "cortical_mapping_dst": {
              "vision_v1-4": {
                  "neighbor_locator_rule_id": "rule_1",
                  "neighbor_locator_rule_param_id": "param_1"
              }
          },
          "neuron_params": {
              "activation_function_id": "",
              "depolarization_threshold": 1.633575495825,
              "orientation_selectivity_id": "",
              "firing_threshold": 2.89235403072,
              "firing_pattern_id": "",
              "refractory_period": 0,
              "axon_avg_length": "",
              "leak_coefficient": 10,
              "axon_avg_connections": "",
              "axon_orientation function": "",
              "consecutive_fire_cnt_max": 0,
              "snooze_length": 1.659231437448,
              "block_boundaries": [
                  28,
                  28,
                  1
              ],
              "geometric_boundaries": {
                  "x": [
                      0,
                      32
                  ],
                  "y": [
                      0,
                      32
                  ],
                  "z": [
                      0,
                      10
                  ]
              }
          }
      },
      "thalamus_utf": {
          "growth_path": "",
          "direction_sensitivity": "/",
          "group_id": "thalamus",
          "sub_group_id": "utf",
          "plot_index": 1,
          "layer_index": 1,
          "total_layer_count": 7,
          "orientation_selectivity_pattern": "",
          "location": "",
          "kernel_size": 5,
          "cortical_neuron_count": 210,
          "location_generation_type": "sequential",
          "synapse_attractivity": 100,
          "init_synapse_needed": False,
          "postsynaptic_current": 20,
          "plasticity_constant": 0,
          "postsynaptic_current_max": 1000,
          "neighbor_locator_rule_id": "rule_1",
          "neighbor_locator_rule_param_id": "param_1",
          "cortical_mapping_dst": {
              "utf8": {
                  "neighbor_locator_rule_id": "rule_0",
                  "neighbor_locator_rule_param_id": "param_1"
              }
          },
          "neuron_params": {
              "activation_function_id": "",
              "depolarization_threshold": 1.633575495825,
              "orientation_selectivity_id": "",
              "firing_threshold": 2.89235403072,
              "firing_pattern_id": "",
              "refractory_period": 0,
              "axon_avg_length": "",
              "leak_coefficient": 10,
              "axon_avg_connections": "",
              "axon_orientation function": "",
              "consecutive_fire_cnt_max": 0,
              "snooze_length": 1.659231437448,
              "block_boundaries": [
                  28,
                  28,
                  1
              ],
              "geometric_boundaries": {
                  "x": [
                      0,
                      32
                  ],
                  "y": [
                      0,
                      32
                  ],
                  "z": [
                      0,
                      10
                  ]
              }
          }
      },
      "vision_v1-1": {
          "growth_path": "",
          "direction_sensitivity": "/",
          "group_id": "vision",
          "sub_group_id": "vision_v1",
          "plot_index": 1,
          "layer_index": 1,
          "total_layer_count": 7,
          "orientation_selectivity_pattern": "",
          "location": "",
          "kernel_size": 5,
          "cortical_neuron_count": 100,
          "location_generation_type": "random",
          "synapse_attractivity": 100,
          "init_synapse_needed": False,
          "postsynaptic_current": 20,
          "plasticity_constant": 0,
          "postsynaptic_current_max": 1000,
          "neighbor_locator_rule_id": "rule_1",
          "neighbor_locator_rule_param_id": "param_1",
          "cortical_mapping_dst": {
              "vision_v2": {
                  "neighbor_locator_rule_id": "rule_5",
                  "neighbor_locator_rule_param_id": "param_1"
              }
          },
          "neuron_params": {
              "activation_function_id": "",
              "depolarization_threshold": 1.633575495825,
              "orientation_selectivity_id": "",
              "firing_threshold": 2.89235403072,
              "firing_pattern_id": "",
              "refractory_period": 0,
              "axon_avg_length": "",
              "leak_coefficient": 10,
              "axon_avg_connections": "",
              "axon_orientation function": "",
              "consecutive_fire_cnt_max": 0,
              "snooze_length": 1.659231437448,
              "block_boundaries": [
                  28,
                  28,
                  1
              ],
              "geometric_boundaries": {
                  "x": [
                      0,
                      32
                  ],
                  "y": [
                      0,
                      32
                  ],
                  "z": [
                      0,
                      10
                  ]
              }
          }
      },
      "vision_v1-2": {
          "growth_path": "",
          "group_id": "vision",
          "sub_group_id": "vision_v1",
          "plot_index": 4,
          "layer_index": 2,
          "total_layer_count": 7,
          "direction_sensitivity": "\\",
          "orientation_selectivity_pattern": "",
          "location": "",
          "kernel_size": 5,
          "cortical_neuron_count": 100,
          "location_generation_type": "random",
          "synapse_attractivity": 100,
          "init_synapse_needed": False,
          "postsynaptic_current": 20,
          "plasticity_constant": 0,
          "postsynaptic_current_max": 1000,
          "neighbor_locator_rule_id": "rule_1",
          "neighbor_locator_rule_param_id": "param_1",
          "cortical_mapping_dst": {
              "vision_v2": {
                  "neighbor_locator_rule_id": "rule_5",
                  "neighbor_locator_rule_param_id": "param_2"
              }
          },
          "neuron_params": {
              "activation_function_id": "",
              "depolarization_threshold": 1.574682375,
              "orientation_selectivity_id": "",
              "firing_threshold": 3.4898094,
              "firing_pattern_id": "",
              "refractory_period": 0,
              "axon_avg_length": "",
              "leak_coefficient": 10,
              "axon_avg_connections": "",
              "axon_orientation function": "",
              "consecutive_fire_cnt_max": 0,
              "snooze_length": 1.83787266,
              "block_boundaries": [
                  28,
                  28,
                  1
              ],
              "geometric_boundaries": {
                  "x": [
                      0,
                      29
                  ],
                  "y": [
                      0,
                      29
                  ],
                  "z": [
                      0,
                      10
                  ]
              }
          }
      },
      "vision_v1-3": {
          "growth_path": "",
          "group_id": "vision",
          "sub_group_id": "vision_v1",
          "plot_index": 7,
          "layer_index": 3,
          "total_layer_count": 7,
          "direction_sensitivity": "|",
          "orientation_selectivity_pattern": "",
          "location": "",
          "kernel_size": 5,
          "cortical_neuron_count": 100,
          "location_generation_type": "random",
          "synapse_attractivity": 100,
          "init_synapse_needed": False,
          "postsynaptic_current": 20,
          "plasticity_constant": 0,
          "postsynaptic_current_max": 1000,
          "neighbor_locator_rule_id": "rule_1",
          "neighbor_locator_rule_param_id": "param_1",
          "cortical_mapping_dst": {
              "vision_v2": {
                  "neighbor_locator_rule_id": "rule_5",
                  "neighbor_locator_rule_param_id": "param_3"
              }
          },
          "neuron_params": {
              "activation_function_id": "",
              "depolarization_threshold": 1.574682375,
              "orientation_selectivity_id": "",
              "firing_threshold": 3.4898094,
              "firing_pattern_id": "",
              "refractory_period": 0,
              "axon_avg_length": "",
              "leak_coefficient": 10,
              "axon_avg_connections": "",
              "axon_orientation function": "",
              "consecutive_fire_cnt_max": 0,
              "snooze_length": 1.83787266,
              "block_boundaries": [
                  28,
                  28,
                  1
              ],
              "geometric_boundaries": {
                  "x": [
                      0,
                      29
                  ],
                  "y": [
                      0,
                      29
                  ],
                  "z": [
                      0,
                      10
                  ]
              }
          }
      },
      "vision_v1-4": {
          "growth_path": "",
          "group_id": "vision",
          "sub_group_id": "vision_v1",
          "plot_index": 10,
          "layer_index": 4,
          "total_layer_count": 7,
          "direction_sensitivity": "-",
          "orientation_selectivity_pattern": "",
          "location": "",
          "kernel_size": 5,
          "cortical_neuron_count": 100,
          "location_generation_type": "random",
          "synapse_attractivity": 100,
          "init_synapse_needed": False,
          "postsynaptic_current": 20,
          "plasticity_constant": 0,
          "postsynaptic_current_max": 1100,
          "neighbor_locator_rule_id": "rule_1",
          "neighbor_locator_rule_param_id": "param_1",
          "cortical_mapping_dst": {
              "vision_v2": {
                  "neighbor_locator_rule_id": "rule_5",
                  "neighbor_locator_rule_param_id": "param_4"
              }
          },
          "neuron_params": {
              "activation_function_id": "",
              "depolarization_threshold": 1.574682375,
              "orientation_selectivity_id": "",
              "firing_threshold": 3.4898094,
              "firing_pattern_id": "",
              "refractory_period": 0,
              "axon_avg_length": "",
              "leak_coefficient": 10,
              "axon_avg_connections": "",
              "axon_orientation function": "",
              "consecutive_fire_cnt_max": 0,
              "snooze_length": 1.83787266,
              "block_boundaries": [
                  28,
                  28,
                  1
              ],
              "geometric_boundaries": {
                  "x": [
                      0,
                      29
                  ],
                  "y": [
                      0,
                      29
                  ],
                  "z": [
                      0,
                      10
                  ]
              }
          }
      },
      "vision_v1-5": {
          "growth_path": "",
          "group_id": "vision",
          "sub_group_id": "vision_v1",
          "plot_index": 13,
          "layer_index": 5,
          "total_layer_count": 7,
          "direction_sensitivity": "I",
          "orientation_selectivity_pattern": "",
          "location": "",
          "kernel_size": 3,
          "cortical_neuron_count": 200,
          "location_generation_type": "random",
          "synapse_attractivity": 100,
          "init_synapse_needed": False,
          "postsynaptic_current": 20,
          "plasticity_constant": 0,
          "postsynaptic_current_max": 1000,
          "neighbor_locator_rule_id": "rule_1",
          "neighbor_locator_rule_param_id": "param_1",
          "cortical_mapping_dst": {
              "vision_v2": {
                  "neighbor_locator_rule_id": "rule_5",
                  "neighbor_locator_rule_param_id": "param_5"
              }
          },
          "neuron_params": {
              "activation_function_id": "",
              "depolarization_threshold": 1.574682375,
              "orientation_selectivity_id": "",
              "firing_threshold": 3.4898094,
              "firing_pattern_id": "",
              "refractory_period": 0,
              "axon_avg_length": "",
              "leak_coefficient": 10,
              "axon_avg_connections": "",
              "axon_orientation function": "",
              "consecutive_fire_cnt_max": 0,
              "snooze_length": 1.83787266,
              "block_boundaries": [
                  28,
                  28,
                  1
              ],
              "geometric_boundaries": {
                  "x": [
                      0,
                      29
                  ],
                  "y": [
                      0,
                      29
                  ],
                  "z": [
                      0,
                      10
                  ]
              }
          }
      },
      "vision_v1-6": {
          "growth_path": "",
          "group_id": "vision",
          "sub_group_id": "vision_v1",
          "plot_index": 16,
          "layer_index": 6,
          "total_layer_count": 7,
          "direction_sensitivity": "o",
          "orientation_selectivity_pattern": "",
          "location": "",
          "kernel_size": 3,
          "cortical_neuron_count": 200,
          "location_generation_type": "random",
          "synapse_attractivity": 100,
          "init_synapse_needed": False,
          "postsynaptic_current": 20,
          "plasticity_constant": 0,
          "postsynaptic_current_max": 1000,
          "neighbor_locator_rule_id": "rule_1",
          "neighbor_locator_rule_param_id": "param_1",
          "cortical_mapping_dst": {
              "vision_v2": {
                  "neighbor_locator_rule_id": "rule_5",
                  "neighbor_locator_rule_param_id": "param_6"
              }
          },
          "neuron_params": {
              "activation_function_id": "",
              "depolarization_threshold": 1.574682375,
              "orientation_selectivity_id": "",
              "firing_threshold": 3.4898094,
              "firing_pattern_id": "",
              "refractory_period": 0,
              "axon_avg_length": "",
              "leak_coefficient": 10,
              "axon_avg_connections": "",
              "axon_orientation function": "",
              "consecutive_fire_cnt_max": 0,
              "snooze_length": 1.83787266,
              "block_boundaries": [
                  28,
                  28,
                  1
              ],
              "geometric_boundaries": {
                  "x": [
                      0,
                      29
                  ],
                  "y": [
                      0,
                      29
                  ],
                  "z": [
                      0,
                      10
                  ]
              }
          }
      },
      "vision_v1-7": {
          "growth_path": "",
          "group_id": "vision",
          "sub_group_id": "vision_v1",
          "plot_index": 19,
          "layer_index": 7,
          "total_layer_count": 7,
          "direction_sensitivity": " ",
          "orientation_selectivity_pattern": "",
          "location": "",
          "kernel_size": 5,
          "cortical_neuron_count": 200,
          "location_generation_type": "random",
          "synapse_attractivity": 100,
          "init_synapse_needed": False,
          "postsynaptic_current": 20,
          "plasticity_constant": 0,
          "postsynaptic_current_max": 1000,
          "neighbor_locator_rule_id": "rule_1",
          "neighbor_locator_rule_param_id": "param_1",
          "cortical_mapping_dst": {
              "vision_v2": {
                  "neighbor_locator_rule_id": "rule_5",
                  "neighbor_locator_rule_param_id": "param_7"
              }
          },
          "neuron_params": {
              "activation_function_id": "",
              "depolarization_threshold": 1.574682375,
              "orientation_selectivity_id": "",
              "firing_threshold": 3.4898094,
              "firing_pattern_id": "",
              "refractory_period": 0,
              "axon_avg_length": "",
              "leak_coefficient": 10,
              "axon_avg_connections": "",
              "axon_orientation function": "",
              "consecutive_fire_cnt_max": 0,
              "snooze_length": 1.83787266,
              "block_boundaries": [
                  28,
                  28,
                  1
              ],
              "geometric_boundaries": {
                  "x": [
                      0,
                      29
                  ],
                  "y": [
                      0,
                      29
                  ],
                  "z": [
                      0,
                      10
                  ]
              }
          }
      },
      "vision_v1-8": {
          "growth_path": "",
          "group_id": "vision",
          "sub_group_id": "vision_v1",
          "plot_index": 19,
          "layer_index": 7,
          "total_layer_count": 7,
          "direction_sensitivity": "a",
          "orientation_selectivity_pattern": "",
          "location": "",
          "kernel_size": 5,
          "cortical_neuron_count": 200,
          "location_generation_type": "random",
          "synapse_attractivity": 100,
          "init_synapse_needed": False,
          "postsynaptic_current": 20,
          "plasticity_constant": 0,
          "postsynaptic_current_max": 1000,
          "neighbor_locator_rule_id": "rule_1",
          "neighbor_locator_rule_param_id": "param_1",
          "cortical_mapping_dst": {
              "vision_v2": {
                  "neighbor_locator_rule_id": "rule_5",
                  "neighbor_locator_rule_param_id": "param_7"
              }
          },
          "neuron_params": {
              "activation_function_id": "",
              "depolarization_threshold": 1.574682375,
              "orientation_selectivity_id": "",
              "firing_threshold": 3.4898094,
              "firing_pattern_id": "",
              "refractory_period": 0,
              "axon_avg_length": "",
              "leak_coefficient": 10,
              "axon_avg_connections": "",
              "axon_orientation function": "",
              "consecutive_fire_cnt_max": 0,
              "snooze_length": 1.83787266,
              "block_boundaries": [
                  28,
                  28,
                  1
              ],
              "geometric_boundaries": {
                  "x": [
                      0,
                      29
                  ],
                  "y": [
                      0,
                      29
                  ],
                  "z": [
                      0,
                      10
                  ]
              }
          }
      },
      "vision_v1-9": {
          "growth_path": "",
          "group_id": "vision",
          "sub_group_id": "vision_v1",
          "plot_index": 19,
          "layer_index": 7,
          "total_layer_count": 7,
          "direction_sensitivity": "b",
          "orientation_selectivity_pattern": "",
          "location": "",
          "kernel_size": 5,
          "cortical_neuron_count": 200,
          "location_generation_type": "random",
          "synapse_attractivity": 100,
          "init_synapse_needed": False,
          "postsynaptic_current": 20,
          "plasticity_constant": 0,
          "postsynaptic_current_max": 1000,
          "neighbor_locator_rule_id": "rule_1",
          "neighbor_locator_rule_param_id": "param_1",
          "cortical_mapping_dst": {
              "vision_v2": {
                  "neighbor_locator_rule_id": "rule_5",
                  "neighbor_locator_rule_param_id": "param_7"
              }
          },
          "neuron_params": {
              "activation_function_id": "",
              "depolarization_threshold": 1.574682375,
              "orientation_selectivity_id": "",
              "firing_threshold": 3.4898094,
              "firing_pattern_id": "",
              "refractory_period": 0,
              "axon_avg_length": "",
              "leak_coefficient": 10,
              "axon_avg_connections": "",
              "axon_orientation function": "",
              "consecutive_fire_cnt_max": 0,
              "snooze_length": 1.83787266,
              "block_boundaries": [
                  28,
                  28,
                  1
              ],
              "geometric_boundaries": {
                  "x": [
                      0,
                      29
                  ],
                  "y": [
                      0,
                      29
                  ],
                  "z": [
                      0,
                      10
                  ]
              }
          }
      },
      "vision_v1-10": {
          "growth_path": "",
          "group_id": "vision",
          "sub_group_id": "vision_v1",
          "plot_index": 19,
          "layer_index": 7,
          "total_layer_count": 7,
          "direction_sensitivity": "c",
          "orientation_selectivity_pattern": "",
          "location": "",
          "kernel_size": 5,
          "cortical_neuron_count": 200,
          "location_generation_type": "random",
          "synapse_attractivity": 100,
          "init_synapse_needed": False,
          "postsynaptic_current": 20,
          "plasticity_constant": 0,
          "postsynaptic_current_max": 1000,
          "neighbor_locator_rule_id": "rule_1",
          "neighbor_locator_rule_param_id": "param_1",
          "cortical_mapping_dst": {
              "vision_v2": {
                  "neighbor_locator_rule_id": "rule_5",
                  "neighbor_locator_rule_param_id": "param_7"
              }
          },
          "neuron_params": {
              "activation_function_id": "",
              "depolarization_threshold": 1.574682375,
              "orientation_selectivity_id": "",
              "firing_threshold": 3.4898094,
              "firing_pattern_id": "",
              "refractory_period": 0,
              "axon_avg_length": "",
              "leak_coefficient": 10,
              "axon_avg_connections": "",
              "axon_orientation function": "",
              "consecutive_fire_cnt_max": 0,
              "snooze_length": 1.83787266,
              "block_boundaries": [
                  28,
                  28,
                  1
              ],
              "geometric_boundaries": {
                  "x": [
                      0,
                      29
                  ],
                  "y": [
                      0,
                      29
                  ],
                  "z": [
                      0,
                      10
                  ]
              }
          }
      },
      "vision_v1-11": {
          "growth_path": "",
          "group_id": "vision",
          "sub_group_id": "vision_v1",
          "plot_index": 19,
          "layer_index": 7,
          "total_layer_count": 7,
          "direction_sensitivity": "d",
          "orientation_selectivity_pattern": "",
          "location": "",
          "kernel_size": 5,
          "cortical_neuron_count": 200,
          "location_generation_type": "random",
          "synapse_attractivity": 100,
          "init_synapse_needed": False,
          "postsynaptic_current": 20,
          "plasticity_constant": 0,
          "postsynaptic_current_max": 1000,
          "neighbor_locator_rule_id": "rule_1",
          "neighbor_locator_rule_param_id": "param_1",
          "cortical_mapping_dst": {
              "vision_v2": {
                  "neighbor_locator_rule_id": "rule_5",
                  "neighbor_locator_rule_param_id": "param_7"
              }
          },
          "neuron_params": {
              "activation_function_id": "",
              "depolarization_threshold": 1.574682375,
              "orientation_selectivity_id": "",
              "firing_threshold": 3.4898094,
              "firing_pattern_id": "",
              "refractory_period": 0,
              "axon_avg_length": "",
              "leak_coefficient": 10,
              "axon_avg_connections": "",
              "axon_orientation function": "",
              "consecutive_fire_cnt_max": 0,
              "snooze_length": 1.83787266,
              "block_boundaries": [
                  28,
                  28,
                  1
              ],
              "geometric_boundaries": {
                  "x": [
                      0,
                      29
                  ],
                  "y": [
                      0,
                      29
                  ],
                  "z": [
                      0,
                      10
                  ]
              }
          }
      },
      "vision_v1-12": {
          "growth_path": "",
          "group_id": "vision",
          "sub_group_id": "vision_v1",
          "plot_index": 19,
          "layer_index": 7,
          "total_layer_count": 7,
          "direction_sensitivity": "e",
          "orientation_selectivity_pattern": "",
          "location": "",
          "kernel_size": 5,
          "cortical_neuron_count": 200,
          "location_generation_type": "random",
          "synapse_attractivity": 100,
          "init_synapse_needed": False,
          "postsynaptic_current": 20,
          "plasticity_constant": 0,
          "postsynaptic_current_max": 1000,
          "neighbor_locator_rule_id": "rule_1",
          "neighbor_locator_rule_param_id": "param_1",
          "cortical_mapping_dst": {
              "vision_v2": {
                  "neighbor_locator_rule_id": "rule_5",
                  "neighbor_locator_rule_param_id": "param_7"
              }
          },
          "neuron_params": {
              "activation_function_id": "",
              "depolarization_threshold": 1.574682375,
              "orientation_selectivity_id": "",
              "firing_threshold": 3.4898094,
              "firing_pattern_id": "",
              "refractory_period": 0,
              "axon_avg_length": "",
              "leak_coefficient": 10,
              "axon_avg_connections": "",
              "axon_orientation function": "",
              "consecutive_fire_cnt_max": 0,
              "snooze_length": 1.83787266,
              "block_boundaries": [
                  28,
                  28,
                  1
              ],
              "geometric_boundaries": {
                  "x": [
                      0,
                      29
                  ],
                  "y": [
                      0,
                      29
                  ],
                  "z": [
                      0,
                      10
                  ]
              }
          }
      },
      "vision_v1-13": {
          "growth_path": "",
          "group_id": "vision",
          "sub_group_id": "vision_v1",
          "plot_index": 19,
          "layer_index": 7,
          "total_layer_count": 7,
          "direction_sensitivity": "f",
          "orientation_selectivity_pattern": "",
          "location": "",
          "kernel_size": 5,
          "cortical_neuron_count": 200,
          "location_generation_type": "random",
          "synapse_attractivity": 100,
          "init_synapse_needed": False,
          "postsynaptic_current": 20,
          "plasticity_constant": 0,
          "postsynaptic_current_max": 1000,
          "neighbor_locator_rule_id": "rule_1",
          "neighbor_locator_rule_param_id": "param_1",
          "cortical_mapping_dst": {
              "vision_v2": {
                  "neighbor_locator_rule_id": "rule_5",
                  "neighbor_locator_rule_param_id": "param_7"
              }
          },
          "neuron_params": {
              "activation_function_id": "",
              "depolarization_threshold": 1.574682375,
              "orientation_selectivity_id": "",
              "firing_threshold": 3.4898094,
              "firing_pattern_id": "",
              "refractory_period": 0,
              "axon_avg_length": "",
              "leak_coefficient": 10,
              "axon_avg_connections": "",
              "axon_orientation function": "",
              "consecutive_fire_cnt_max": 0,
              "snooze_length": 1.83787266,
              "block_boundaries": [
                  28,
                  28,
                  1
              ],
              "geometric_boundaries": {
                  "x": [
                      0,
                      29
                  ],
                  "y": [
                      0,
                      29
                  ],
                  "z": [
                      0,
                      10
                  ]
              }
          }
      },
      "vision_v1-14": {
          "growth_path": "",
          "group_id": "vision",
          "sub_group_id": "vision_v1",
          "plot_index": 19,
          "layer_index": 7,
          "total_layer_count": 7,
          "direction_sensitivity": "g",
          "orientation_selectivity_pattern": "",
          "location": "",
          "kernel_size": 5,
          "cortical_neuron_count": 200,
          "location_generation_type": "random",
          "synapse_attractivity": 100,
          "init_synapse_needed": False,
          "postsynaptic_current": 20,
          "plasticity_constant": 0,
          "postsynaptic_current_max": 1000,
          "neighbor_locator_rule_id": "rule_1",
          "neighbor_locator_rule_param_id": "param_1",
          "cortical_mapping_dst": {
              "vision_v2": {
                  "neighbor_locator_rule_id": "rule_5",
                  "neighbor_locator_rule_param_id": "param_7"
              }
          },
          "neuron_params": {
              "activation_function_id": "",
              "depolarization_threshold": 1.574682375,
              "orientation_selectivity_id": "",
              "firing_threshold": 3.4898094,
              "firing_pattern_id": "",
              "refractory_period": 0,
              "axon_avg_length": "",
              "leak_coefficient": 10,
              "axon_avg_connections": "",
              "axon_orientation function": "",
              "consecutive_fire_cnt_max": 0,
              "snooze_length": 1.83787266,
              "block_boundaries": [
                  28,
                  28,
                  1
              ],
              "geometric_boundaries": {
                  "x": [
                      0,
                      29
                  ],
                  "y": [
                      0,
                      29
                  ],
                  "z": [
                      0,
                      10
                  ]
              }
          }
      },
      "vision_v1-15": {
          "growth_path": "",
          "group_id": "vision",
          "sub_group_id": "vision_v1",
          "plot_index": 19,
          "layer_index": 7,
          "total_layer_count": 7,
          "direction_sensitivity": "h",
          "orientation_selectivity_pattern": "",
          "location": "",
          "kernel_size": 5,
          "cortical_neuron_count": 200,
          "location_generation_type": "random",
          "synapse_attractivity": 100,
          "init_synapse_needed": False,
          "postsynaptic_current": 20,
          "plasticity_constant": 0,
          "postsynaptic_current_max": 1000,
          "neighbor_locator_rule_id": "rule_1",
          "neighbor_locator_rule_param_id": "param_1",
          "cortical_mapping_dst": {
              "vision_v2": {
                  "neighbor_locator_rule_id": "rule_5",
                  "neighbor_locator_rule_param_id": "param_7"
              }
          },
          "neuron_params": {
              "activation_function_id": "",
              "depolarization_threshold": 1.574682375,
              "orientation_selectivity_id": "",
              "firing_threshold": 3.4898094,
              "firing_pattern_id": "",
              "refractory_period": 0,
              "axon_avg_length": "",
              "leak_coefficient": 10,
              "axon_avg_connections": "",
              "axon_orientation function": "",
              "consecutive_fire_cnt_max": 0,
              "snooze_length": 1.83787266,
              "block_boundaries": [
                  28,
                  28,
                  1
              ],
              "geometric_boundaries": {
                  "x": [
                      0,
                      29
                  ],
                  "y": [
                      0,
                      29
                  ],
                  "z": [
                      0,
                      10
                  ]
              }
          }
      },
      "vision_v1-16": {
          "growth_path": "",
          "group_id": "vision",
          "sub_group_id": "vision_v1",
          "plot_index": 19,
          "layer_index": 7,
          "total_layer_count": 7,
          "direction_sensitivity": "i",
          "orientation_selectivity_pattern": "",
          "location": "",
          "kernel_size": 5,
          "cortical_neuron_count": 200,
          "location_generation_type": "random",
          "synapse_attractivity": 100,
          "init_synapse_needed": False,
          "postsynaptic_current": 20,
          "plasticity_constant": 0,
          "postsynaptic_current_max": 1000,
          "neighbor_locator_rule_id": "rule_1",
          "neighbor_locator_rule_param_id": "param_1",
          "cortical_mapping_dst": {
              "vision_v2": {
                  "neighbor_locator_rule_id": "rule_5",
                  "neighbor_locator_rule_param_id": "param_7"
              }
          },
          "neuron_params": {
              "activation_function_id": "",
              "depolarization_threshold": 1.574682375,
              "orientation_selectivity_id": "",
              "firing_threshold": 3.4898094,
              "firing_pattern_id": "",
              "refractory_period": 0,
              "axon_avg_length": "",
              "leak_coefficient": 10,
              "axon_avg_connections": "",
              "axon_orientation function": "",
              "consecutive_fire_cnt_max": 0,
              "snooze_length": 1.83787266,
              "block_boundaries": [
                  28,
                  28,
                  1
              ],
              "geometric_boundaries": {
                  "x": [
                      0,
                      29
                  ],
                  "y": [
                      0,
                      29
                  ],
                  "z": [
                      0,
                      10
                  ]
              }
          }
      },
      "vision_v1-17": {
          "growth_path": "",
          "group_id": "vision",
          "sub_group_id": "vision_v1",
          "plot_index": 19,
          "layer_index": 7,
          "total_layer_count": 7,
          "direction_sensitivity": "j",
          "orientation_selectivity_pattern": "",
          "location": "",
          "kernel_size": 5,
          "cortical_neuron_count": 200,
          "location_generation_type": "random",
          "synapse_attractivity": 100,
          "init_synapse_needed": False,
          "postsynaptic_current": 20,
          "plasticity_constant": 0,
          "postsynaptic_current_max": 1000,
          "neighbor_locator_rule_id": "rule_1",
          "neighbor_locator_rule_param_id": "param_1",
          "cortical_mapping_dst": {
              "vision_v2": {
                  "neighbor_locator_rule_id": "rule_5",
                  "neighbor_locator_rule_param_id": "param_7"
              }
          },
          "neuron_params": {
              "activation_function_id": "",
              "depolarization_threshold": 1.574682375,
              "orientation_selectivity_id": "",
              "firing_threshold": 3.4898094,
              "firing_pattern_id": "",
              "refractory_period": 0,
              "axon_avg_length": "",
              "leak_coefficient": 10,
              "axon_avg_connections": "",
              "axon_orientation function": "",
              "consecutive_fire_cnt_max": 0,
              "snooze_length": 1.83787266,
              "block_boundaries": [
                  28,
                  28,
                  1
              ],
              "geometric_boundaries": {
                  "x": [
                      0,
                      29
                  ],
                  "y": [
                      0,
                      29
                  ],
                  "z": [
                      0,
                      10
                  ]
              }
          }
      },
      "vision_v2": {
          "growth_path": "",
          "group_id": "vision",
          "sub_group_id": "vision_v2",
          "plot_index": 2,
          "orientation_selectivity_pattern": "",
          "location": "",
          "kernel_size": 3,
          "cortical_neuron_count": 500,
          "location_generation_type": "random",
          "synapse_attractivity": 100,
          "init_synapse_needed": False,
          "postsynaptic_current": 150,
          "plasticity_constant": 0,
          "postsynaptic_current_max": 200,
          "neighbor_locator_rule_id": "rule_1",
          "neighbor_locator_rule_param_id": "param_2",
          "cortical_mapping_dst": {
                  "vision_v4-1": {
                      "neighbor_locator_rule_id": "rule_6",
                      "neighbor_locator_rule_param_id": "param_2"},
                  "vision_v4-2": {
                      "neighbor_locator_rule_id": "rule_6",
                      "neighbor_locator_rule_param_id": "param_2"},
                  "vision_v4-3": {
                      "neighbor_locator_rule_id": "rule_6",
                      "neighbor_locator_rule_param_id": "param_2"},
                  "vision_v4-4": {
                      "neighbor_locator_rule_id": "rule_6",
                      "neighbor_locator_rule_param_id": "param_2"},
                  "vision_v4-5": {
                      "neighbor_locator_rule_id": "rule_6",
                      "neighbor_locator_rule_param_id": "param_2"},
                  "vision_v4-6": {
                      "neighbor_locator_rule_id": "rule_6",
                      "neighbor_locator_rule_param_id": "param_2"}
          },
          "neuron_params": {
              "activation_function_id": "",
              "depolarization_threshold": 1.574682375,
              "firing_threshold": 1,
              "firing_pattern_id": "",
              "refractory_period": 0,
              "orientation_selectivity_id": "",
              "axon_avg_length": "",
              "leak_coefficient": 5,
              "axon_avg_connections": "",
              "axon_orientation function": "",
              "consecutive_fire_cnt_max": 0,
              "snooze_length": 0.0,
              "block_boundaries": [
                  28,
                  28,
                  17
              ],
              "geometric_boundaries": {
                  "x": [
                      0,
                      170
                  ],
                  "y": [
                      0,
                      170
                  ],
                  "z": [
                      0,
                      170
                  ]
              }
          }
      },
      "vision_v4-1": {
          "growth_path": "",
          "group_id": "vision",
          "sub_group_id": "vision_v4",
          "plot_index": 1,
          "orientation_selectivity_pattern": "/",
          "location": "",
          "kernel_size": 3,
          "cortical_neuron_count": 500,
          "location_generation_type": "random",
          "synapse_attractivity": 100,
          "init_synapse_needed": False,
          "postsynaptic_current": 150,
          "plasticity_constant": 0,
          "postsynaptic_current_max": 200,
          "neighbor_locator_rule_id": "rule_1",
          "neighbor_locator_rule_param_id": "param_2",
          "cortical_mapping_dst": {
              "vision_IT": {
                  "neighbor_locator_rule_id": "rule_6",
                  "neighbor_locator_rule_param_id": "param_2"
              }
          },
          "neuron_params": {
              "activation_function_id": "",
              "depolarization_threshold": 1.574682375,
              "firing_threshold": 1.1632698,
              "firing_pattern_id": "",
              "refractory_period": 0,
              "orientation_selectivity_id": "",
              "axon_avg_length": "",
              "leak_coefficient": 5,
              "axon_avg_connections": "",
              "axon_orientation function": "",
              "consecutive_fire_cnt_max": 0,
              "snooze_length": 0.0,
              "block_boundaries": [
                  56,
                  56,
                  1
              ],
              "geometric_boundaries": {
                  "x": [
                      0,
                      100
                  ],
                  "y": [
                      0,
                      100
                  ],
                  "z": [
                      0,
                      10
                  ]
              }
          }
      },
      "vision_v4-2": {
          "growth_path": "",
          "group_id": "vision",
          "sub_group_id": "vision_v4",
          "plot_index": 2,
          "orientation_selectivity_pattern": "\\",
          "location": "",
          "kernel_size": 3,
          "cortical_neuron_count": 500,
          "location_generation_type": "random",
          "synapse_attractivity": 100,
          "init_synapse_needed": False,
          "postsynaptic_current": 150,
          "plasticity_constant": 0,
          "postsynaptic_current_max": 200,
          "neighbor_locator_rule_id": "rule_1",
          "neighbor_locator_rule_param_id": "param_2",
          "cortical_mapping_dst": {
              "vision_IT": {
                  "neighbor_locator_rule_id": "rule_6",
                  "neighbor_locator_rule_param_id": "param_2"
              }
          },
          "neuron_params": {
              "activation_function_id": "",
              "depolarization_threshold": 1.574682375,
              "firing_threshold": 1.1632698,
              "firing_pattern_id": "",
              "refractory_period": 0,
              "orientation_selectivity_id": "",
              "axon_avg_length": "",
              "leak_coefficient": 5,
              "axon_avg_connections": "",
              "axon_orientation function": "",
              "consecutive_fire_cnt_max": 0,
              "snooze_length": 0.0,
              "block_boundaries": [
                  56,
                  56,
                  1
              ],
              "geometric_boundaries": {
                  "x": [
                      0,
                      100
                  ],
                  "y": [
                      0,
                      100
                  ],
                  "z": [
                      0,
                      10
                  ]
              }
          }
      },
      "vision_v4-3": {
          "growth_path": "",
          "group_id": "vision",
          "sub_group_id": "vision_v4",
          "plot_index": 3,
          "orientation_selectivity_pattern": "-",
          "location": "",
          "kernel_size": 3,
          "cortical_neuron_count": 500,
          "location_generation_type": "random",
          "synapse_attractivity": 100,
          "init_synapse_needed": False,
          "postsynaptic_current": 150,
          "plasticity_constant": 0,
          "postsynaptic_current_max": 200,
          "neighbor_locator_rule_id": "rule_1",
          "neighbor_locator_rule_param_id": "param_2",
          "cortical_mapping_dst": {
              "vision_IT": {
                  "neighbor_locator_rule_id": "rule_6",
                  "neighbor_locator_rule_param_id": "param_2"
              }
          },
          "neuron_params": {
              "activation_function_id": "",
              "depolarization_threshold": 1.574682375,
              "firing_threshold": 1.1632698,
              "firing_pattern_id": "",
              "refractory_period": 0,
              "orientation_selectivity_id": "",
              "axon_avg_length": "",
              "leak_coefficient": 5,
              "axon_avg_connections": "",
              "axon_orientation function": "",
              "consecutive_fire_cnt_max": 0,
              "snooze_length": 0.0,
              "block_boundaries": [
                  56,
                  56,
                  1
              ],
              "geometric_boundaries": {
                  "x": [
                      0,
                      100
                  ],
                  "y": [
                      0,
                      100
                  ],
                  "z": [
                      0,
                      10
                  ]
              }
          }
      },
      "vision_v4-4": {
          "growth_path": "",
          "group_id": "vision",
          "sub_group_id": "vision_v4",
          "plot_index": 4,
          "orientation_selectivity_pattern": "|",
          "location": "",
          "kernel_size": 3,
          "cortical_neuron_count": 500,
          "location_generation_type": "random",
          "synapse_attractivity": 100,
          "init_synapse_needed": False,
          "postsynaptic_current": 150,
          "plasticity_constant": 0,
          "postsynaptic_current_max": 200,
          "neighbor_locator_rule_id": "rule_1",
          "neighbor_locator_rule_param_id": "param_2",
          "cortical_mapping_dst": {
              "vision_IT": {
                  "neighbor_locator_rule_id": "rule_6",
                  "neighbor_locator_rule_param_id": "param_2"
              }
          },
          "neuron_params": {
              "activation_function_id": "",
              "depolarization_threshold": 1.574682375,
              "firing_threshold": 1.1632698,
              "firing_pattern_id": "",
              "refractory_period": 0,
              "orientation_selectivity_id": "",
              "axon_avg_length": "",
              "leak_coefficient": 5,
              "axon_avg_connections": "",
              "axon_orientation function": "",
              "consecutive_fire_cnt_max": 0,
              "snooze_length": 0.0,
              "block_boundaries": [
                  56,
                  56,
                  1
              ],
              "geometric_boundaries": {
                  "x": [
                      0,
                      100
                  ],
                  "y": [
                      0,
                      100
                  ],
                  "z": [
                      0,
                      10
                  ]
              }
          }
      },
      "vision_v4-5": {
          "growth_path": "",
          "group_id": "vision",
          "sub_group_id": "vision_v4",
          "plot_index": 5,
          "orientation_selectivity_pattern": "o",
          "location": "",
          "kernel_size": 3,
          "cortical_neuron_count": 500,
          "location_generation_type": "random",
          "synapse_attractivity": 100,
          "init_synapse_needed": False,
          "postsynaptic_current": 150,
          "plasticity_constant": 0,
          "postsynaptic_current_max": 200,
          "neighbor_locator_rule_id": "rule_1",
          "neighbor_locator_rule_param_id": "param_2",
          "cortical_mapping_dst": {
              "vision_IT": {
                  "neighbor_locator_rule_id": "rule_6",
                  "neighbor_locator_rule_param_id": "param_2"
              }
          },
          "neuron_params": {
              "activation_function_id": "",
              "depolarization_threshold": 1.574682375,
              "firing_threshold": 1.1632698,
              "firing_pattern_id": "",
              "refractory_period": 0,
              "orientation_selectivity_id": "",
              "axon_avg_length": "",
              "leak_coefficient": 5,
              "axon_avg_connections": "",
              "axon_orientation function": "",
              "consecutive_fire_cnt_max": 0,
              "snooze_length": 0.0,
              "block_boundaries": [
                  56,
                  56,
                  1
              ],
              "geometric_boundaries": {
                  "x": [
                      0,
                      100
                  ],
                  "y": [
                      0,
                      100
                  ],
                  "z": [
                      0,
                      10
                  ]
              }
          }
      },
      "vision_v4-6": {
          "growth_path": "",
          "group_id": "vision",
          "sub_group_id": "vision_v4",
          "plot_index": 6,
          "orientation_selectivity_pattern": ".",
          "location": "",
          "kernel_size": 3,
          "cortical_neuron_count": 500,
          "location_generation_type": "random",
          "synapse_attractivity": 100,
          "init_synapse_needed": False,
          "postsynaptic_current": 150,
          "plasticity_constant": 0,
          "postsynaptic_current_max": 200,
          "neighbor_locator_rule_id": "rule_1",
          "neighbor_locator_rule_param_id": "param_2",
          "cortical_mapping_dst": {
              "vision_IT": {
                  "neighbor_locator_rule_id": "rule_6",
                  "neighbor_locator_rule_param_id": "param_2"
              }
          },
          "neuron_params": {
              "activation_function_id": "",
              "depolarization_threshold": 1.574682375,
              "firing_threshold": 1.1632698,
              "firing_pattern_id": "",
              "refractory_period": 0,
              "orientation_selectivity_id": "",
              "axon_avg_length": "",
              "leak_coefficient": 5,
              "axon_avg_connections": "",
              "axon_orientation function": "",
              "consecutive_fire_cnt_max": 0,
              "snooze_length": 0.0,
              "block_boundaries": [
                  56,
                  56,
                  1
              ],
              "geometric_boundaries": {
                  "x": [
                      0,
                      100
                  ],
                  "y": [
                      0,
                      100
                  ],
                  "z": [
                      0,
                      10
                  ]
              }
          }
      },
      "vision_IT": {
          "growth_path": "",
          "group_id": "vision",
          "sub_group_id": "vision_IT",
          "plot_index": 3,
          "orientation_selectivity_pattern": "",
          "location": "",
          "kernel_size": 7,
          "cortical_neuron_count": 500,
          "location_generation_type": "random",
          "synapse_attractivity": 80,
          "init_synapse_needed": False,
          "postsynaptic_current": 100,
          "plasticity_constant": 0.5,
          "postsynaptic_current_max": 300,
          "neighbor_locator_rule_id": "rule_1",
          "neighbor_locator_rule_param_id": "param_2",
          "cortical_mapping_dst": {
              "vision_memory": {
                  "neighbor_locator_rule_id": "rule_6",
                  "neighbor_locator_rule_param_id": "param_2"
              }
          },
          "neuron_params": {
              "activation_function_id": "",
              "orientation_selectivity_id": "",
              "depolarization_threshold": 1.574682375,
              "firing_threshold": 0.882,
              "firing_pattern_id": "",
              "refractory_period": 0,
              "axon_avg_length": "",
              "leak_coefficient": 5,
              "axon_avg_connections": "",
              "axon_orientation function": "",
              "consecutive_fire_cnt_max": 0,
              "snooze_length": 1.02467266,
              "block_boundaries": [
                  56,
                  56,
                  1
              ],
              "geometric_boundaries": {
                  "x": [
                      0,
                      170
                  ],
                  "y": [
                      0,
                      170
                  ],
                  "z": [
                      0,
                      170
                  ]
              }
          }
      },
      "vision_memory": {
          "growth_path": "",
          "group_id": "Memory",
          "sub_group_id": "vision",
          "plot_index": 1,
          "orientation_selectivity_pattern": "",
          "location": "",
          "kernel_size": 7,
          "cortical_neuron_count": 1001,
          "location_generation_type": "random",
          "synapse_attractivity": 10,
          "init_synapse_needed": False,
          "postsynaptic_current": 0.9,
          "plasticity_constant": 1,
          "postsynaptic_current_max": 60,
          "neighbor_locator_rule_id": "rule_0",
          "neighbor_locator_rule_param_id": "param_1",
          "cortical_mapping_dst": {},
          "neuron_params": {
              "activation_function_id": "",
              "orientation_selectivity_id": "",
              "depolarization_threshold": 5,
              "firing_threshold": 1.5,
              "firing_pattern_id": "",
              "refractory_period": 0,
              "axon_avg_length": "",
              "leak_coefficient": 5,
              "axon_avg_connections": "",
              "axon_orientation function": "",
              "consecutive_fire_cnt_max": 2,
              "snooze_length": 8,
              "block_boundaries": [
                  50,
                  50,
                  1
              ],
              "geometric_boundaries": {
                  "x": [
                      0,
                      500
                  ],
                  "y": [
                      0,
                      500
                  ],
                  "z": [
                      0,
                      50
                  ]
              }
          }
      },
      "utf8": {
          "growth_path": "",
          "group_id": "IPU",
          "sub_group_id": "IPU_utf8",
          "plot_index": 1,
          "orientation_selectivity_pattern": "",
          "location": "",
          "kernel_size": 7,
          "cortical_neuron_count": 300,
          "location_generation_type": "sequential",
          "synapse_attractivity": 100,
          "init_synapse_needed": False,
          "postsynaptic_current": 501,
          "plasticity_constant": 0.05,
          "postsynaptic_current_max": 501,
          "neighbor_locator_rule_id": "rule_0",
          "neighbor_locator_rule_param_id": "param_1",
          "cortical_mapping_dst": {
              "utf8_memory": {
                  "neighbor_locator_rule_id": "rule_0",
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
                  300
              ],
              "geometric_boundaries": {
                  "x": [
                      0,
                      1
                  ],
                  "y": [
                      0,
                      1
                  ],
                  "z": [
                      0,
                      300
                  ]
              }
          }
      },
      "utf8_memory": {
          "growth_path": "",
          "group_id": "Memory",
          "sub_group_id": "utf8",
          "plot_index": 2,
          "orientation_selectivity_pattern": "",
          "location": "",
          "kernel_size": 7,
          "cortical_neuron_count": 300,
          "location_generation_type": "sequential",
          "synapse_attractivity": 100,
          "init_synapse_needed": False,
          "postsynaptic_current": 11.2,
          "plasticity_constant": 1,
          "postsynaptic_current_max": 11.2,
          "neighbor_locator_rule_id": "rule_0",
          "neighbor_locator_rule_param_id": "param_1",
          "cortical_mapping_dst": {
              "utf8_out": {
                  "neighbor_locator_rule_id": "rule_0",
                  "neighbor_locator_rule_param_id": "param_2"
              }
          },
          "neuron_params": {
              "activation_function_id": "",
              "orientation_selectivity_id": "",
              "depolarization_threshold": 20,
              "firing_threshold": 20,
              "firing_pattern_id": "",
              "refractory_period": 0,
              "axon_avg_length": "",
              "leak_coefficient": 5,
              "axon_avg_connections": "",
              "axon_orientation function": "",
              "consecutive_fire_cnt_max": 100000,
              "snooze_length": 2,
              "block_boundaries": [
                  1,
                  1,
                  300
              ],
              "geometric_boundaries": {
                  "x": [
                      0,
                      1
                  ],
                  "y": [
                      0,
                      1
                  ],
                  "z": [
                      0,
                      300
                  ]
              }
          }
      },
      "utf8_out": {
          "growth_path": "",
          "group_id": "OPU",
          "sub_group_id": "OPU_utf8",
          "plot_index": 1,
          "orientation_selectivity_pattern": "",
          "location": "",
          "kernel_size": 7,
          "cortical_neuron_count": 300,
          "location_generation_type": "sequential",
          "synapse_attractivity": 100,
          "init_synapse_needed": False,
          "postsynaptic_current": 0.51,
          "plasticity_constant": 0.05,
          "postsynaptic_current_max": 1,
          "neighbor_locator_rule_id": "rule_0",
          "neighbor_locator_rule_param_id": "param_1",
          "cortical_mapping_dst": {},
          "neuron_params": {
              "activation_function_id": "",
              "orientation_selectivity_id": "",
              "depolarization_threshold": 20,
              "firing_threshold": 10,
              "firing_pattern_id": "",
              "refractory_period": 0,
              "axon_avg_length": "",
              "leak_coefficient": 1,
              "axon_avg_connections": "",
              "axon_orientation function": "",
              "consecutive_fire_cnt_max": 1,
              "snooze_length": 0,
              "block_boundaries": [
                  1,
                  1,
                  300
              ],
              "geometric_boundaries": {
                  "x": [
                      0,
                      1
                  ],
                  "y": [
                      0,
                      1
                  ],
                  "z": [
                      0,
                      300
                  ]
              }
          }
      },
      "pain":{
          "growth_path": "",
          "group_id": "PAIN",
          "sub_group_id": "pain",
          "plot_index": 1,
          "orientation_selectivity_pattern": "",
          "location": "",
          "kernel_size": 7,
          "cortical_neuron_count": 5,
          "location_generation_type": "sequential",
          "synapse_attractivity": 100,
          "init_synapse_needed": False,
          "postsynaptic_current": 0.51,
          "plasticity_constant": 0.05,
          "postsynaptic_current_max": 1,
          "neighbor_locator_rule_id": "rule_0",
          "neighbor_locator_rule_param_id": "param_1",
          "cortical_mapping_dst": {},
          "neuron_params": {
              "activation_function_id": "",
              "orientation_selectivity_id": "",
              "depolarization_threshold": 20,
              "firing_threshold": 10,
              "firing_pattern_id": "",
              "refractory_period": 0,
              "axon_avg_length": "",
              "leak_coefficient": 1,
              "axon_avg_connections": "",
              "axon_orientation function": "",
              "consecutive_fire_cnt_max": 1,
              "snooze_length": 0,
              "block_boundaries": [
                  1,
                  1,
                  1
              ],
              "geometric_boundaries": {
                  "x": [
                      0,
                      1
                  ],
                  "y": [
                      0,
                      1
                  ],
                  "z": [
                      0,
                      5
                  ]
              }
          }
      },
      "thalamus_auditory": {
          "growth_path": "",
          "direction_sensitivity": "/",
          "group_id": "thalamus",
          "sub_group_id": "auditory",
          "plot_index": 1,
          "layer_index": 1,
          "total_layer_count": 7,
          "orientation_selectivity_pattern": "",
          "location": "",
          "kernel_size": 5,
          "cortical_neuron_count": 210,
          "location_generation_type": "random",
          "synapse_attractivity": 100,
          "init_synapse_needed": False,
          "postsynaptic_current": 20,
          "plasticity_constant": 0,
          "postsynaptic_current_max": 1000,
          "neighbor_locator_rule_id": "rule_1",
          "neighbor_locator_rule_param_id": "param_1",
          "cortical_mapping_dst": {},
          "neuron_params": {
              "activation_function_id": "",
              "depolarization_threshold": 1.633575495825,
              "orientation_selectivity_id": "",
              "firing_threshold": 2.89235403072,
              "firing_pattern_id": "",
              "refractory_period": 0,
              "axon_avg_length": "",
              "leak_coefficient": 10,
              "axon_avg_connections": "",
              "axon_orientation function": "",
              "consecutive_fire_cnt_max": 0,
              "snooze_length": 1.659231437448,
              "block_boundaries": [
                  28,
                  28,
                  1
              ],
              "geometric_boundaries": {
                  "x": [
                      0,
                      32
                  ],
                  "y": [
                      0,
                      32
                  ],
                  "z": [
                      0,
                      10
                  ]
              }
          }
      },
      "proximity": {
        "growth_path" : "",
        "group_id" : "IPU",
        "sub_group_id" : "IPU_proximity",
        "plot_index" : 3,
        "orientation_selectivity_pattern" : "",
        "location" : "",
        "kernel_size" : 7,
        "cortical_neuron_count" : 50000,
        "location_generation_type" : "random",
        "synapse_attractivity" : 80,
        "init_synapse_needed" : False,
        "postsynaptic_current" : 100,
        "plasticity_constant" : 0.5,
        "postsynaptic_current_max" : 300,
        "neighbor_locator_rule_id" : "rule_1",
        "neighbor_locator_rule_param_id" : "param_2",
        "cortical_mapping_dst" : {
            "proximity_memory" : {
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
                360,
                180,
                20
            ],
            "geometric_boundaries" : {
                "x" : [
                    0,
                    3600
                ],
                "y" : [
                    0,
                    1800
                ],
                "z" : [
                    0,
                    10
                ]
            }
        }
    },
    "proximity_memory" : {
        "growth_path" : "",
        "group_id" : "Memory",
        "sub_group_id" : "proximity",
        "plot_index" : 1,
        "orientation_selectivity_pattern" : "",
        "location" : "",
        "kernel_size" : 7,
        "cortical_neuron_count" : 1001,
        "location_generation_type" : "random",
        "synapse_attractivity" : 10,
        "init_synapse_needed" : False,
        "postsynaptic_current" : 0.9,
        "plasticity_constant" : 1,
        "postsynaptic_current_max" : 60,
        "neighbor_locator_rule_id" : "rule_0",
        "neighbor_locator_rule_param_id" : "param_1",
        "cortical_mapping_dst" : {
            "movement_opu" : {
                "neighbor_locator_rule_id": "rule_1",
                "neighbor_locator_rule_param_id": "param_2"
            }
        },
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
                50,
                50,
                1
            ],
            "geometric_boundaries" : {
                "x" : [
                    0,
                    500
                ],
                "y" : [
                    0,
                    500
                ],
                "z" : [
                    0,
                    50
                ]
            }
        }
    },
      "movement_opu": {
          "growth_path": "",
          "group_id": "OPU",
          "sub_group_id": "OPU_movement",
          "plot_index": 1,
          "orientation_selectivity_pattern": "",
          "location": "",
          "kernel_size": 7,
          "cortical_neuron_count": 4,
          "location_generation_type": "sequential",
          "synapse_attractivity": 100,
          "init_synapse_needed": False,
          "postsynaptic_current": 0.51,
          "plasticity_constant": 0.05,
          "postsynaptic_current_max": 1,
          "neighbor_locator_rule_id": "rule_0",
          "neighbor_locator_rule_param_id": "param_1",
          "cortical_mapping_dst": {},
          "neuron_params": {
              "activation_function_id": "",
              "orientation_selectivity_id": "",
              "depolarization_threshold": 20,
              "firing_threshold": 10,
              "firing_pattern_id": "",
              "refractory_period": 0,
              "axon_avg_length": "",
              "leak_coefficient": 1,
              "axon_avg_connections": "",
              "axon_orientation function": "",
              "consecutive_fire_cnt_max": 1,
              "snooze_length": 0,
              "block_boundaries": [
                  1,
                  1,
                  4
              ],
              "geometric_boundaries": {
                  "x": [
                      0,
                      1
                  ],
                  "y": [
                      0,
                      1
                  ],
                  "z": [
                      0,
                      4
                  ]
              }
          }
      }
  }
}
