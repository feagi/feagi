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
      },
      "rule_7": {
          "param_1": 1,
          "param_2": 1
      },
      "rule_9": {
          "param_1": 1,
          "param_2": 1
      },
      "rule_10": {
          "param_1": 1,
          "param_2": 1
      },
      "rule_11": {
          "param_1": 1,
          "param_2": 1
      },
      "rule_12": {
          "param_1": 1,
          "param_2": 1
      },
      "rule_13": {
          "param_1": 1,
          "param_2": 1
      },
      "rule_14": {
          "param_1": 1,
          "param_2": 1
      },
      "rule_15": {
          "param_1": 1,
          "param_2": 1
      },
      "rule_16": {
          "param_1": 1,
          "param_2": 1
      },
      "rule_17": {
          "param_1": 1,
          "param_2": 1
      },
      "rule_18": {
          "param_1": 1,
          "param_2": 1
      },
      "rule_19": {
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
  "species": {
      "class": "toy",
      "brand": "gazebo",
      "model": "smart_car"
  },
  "blueprint": {
      "vision_thalamus_1": {
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
      "vision_thalamus_2": {
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
      "vision_thalamus_3": {
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
      "vision_thalamus_4": {
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
      "utf_thalamus": {
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
              "utf8_ipu": {
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
                      280
                  ],
                  "y": [
                      0,
                      280
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
          "cortical_neuron_count": 20,
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
      "utf8_ipu": {
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
      "pain": {
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
                  5
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
                      50
                  ]
              }
          }
      },
      "auditory_thalamus": {
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
      "battery_ipu": {
          "growth_path": "",
          "group_id": "IPU",
          "sub_group_id": "IPU_range",
          "plot_index": 1,
          "orientation_selectivity_pattern": "",
          "location": "",
          "kernel_size": 7,
          "cortical_neuron_count": 1000,
          "location_generation_type": "random",
          "synapse_attractivity": 100,
          "init_synapse_needed": False,
          "postsynaptic_current": 500,
          "plasticity_constant": 0.05,
          "postsynaptic_current_max": 35,
          "neighbor_locator_rule_id": "rule_5",
          "neighbor_locator_rule_param_id": "param_3",
          "cortical_mapping_dst": {
            "motor_babble_delay": {
                "neighbor_locator_rule_id" : "rule_17",
                "neighbor_locator_rule_param_id" : "param_1"
            },
            "motor_babble_endpoint": {
                "neighbor_locator_rule_id" : "rule_6",
                "neighbor_locator_rule_param_id" : "param_2",
                "plasticity": True
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
      },
      "proximity_ipu": {
        "growth_path" : "",
        "group_id" : "IPU",
        "sub_group_id" : "IPU_proximity",
        "plot_index" : 1,
        "orientation_selectivity_pattern" : "",
        "location" : "",
        "kernel_size" : 7,
        "cortical_neuron_count" : 1000,
        "location_generation_type" : "random",
        "synapse_attractivity" : 100,
        "init_synapse_needed" : False,
        "postsynaptic_current" : 500,
        "plasticity_constant" : 0.05,
        "postsynaptic_current_max" : 35,
        "neighbor_locator_rule_id" : "rule_5",
        "neighbor_locator_rule_param_id" : "param_3",
        "cortical_mapping_dst" : {
            "proximity_thalamus": {
                "neighbor_locator_rule_id" : "rule_6",
                "neighbor_locator_rule_param_id" : "param_2"
            },
        },
        "neuron_params" : {
            "activation_function_id" : "",
            "orientation_selectivity_id" : "",
            "depolarization_threshold" : 5,
            "firing_threshold" : 1,
            "firing_pattern_id" : "",
            "refractory_period" : 0,
            "axon_avg_length" : "",
            "leak_coefficient" : 10,
            "axon_avg_connections" : "",
            "axon_orientation function" : "",
            "consecutive_fire_cnt_max" : 3,
            "snooze_length" : 0,
            "block_boundaries" : [
                1,
                1,
                20
            ],
            "geometric_boundaries" : {
                "x" : [
                    0,
                    10
                ],
                "y" : [
                    0,
                    10
                ],
                "z" : [
                    0,
                    200
                ]
            }
        }
      },
      "proximity_thalamus": {
        "growth_path" : "",
        "group_id" : "IPU",
        "sub_group_id" : "IPU_proximity",
        "plot_index" : 1,
        "orientation_selectivity_pattern" : "",
        "location" : "",
        "kernel_size" : 7,
        "cortical_neuron_count" : 1000,
        "location_generation_type" : "random",
        "synapse_attractivity" : 100,
        "init_synapse_needed" : False,
        "postsynaptic_current" : 500,
        "plasticity_constant" : 0.05,
        "postsynaptic_current_max" : 35,
        "neighbor_locator_rule_id" : "rule_5",
        "neighbor_locator_rule_param_id" : "param_3",
        "cortical_mapping_dst" : {
            "proximity_decrease_filter": {
                "neighbor_locator_rule_id" : "rule_6",
                "neighbor_locator_rule_param_id" : "param_2"
            },
            "proximity_increase_filter": {
                "neighbor_locator_rule_id": "rule_6",
                "neighbor_locator_rule_param_id": "param_2"
            }
        },
        "neuron_params" : {
            "activation_function_id" : "",
            "orientation_selectivity_id" : "",
            "depolarization_threshold" : 5,
            "firing_threshold" : 1,
            "firing_pattern_id" : "",
            "refractory_period" : 0,
            "axon_avg_length" : "",
            "leak_coefficient" : 10,
            "axon_avg_connections" : "",
            "axon_orientation function" : "",
            "consecutive_fire_cnt_max" : 3,
            "snooze_length" : 0,
            "block_boundaries" : [
                1,
                1,
                20
            ],
            "geometric_boundaries" : {
                "x" : [
                    0,
                    10
                ],
                "y" : [
                    0,
                    10
                ],
                "z" : [
                    0,
                    200
                ]
            }
        }
      },
      "proximity_decrease_filter": {
        "growth_path" : "",
        "group_id" : "IPU",
        "sub_group_id" : "IPU_proximity",
        "plot_index" : 1,
        "orientation_selectivity_pattern" : "",
        "location" : "",
        "kernel_size" : 7,
        "cortical_neuron_count" : 1000,
        "location_generation_type" : "random",
        "synapse_attractivity" : 100,
        "init_synapse_needed" : False,
        "postsynaptic_current" : 100,
        "plasticity_constant" : 0.05,
        "postsynaptic_current_max" : 35,
        "neighbor_locator_rule_id" : "rule_5",
        "neighbor_locator_rule_param_id" : "param_3",
        "cortical_mapping_dst" : {
            "proximity_decrease_filter": {
                "neighbor_locator_rule_id" : "rule_9",
                "neighbor_locator_rule_param_id" : "param_1"
            },
            "proximity_decrease": {
                "neighbor_locator_rule_id" : "rule_11",
                "neighbor_locator_rule_param_id" : "param_1"
            }
        },
        "neuron_params" : {
            "activation_function_id" : "",
            "orientation_selectivity_id" : "",
            "depolarization_threshold" : 5,
            "firing_threshold" : 1,
            "firing_pattern_id" : "",
            "refractory_period" : 0,
            "axon_avg_length" : "",
            "leak_coefficient" : 10,
            "axon_avg_connections" : "",
            "axon_orientation function" : "",
            "consecutive_fire_cnt_max" : 3,
            "snooze_length" : 0,
            "block_boundaries" : [
                1,
                25,
                20
            ],
            "geometric_boundaries" : {
                "x" : [
                    0,
                    10
                ],
                "y" : [
                    0,
                    250
                ],
                "z" : [
                    0,
                    200
                ]
            }
        }
      },
      "proximity_decrease": {
        "growth_path" : "",
        "group_id" : "IPU",
        "sub_group_id" : "IPU_proximity",
        "plot_index" : 1,
        "orientation_selectivity_pattern" : "",
        "location" : "",
        "kernel_size" : 7,
        "cortical_neuron_count" : 100,
        "location_generation_type" : "random",
        "synapse_attractivity" : 100,
        "init_synapse_needed" : False,
        "postsynaptic_current" : 500,
        "plasticity_constant" : 0.05,
        "postsynaptic_current_max" : 35,
        "neighbor_locator_rule_id" : "rule_5",
        "neighbor_locator_rule_param_id" : "param_3",
        "cortical_mapping_dst" : {
            "motor_combinations": {
                "neighbor_locator_rule_id" : "rule_13",
                "neighbor_locator_rule_param_id" : "param_1",
                "plasticity": True
            }
        },
        "neuron_params" : {
            "activation_function_id" : "",
            "orientation_selectivity_id" : "",
            "depolarization_threshold" : 5,
            "firing_threshold" : 1,
            "firing_pattern_id" : "",
            "refractory_period" : 0,
            "axon_avg_length" : "",
            "leak_coefficient" : 10,
            "axon_avg_connections" : "",
            "axon_orientation function" : "",
            "consecutive_fire_cnt_max" : 3,
            "snooze_length" : 0,
            "block_boundaries" : [
                1,
                1,
                1
            ],
            "geometric_boundaries" : {
                "x" : [
                    0,
                    10
                ],
                "y" : [
                    0,
                    10
                ],
                "z" : [
                    0,
                    10
                ]
            }
        }
      },
      "proximity_increase_filter": {
        "growth_path" : "",
        "group_id" : "IPU",
        "sub_group_id" : "IPU_proximity",
        "plot_index" : 1,
        "orientation_selectivity_pattern" : "",
        "location" : "",
        "kernel_size" : 7,
        "cortical_neuron_count" : 1000,
        "location_generation_type" : "random",
        "synapse_attractivity" : 100,
        "init_synapse_needed" : False,
        "postsynaptic_current" : 100,
        "plasticity_constant" : 0.05,
        "postsynaptic_current_max" : 35,
        "neighbor_locator_rule_id" : "rule_5",
        "neighbor_locator_rule_param_id" : "param_3",
        "cortical_mapping_dst" : {
            "proximity_increase_filter": {
                "neighbor_locator_rule_id" : "rule_10",
                "neighbor_locator_rule_param_id" : "param_1"
            },
            "proximity_increase": {
                "neighbor_locator_rule_id" : "rule_12",
                "neighbor_locator_rule_param_id" : "param_1"
            }
        },
        "neuron_params" : {
            "activation_function_id" : "",
            "orientation_selectivity_id" : "",
            "depolarization_threshold" : 5,
            "firing_threshold" : 1,
            "firing_pattern_id" : "",
            "refractory_period" : 0,
            "axon_avg_length" : "",
            "leak_coefficient" : 10,
            "axon_avg_connections" : "",
            "axon_orientation function" : "",
            "consecutive_fire_cnt_max" : 3,
            "snooze_length" : 0,
            "block_boundaries" : [
                1,
                25,
                20
            ],
            "geometric_boundaries" : {
                "x" : [
                    0,
                    10
                ],
                "y" : [
                    0,
                    250
                ],
                "z" : [
                    0,
                    200
                ]
            }
        }
      },
      "proximity_increase": {
        "growth_path" : "",
        "group_id" : "IPU",
        "sub_group_id" : "IPU_proximity",
        "plot_index" : 1,
        "orientation_selectivity_pattern" : "",
        "location" : "",
        "kernel_size" : 7,
        "cortical_neuron_count" : 100,
        "location_generation_type" : "random",
        "synapse_attractivity" : 100,
        "init_synapse_needed" : False,
        "postsynaptic_current" : 500,
        "plasticity_constant" : 0.05,
        "postsynaptic_current_max" : 35,
        "neighbor_locator_rule_id" : "rule_5",
        "neighbor_locator_rule_param_id" : "param_3",
        "cortical_mapping_dst" : {
            "motor_combinations": {
                "neighbor_locator_rule_id" : "rule_13",
                "neighbor_locator_rule_param_id" : "param_1",
                "plasticity": True
            }
        },
        "neuron_params" : {
            "activation_function_id" : "",
            "orientation_selectivity_id" : "",
            "depolarization_threshold" : 5,
            "firing_threshold" : 1,
            "firing_pattern_id" : "",
            "refractory_period" : 0,
            "axon_avg_length" : "",
            "leak_coefficient" : 10,
            "axon_avg_connections" : "",
            "axon_orientation function" : "",
            "consecutive_fire_cnt_max" : 3,
            "snooze_length" : 0,
            "block_boundaries" : [
                1,
                1,
                1
            ],
            "geometric_boundaries" : {
                "x" : [
                    0,
                    10
                ],
                "y" : [
                    0,
                    10
                ],
                "z" : [
                    0,
                    10
                ]
            }
        }
      },
      "proximity_memory": {
        "growth_path" : "",
        "group_id" : "memory",
        "sub_group_id" : "proximity",
        "plot_index" : 2,
        "orientation_selectivity_pattern" : "",
        "location" : "",
        "kernel_size" : 7,
        "cortical_neuron_count" : 1000,
        "location_generation_type" : "random",
        "synapse_attractivity" : 100,
        "init_synapse_needed" : False,
        "postsynaptic_current" : 11.2,
        "plasticity_constant" : 1,
        "postsynaptic_current_max" : 11.2,
        "neighbor_locator_rule_id" : "rule_1",
        "neighbor_locator_rule_param_id" : "param_1",
        "cortical_mapping_dst" : {
            "direction_opu" : {
                "neighbor_locator_rule_id": "rule_1",
                "neighbor_locator_rule_param_id": "param_1"
            }
        },
        "neuron_params" : {
            "activation_function_id" : "",
            "orientation_selectivity_id" : "",
            "depolarization_threshold" : 5,
            "firing_threshold" : 0.001,
            "firing_pattern_id" : "",
            "refractory_period" : 0,
            "axon_avg_length" : "",
            "leak_coefficient" : 1,
            "axon_avg_connections" : "",
            "axon_orientation function" : "",
            "consecutive_fire_cnt_max" : 100000,
            "snooze_length" : 1,
            "block_boundaries" : [
                5,
                5,
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
                    200
                ]
            }
        }
      },
      "motor_thalamus": {
        "growth_path": "",
        "direction_sensitivity": "/",
        "group_id": "thalamus",
        "sub_group_id": "Motor_Cortex",
        "plot_index": 1,
        "orientation_selectivity_pattern": "",
        "location": "",
        "kernel_size": 7,
        "cortical_neuron_count": 500,
        "location_generation_type": "random",
        "synapse_attractivity": 100,
        "init_synapse_needed": False,
        "postsynaptic_current": 5,
        "plasticity_constant": 0.05,
        "postsynaptic_current_max": 500,
        "neighbor_locator_rule_id": "rule_5",
        "neighbor_locator_rule_param_id": "param_3",
        "cortical_mapping_dst": {
            "motor_opu": {
                "neighbor_locator_rule_id" : "rule_6",
                "neighbor_locator_rule_param_id" : "param_2"
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
            "consecutive_fire_cnt_max": 3,
            "snooze_length": 0,
            "block_boundaries": [
                8,
                1,
                1
            ],
            "geometric_boundaries": {
                "x": [
                    0,
                    80
                ],
                "y": [
                    0,
                    10
                ],
                "z": [
                    0,
                    10
                ]
            }
        }
      },
      "motor_combinations": {
        "growth_path": "",
        "direction_sensitivity": "/",
        "group_id": "thalamus",
        "sub_group_id": "Motor_Cortex",
        "plot_index": 1,
        "orientation_selectivity_pattern": "",
        "location": "",
        "kernel_size": 7,
        "cortical_neuron_count": 1000,
        "location_generation_type": "random",
        "synapse_attractivity": 100,
        "init_synapse_needed": False,
        "postsynaptic_current": 100,
        "plasticity_constant": 0.05,
        "postsynaptic_current_max": 500,
        "neighbor_locator_rule_id": "rule_5",
        "neighbor_locator_rule_param_id": "param_3",
        "cortical_mapping_dst": {
            "motor_thalamus": {
                "neighbor_locator_rule_id" : "rule_15",
                "neighbor_locator_rule_param_id" : "param_1"
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
            "consecutive_fire_cnt_max": 3,
            "snooze_length": 0,
            "block_boundaries": [
                256,
                1,
                1
            ],
            "geometric_boundaries": {
                "x": [
                    0,
                    256
                ],
                "y": [
                    0,
                    10
                ],
                "z": [
                    0,
                    10
                ]
            }
        }
      },
      "motor_babble_delay": {
        "growth_path": "",
        "direction_sensitivity": "/",
        "group_id": "thalamus",
        "sub_group_id": "Motor_Cortex",
        "plot_index": 1,
        "orientation_selectivity_pattern": "",
        "location": "",
        "kernel_size": 7,
        "cortical_neuron_count": 200,
        "location_generation_type": "random",
        "synapse_attractivity": 100,
        "init_synapse_needed": False,
        "postsynaptic_current": 5,
        "plasticity_constant": 0.05,
        "postsynaptic_current_max": 500,
        "neighbor_locator_rule_id": "rule_5",
        "neighbor_locator_rule_param_id": "param_3",
        "cortical_mapping_dst": {
            "motor_babble_endpoint": {
                "neighbor_locator_rule_id" : "rule_18",
                "neighbor_locator_rule_param_id" : "param_1",
                # "type": "inhibitory"
            },
            "motor_babble_delay": {
                "neighbor_locator_rule_id" : "rule_16",
                "neighbor_locator_rule_param_id" : "param_1"
            },
            "motor_combinations": {
                "neighbor_locator_rule_id" : "rule_19",
                "neighbor_locator_rule_param_id" : "param_1"
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
            "consecutive_fire_cnt_max": 3,
            "snooze_length": 0,
            "block_boundaries": [
                512,
                1,
                1
            ],
            "geometric_boundaries": {
                "x": [
                    0,
                    1024
                ],
                "y": [
                    0,
                    10
                ],
                "z": [
                    0,
                    10
                ]
            }
        }
      },
      "motor_babble_endpoint": {
        "growth_path": "",
        "direction_sensitivity": "/",
        "group_id": "thalamus",
        "sub_group_id": "Motor_Cortex",
        "plot_index": 1,
        "orientation_selectivity_pattern": "",
        "location": "",
        "kernel_size": 7,
        "cortical_neuron_count": 200,
        "location_generation_type": "random",
        "synapse_attractivity": 100,
        "init_synapse_needed": False,
        "postsynaptic_current": 5,
        "plasticity_constant": 0.05,
        "postsynaptic_current_max": 500,
        "neighbor_locator_rule_id": "rule_5",
        "neighbor_locator_rule_param_id": "param_3",
        "cortical_mapping_dst": {
            "motor_babble_delay": {
                "neighbor_locator_rule_id" : "rule_17",
                "neighbor_locator_rule_param_id" : "param_1",
                # "type": "inhibitory"
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
            "consecutive_fire_cnt_max": 3,
            "snooze_length": 0,
            "block_boundaries": [
                1,
                1,
                1
            ],
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
                    10
                ]
            }
        }
      },
      "motor_memory": {
          "growth_path": "",
          "group_id": "Memory",
          "sub_group_id": "Motor_Cortex",
          "plot_index": 1,
          "orientation_selectivity_pattern": "",
          "location": "",
          "kernel_size": 7,
          "cortical_neuron_count": 80,
          "location_generation_type": "random",
          "synapse_attractivity": 100,
          "init_synapse_needed": False,
          "postsynaptic_current": 501,
          "plasticity_constant": 0.05,
          "postsynaptic_current_max": 501,
          "neighbor_locator_rule_id": "rule_0",
          "neighbor_locator_rule_param_id": "param_1",
          "cortical_mapping_dst": {
              "motor_thalamus": {
                  "neighbor_locator_rule_id": "rule_6",
                  "neighbor_locator_rule_param_id": "param_1"
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
                  8
              ],
              "geometric_boundaries": {
                  "x": [
                      0,
                      5
                  ],
                  "y": [
                      0,
                      5
                  ],
                  "z": [
                      0,
                      24
                  ]
              }
          }
      },
      "motor_ipu": {
          "growth_path": "",
          "group_id": "IPU",
          "sub_group_id": "OPU_motor",
          "plot_index": 1,
          "orientation_selectivity_pattern": "",
          "location": "",
          "kernel_size": 7,
          "cortical_neuron_count": 1000,
          "location_generation_type": "random",
          "synapse_attractivity": 80,
          "init_synapse_needed": False,
          "postsynaptic_current": 0.51,
          "plasticity_constant": 0.05,
          "postsynaptic_current_max": 1,
          "neighbor_locator_rule_id": "rule_1",
          "neighbor_locator_rule_param_id": "param_1",
          "cortical_mapping_dst": {},
          "neuron_params": {
              "activation_function_id": "",
              "orientation_selectivity_id": "",
              "depolarization_threshold": 20,
              "firing_threshold": 1,
              "firing_pattern_id": "",
              "refractory_period": 0,
              "axon_avg_length": "",
              "leak_coefficient": 1,
              "axon_avg_connections": "",
              "axon_orientation function": "",
              "consecutive_fire_cnt_max": 1,
              "snooze_length": 0,
              "block_boundaries": [
                  4,
                  1,
                  20
              ],
              "geometric_boundaries": {
                  "x": [
                      0,
                      5
                  ],
                  "y": [
                      0,
                      5
                  ],
                  "z": [
                      0,
                      20
                  ]
              }
          }
      },
      "motor_opu": {
        "growth_path": "",
        "group_id": "OPU",
        "sub_group_id": "OPU_motor",
        "plot_index": 1,
        "orientation_selectivity_pattern": "",
        "location": "",
        "kernel_size": 7,
        "cortical_neuron_count": 500,
        "location_generation_type": "random",
        "synapse_attractivity": 80,
        "init_synapse_needed": False,
        "postsynaptic_current": 0.51,
        "plasticity_constant": 0.05,
        "postsynaptic_current_max": 1,
        "neighbor_locator_rule_id": "rule_1",
        "neighbor_locator_rule_param_id": "param_1",
        "cortical_mapping_dst": {},
        "neuron_params": {
            "activation_function_id": "",
            "orientation_selectivity_id": "",
            "depolarization_threshold": 20,
            "firing_threshold": 1,
            "firing_pattern_id": "",
            "refractory_period": 0,
            "axon_avg_length": "",
            "leak_coefficient": 1,
            "axon_avg_connections": "",
            "axon_orientation function": "",
            "consecutive_fire_cnt_max": 1,
            "snooze_length": 0,
            "block_boundaries": [
                8,
                1,
                1
            ],
            "geometric_boundaries": {
                "x": [
                    0,
                    80
                ],
                "y": [
                    0,
                    10
                ],
                "z": [
                    0,
                    10
                ]
            }
        }
      },
      "ir_ipu": {
        "growth_path": "",
        "group_id": "IPU",
        "sub_group_id": "IPU_ir_sensor",
        "plot_index": 1,
        "orientation_selectivity_pattern": "",
        "location": "",
        "kernel_size": 7,
        "cortical_neuron_count": 3,
        "location_generation_type": "sequential",
        "synapse_attractivity": 100,
        "init_synapse_needed": False,
        "postsynaptic_current": 50000,
        "plasticity_constant": 0.05,
        "postsynaptic_current_max": 501,
        "neighbor_locator_rule_id": "rule_0",
        "neighbor_locator_rule_param_id": "param_1",
        "cortical_mapping_dst": {
            # "ir_mapper": {
            #     "neighbor_locator_rule_id" : "rule_7",
            #     "neighbor_locator_rule_param_id" : "param_1"
            # },
            # "motor_ipu": {
            #     "neighbor_locator_rule_id": "rule_6",
            #     "neighbor_locator_rule_param_id": "param_1"
            # }
            # "motor_opu": {
            #     "neighbor_locator_rule_id": "rule_6",
            #     "neighbor_locator_rule_param_id": "param_1"
            # }
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
                3,
                1,
                1
            ],
            "geometric_boundaries": {
                "x": [
                    0,
                    3
                ],
                "y": [
                    0,
                    1
                ],
                "z": [
                    0,
                    1
                ]
            }
        }
      },
      "ir_mapper": {
        "growth_path": "",
        "group_id": "IPU",
        "sub_group_id": "IPU_ir_sensor",
        "plot_index": 1,
        "orientation_selectivity_pattern": "",
        "location": "",
        "kernel_size": 7,
        "cortical_neuron_count": 80,
        "location_generation_type": "random",
        "synapse_attractivity": 100,
        "init_synapse_needed": False,
        "postsynaptic_current": 501,
        "plasticity_constant": 0.05,
        "postsynaptic_current_max": 501,
        "neighbor_locator_rule_id": "rule_0",
        "neighbor_locator_rule_param_id": "param_1",
        "cortical_mapping_dst": {
            "ir_filter": {
                "neighbor_locator_rule_id": "rule_0",
                "neighbor_locator_rule_param_id": "param_1"
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
                8
            ],
            "geometric_boundaries": {
                "x": [
                    0,
                    5
                ],
                "y": [
                    0,
                    5
                ],
                "z": [
                    0,
                    24
                ]
            }
        }
      },
      "ir_filter": {
          "growth_path": "",
          "group_id": "IPU",
          "sub_group_id": "IPU_ir_sensor",
          "plot_index": 1,
          "orientation_selectivity_pattern": "",
          "location": "",
          "kernel_size": 7,
          "cortical_neuron_count": 80,
          "location_generation_type": "random",
          "synapse_attractivity": 100,
          "init_synapse_needed": False,
          "postsynaptic_current": 501,
          "plasticity_constant": 0.05,
          "postsynaptic_current_max": 501,
          "neighbor_locator_rule_id": "rule_0",
          "neighbor_locator_rule_param_id": "param_1",
          "cortical_mapping_dst": {
              "ir_filter": {
                  "neighbor_locator_rule_id": "rule_6",
                  "neighbor_locator_rule_param_id": "param_1"
              },
              "ir_memory": {
                  "neighbor_locator_rule_id": "rule_6",
                  "neighbor_locator_rule_param_id": "param_1"
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
                  8
              ],
              "geometric_boundaries": {
                  "x": [
                      0,
                      5
                  ],
                  "y": [
                      0,
                      5
                  ],
                  "z": [
                      0,
                      24
                  ]
              }
          }
      },
      "ir_memory": {
          "growth_path": "",
          "group_id": "IPU",
          "sub_group_id": "IPU_ir_sensor",
          "plot_index": 1,
          "orientation_selectivity_pattern": "",
          "location": "",
          "kernel_size": 7,
          "cortical_neuron_count": 80,
          "location_generation_type": "random",
          "synapse_attractivity": 100,
          "init_synapse_needed": False,
          "postsynaptic_current": 501,
          "plasticity_constant": 0.05,
          "postsynaptic_current_max": 501,
          "neighbor_locator_rule_id": "rule_0",
          "neighbor_locator_rule_param_id": "param_1",
          "cortical_mapping_dst": {
              "motor_memory": {
                  "neighbor_locator_rule_id": "rule_6",
                  "neighbor_locator_rule_param_id": "param_1"
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
                  8
              ],
              "geometric_boundaries": {
                  "x": [
                      0,
                      5
                  ],
                  "y": [
                      0,
                      5
                  ],
                  "z": [
                      0,
                      24
                  ]
              }
          }
      },
      "led_opu": {
        "growth_path": "",
        "group_id": "OPU",
        "sub_group_id": "OPU_led",
        "plot_index": 1,
        "orientation_selectivity_pattern": "",
        "location": "",
        "kernel_size": 7,
        "cortical_neuron_count": 1000,
        "location_generation_type": "random",
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
            "firing_threshold": 1,
            "firing_pattern_id": "",
            "refractory_period": 0,
            "axon_avg_length": "",
            "leak_coefficient": 1,
            "axon_avg_connections": "",
            "axon_orientation function": "",
            "consecutive_fire_cnt_max": 1,
            "snooze_length": 0,
            "block_boundaries": [
                8,
                1,
                3
            ],
            "geometric_boundaries": {
                "x": [
                    0,
                    80
                ],
                "y": [
                    0,
                    10
                ],
                "z": [
                    0,
                    30
                ]
            }
        }
    }
  }
}
