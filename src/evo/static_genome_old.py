genome = {
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
          "growth_path": "",
          "group_id": "IPU",
          "sub_group_id": "IPU_range",
          "plot_index": 1,
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
        "postsynaptic_current" : -500,
        "plasticity_constant" : 0.05,
        "postsynaptic_current_max" : 35,
        "neighbor_locator_rule_id" : "rule_5",
        "neighbor_locator_rule_param_id" : "param_3",
        "cortical_mapping_dst" : {
            "motor_thalamus": {
                "neighbor_locator_rule_id" : "rule_6",
                "neighbor_locator_rule_param_id" : "param_2"
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
        "cortical_neuron_count": 1000,
        "location_generation_type": "random",
        "synapse_attractivity": 100,
        "init_synapse_needed": False,
        "postsynaptic_current": 5,
        "plasticity_constant": 0,
        "postsynaptic_current_max": 5000,
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
                5,
                5,
                20
            ],
            "relative_coordinate": [
                50,
                 0,
                 0
            ],
            "visualization": True,
            "geometric_boundaries": {
                "x": [
                    0,
                    3600
                ],
                "y": [
                    0,
                    1800
                ],
                "z": [
                    0,
                    200
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
              "relative_coordinate": [
                30,
                 0,
                 0
              ],
              "visualization": True,
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
              "relative_coordinate": [
                 90,
                  0,
                  0
              ],
              "visualization": True,
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
            "relative_coordinate": [
                20,
                 0,
                 0
            ],
            "visualization": True,
            "geometric_boundaries": {
                "x": [
                    0,
                    40
                ],
                "y": [
                    0,
                    10
                ],
                "z": [
                    0,
                    200
                ]
            }
        }
      },
      "servo_opu": {
          "growth_path": "",
          "group_id": "OPU",
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
                  2,
                  1,
                  180
              ],
              "geometric_boundaries": {
                  "x": [
                      0,
                      20
                  ],
                  "y": [
                      0,
                      10
                  ],
                  "z": [
                      0,
                      360
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
            "motor_opu": {
                "neighbor_locator_rule_id": "rule_9",
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
