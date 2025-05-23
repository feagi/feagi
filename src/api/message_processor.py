#
# Copyright 2016-Present Neuraville Inc. All Rights Reserved.
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

import datetime
# import json
import json

from src.inf import runtime_data, disk_ops
# from src.evo.genome_processor import genome_ver_check
# from src.evo.autopilot import update_generation_dict
from src.evo.x_genesis import update_cortical_properties, update_morphology_properties, update_cortical_mappings
from src.evo.x_genesis import add_core_cortical_area, add_custom_cortical_area, cortical_removal, append_circuit
from src.evo.x_genesis import create_missing_pns_areas, cortical_regeneration
from src.inf.db_handler import InfluxManagement
from src.inf.initialize import deploy_genome
from src.evo.templates import cortical_types
from src.pns.stimuli_translator import induce_manual_stimulation, induce_sustained_stimulation
from src.pns.vision import reconfigure_vision

influx = InfluxManagement()


def record_training_event(event_name, details=None):
    timestamp = datetime.datetime.utcnow().isoformat()
    runtime_data.training_stats[timestamp] = {'event_name': event_name, 'details': details}


def api_message_processor(api_message):
    """
    Processes the incoming API calls to FEAGI
    """

    if 'burst_management' in api_message:
        if 'burst_duration' in api_message['burst_management']:
            if api_message['burst_management']['burst_duration'] is not None:
                runtime_data.burst_timer = api_message['burst_management']['burst_duration']
                runtime_data.genome['physiology']['burst_delay'] = runtime_data.burst_timer

    if 'stimulation_script' in api_message:
        runtime_data.stimulation_script = api_message['stimulation_script']['stimulation_script']

    if 'log_management' in api_message:
        if 'print_cortical_activity_counters' in api_message['log_management']:
            runtime_data.parameters['Logs']['print_cortical_activity_counters'] \
                = api_message['log_management']['print_messenger_logs']
        if 'print_burst_info' in api_message['log_management']:
            runtime_data.parameters['Logs']['print_burst_info'] \
                = api_message['log_management']['print_burst_info']
        if 'print_messenger_logs' in api_message['log_management']:
            runtime_data.parameters['Logs']['print_messenger_logs'] \
                = api_message['log_management']['print_messenger_logs']

    if 'connectome_path' in api_message:
        if api_message['connectome_path']:
            print("Taking a snapshot of the brain... ... ...")
            disk_ops.save_brain_to_disk(connectome_path=api_message['connectome_path'],
                                        type='snapshot')
        else:
            disk_ops.save_brain_to_disk()

    if 'neuron_mp_collection_scope' in api_message:
        if api_message['neuron_mp_collection_scope'] is not None:
            payload = api_message['neuron_mp_collection_scope']['collection_scope']
            runtime_data.neuron_mp_collection_scope = payload
            print('Membrane Potential state collection scope has been updated.')
        else:
            print('Membrane Potential state collection scope did not change.')

    if 'neuron_psp_collection_scope' in api_message:
        if api_message['neuron_psp_collection_scope'] is not None:
            payload = api_message['neuron_psp_collection_scope']['collection_scope']
            runtime_data.neuron_psp_collection_scope = payload
            print('Membrane Potential state collection scope has been updated.')
        else:
            print('Membrane Potential state collection scope did not change.')

    if 'network_management' in api_message:
        print("api_message", api_message)
        if 'godot_host' in api_message['network_management']:
            runtime_data.parameters['Sockets']['godot_host_name'] = api_message['network_management']['godot_host']
        if 'godot_port' in api_message['network_management']:
            runtime_data.parameters['Sockets']['feagi_inbound_port_godot'] = \
                api_message['network_management']['godot_port']

        if 'gazebo_host' in api_message['network_management']:
            runtime_data.parameters['Sockets']['gazebo_host_name'] = api_message['network_management']['gazebo_host']
        if 'gazebo_port' in api_message['network_management']:
            runtime_data.parameters['Sockets']['feagi_inbound_port_gazebo'] = \
                api_message['network_management']['gazebo_port']

        # if 'embodiment_host' in api_message['network_management']:
        #     runtime_data.parameters['Sockets']['embodiment_host_name'] = \
        #         api_message['network_management']['embodiment_host']
        # if 'embodiment_port' in api_message['network_management']:
        #     runtime_data.parameters['Sockets']['feagi_inbound_port_embodiment'] = \
        #         api_message['network_management']['embodiment_port']

    if 'shock' in api_message:
        if api_message["shock"]:
            runtime_data.shock_admin = True
            runtime_data.shock_scenarios = api_message["shock"]
            print("Shock admin has been turned on.")
        else:
            runtime_data.shock_admin = False
            print("Shock admin has been turned off.")

    if 'regenerate' in api_message:
        print("<>-----------------------<>\n Regeneration issued!")
        runtime_data.exit_condition = True
        # neuroembryogenesis.develop_brain(reincarnation_mode=runtime_data.parameters[
        #     'Brain_Development']['reincarnation_mode'])

    if 'genome' in api_message:
        print("\n\n\n")
        print("========================================================")
        print(" Genome loading has been initiated by an API call...")
        print("========================================================")

        genome_data = dict(api_message['genome'])
        deploy_genome(neuroembryogenesis_flag=True, reset_runtime_data_flag=True, genome_data=genome_data)

    if 'beacon_sub' in api_message:
        print("The following FEAGI beacon subscriber has been added:\n", api_message['beacon_sub'])
        runtime_data.beacon_sub.add(api_message['beacon_sub'])

    if 'beacon_unsub' in api_message:
        if api_message['beacon_unsub'] in runtime_data.beacon_sub:
            runtime_data.beacon_sub.remove(api_message['beacon_unsub'])
            print("The following subscriber has been removed from FEAGI beacon:\n", api_message['beacon_unsub'])

    if 'robot_controller' in api_message:
        if api_message['robot_controller']['motor_power_coefficient']:
            runtime_data.robot_controller['motor_power_coefficient'] = \
                api_message['robot_controller']['motor_power_coefficient']
        if api_message['robot_controller']['robot_starting_position']:
            runtime_data.robot_controller['robot_starting_position'] = \
                api_message['robot_controller']['robot_starting_position']

    if 'robot_model' in api_message:
        if api_message['robot_model']['robot_sdf_file_name']:
            runtime_data.robot_model['robot_sdf_file_name'] = \
                api_message['robot_model']['robot_sdf_file_name']

        if api_message['robot_model']['robot_sdf_file_name_path']:
            runtime_data.robot_model['robot_sdf_file_name_path'] = \
                api_message['robot_model']['robot_sdf_file_name_path']

        if api_message['robot_model']['gazebo_floor_img_file']:
            runtime_data.robot_model['gazebo_floor_img_file'] = \
                api_message['robot_model']['gazebo_floor_img_file']

        if api_message['robot_model']['gazebo_floor_img_file_path']:
            runtime_data.robot_model['gazebo_floor_img_file_path'] = \
                api_message['robot_model']['gazebo_floor_img_file_path']

        if api_message['robot_model']['mu']:
            runtime_data.robot_model['mu'] = \
                api_message['robot_model']['mu']

        if api_message['robot_model']['mu2']:
            runtime_data.robot_model['mu2'] = \
                api_message['robot_model']['mu2']

        if api_message['robot_model']['fdir']:
            runtime_data.robot_model['fdir'] = \
                api_message['robot_model']['fdir']

        if api_message['robot_model']['slip1']:
            runtime_data.robot_model['slip1'] = \
                api_message['robot_model']['slip1']

        if api_message['robot_model']['slip2']:
            runtime_data.robot_model['slip2'] = \
                api_message['robot_model']['slip2']

    if 'update_cortical_properties' in api_message:
        update_cortical_properties(cortical_properties=api_message['update_cortical_properties'])

    if 'update_multiple_cortical_properties' in api_message:
        for message in api_message['update_multiple_cortical_properties']:
            update_cortical_properties(cortical_properties=message)

    if 'update_cortical_mappings' in api_message:
        update_cortical_mappings(cortical_mappings=api_message['update_cortical_mappings'])

    if 'update_morphology_properties' in api_message:
        update_morphology_properties(morphology_properties=api_message['update_morphology_properties'])

    if 'delete_cortical_area' in api_message:
        cortical_removal(cortical_area=api_message['delete_cortical_area'],
                         genome_scrub=True)

    if 'add_core_cortical_area' in api_message:
        add_core_cortical_area(cortical_properties=api_message['add_core_cortical_area'])

    if 'add_custom_cortical_area' in api_message:
        add_custom_cortical_area(cortical_name=api_message['add_custom_cortical_area']['cortical_name'],
                                 coordinates_3d=api_message['add_custom_cortical_area']['coordinates_3d'],
                                 coordinates_2d=api_message['add_custom_cortical_area']['coordinates_2d'],
                                 cortical_dimensions=api_message['add_custom_cortical_area']['cortical_dimensions'],
                                 parent_region_id=api_message['add_custom_cortical_area']['parent_region_id'],
                                 is_memory=api_message['add_custom_cortical_area']['is_memory'],
                                 copy_of=api_message['add_custom_cortical_area']['copy_of'],
                                 cortical_area_id=api_message['add_custom_cortical_area']['cortical_id'],
                                 )

    if 'append_circuit' in api_message:
        if runtime_data.genome:
            append_circuit(source_genome=api_message['append_circuit']["genome_str"],
                           circuit_origin=api_message['append_circuit']['circuit_origin'],
                           parent_brain_region=api_message['append_circuit']['parent_brain_region'],
                           rewire_mode=api_message['append_circuit']['rewire_mode'])
        else:
            deploy_genome(neuroembryogenesis_flag=True, reset_runtime_data_flag=True,
                          genome_data=api_message['append_circuit']["genome_str"])

    if 'reward' in api_message:
        reward_intensity = api_message['reward']
        if runtime_data.influxdb:
            print("+++++++++++  REWARD ++++++++++++++")
            runtime_data.influxdb.insert_game_activity(genome_id=runtime_data.genome_id, event="success",
                                                       intensity=reward_intensity)
        else:
            record_training_event(event_name="reward", details={"intensity": reward_intensity})

    if 'punishment' in api_message:
        punishment_intensity = api_message['punishment']
        if runtime_data.influxdb:
            print("----------- PUNISHMENT --------------")
            runtime_data.influxdb.insert_game_activity(genome_id=runtime_data.genome_id, event="punishment",
                                                       intensity=punishment_intensity)
        else:
            record_training_event(event_name="punishment", details={"intensity": punishment_intensity})

    if 'game_over' in api_message:
        if runtime_data.influxdb:
            runtime_data.influxdb.insert_game_activity(genome_id=runtime_data.genome_id, event="game_over")
        else:
            record_training_event(event_name="game_over")

    if 'update_pns_areas' in api_message:
        agent_capabilities = api_message['update_pns_areas']
        if agent_capabilities:
            dev_list = {}
            for device_type in agent_capabilities:
                cortical_type = "unknown"

                if device_type in ["input", "inputs"]:
                    cortical_type = "IPU"

                if device_type in ["output", "outputs"]:
                    cortical_type = "OPU"

                for device_name in agent_capabilities[device_type]:
                    if cortical_type in ["IPU", "OPU"]:
                        if device_name in cortical_types[cortical_type]["name_to_id_mapping"]:

                            max_feagi_index = 0
                            for _ in agent_capabilities[device_type][device_name]:
                                device_is_active = not agent_capabilities[device_type][device_name][_].get("disabled",
                                                                                                           False)
                                if device_is_active:
                                    feagi_index = agent_capabilities[device_type][device_name][_].get("feagi_index", 0)
                                    if int(feagi_index) > max_feagi_index:
                                        max_feagi_index = int(feagi_index)

                            for cortical_id in cortical_types[cortical_type]["name_to_id_mapping"][device_name]:
                                dev_list[cortical_id] = {"max_feagi_index": max_feagi_index}
                    else:
                        print(f"Device name {device_name} has invalid type {device_type}!")

            create_missing_pns_areas(dev_list=dev_list)

    if 'manual_stimulation' in api_message:
        induce_manual_stimulation(stimulation=api_message["manual_stimulation"])

    if 'sustained_stimulation' in api_message:
        induce_sustained_stimulation(stimulation=api_message["sustained_stimulation"])

    if 'vision' in api_message:
        vision_configuration_params = dict(api_message['vision'])
        reconfigure_vision(vision_parameters=vision_configuration_params)

    if 'reset' in api_message:
        cortical_regeneration(cortical_area=api_message["reset"])
