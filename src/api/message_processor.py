
import json
from inf import runtime_data, disk_ops
from inf.initialize import init_brain, reset_runtime_data, id_gen
from evo.genome_processor import genome_ver_check
from evo.neuroembryogenesis import develop_brain
from evo.autopilot import update_generation_dict


def api_message_processor(api_message):
    """
    Processes the incoming API calls to FEAGI
    """

    if 'burst_management' in api_message:
        if 'burst_duration' in api_message['burst_management']:
            if api_message['burst_management']['burst_duration'] is not None:
                runtime_data.burst_timer = api_message['burst_management']['burst_duration']

    if 'stimulation_script' in api_message:
        runtime_data.stimulation_script = api_message['stimulation_script']['stimulation_script']

    if 'log_management' in api_message:
        if 'print_burst_info' in api_message['log_management']:
            runtime_data.parameters['Logs']['print_burst_info'] \
                = api_message['log_management']['print_burst_info']
        if 'print_messenger_logs' in api_message['log_management']:
            runtime_data.parameters['Logs']['print_messenger_logs'] \
                = api_message['log_management']['print_messenger_logs']

    if 'connectome_snapshot' in api_message:
        if 'connectome_path' in api_message['connectome_snapshot']:
            if api_message['connectome_snapshot']['connectome_path']:
                print("Taking a snapshot of the brain... ... ...")
                disk_ops.save_brain_to_disk(connectome_path=api_message['connectome_snapshot']['connectome_path'],
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
        reset_runtime_data()
        runtime_data.genome_counter += 1
        runtime_data.genome_reset_flag = False
        runtime_data.genome_ver = None
        runtime_data.genome_orig = dict(api_message['genome']).copy()
        runtime_data.genome = api_message['genome']
        runtime_data.genome = genome_ver_check(runtime_data.genome)
        runtime_data.genome_ver = "2.0"
        init_brain()
        if 'genome_id' not in runtime_data.genome:
            runtime_data.genome['genome_id'] = id_gen(signature="_G")
        runtime_data.genome_id = runtime_data.genome['genome_id']
        print("#$% " * 30)
        print("brain_run_id", runtime_data.brain_run_id)
        if runtime_data.autopilot:
            update_generation_dict(genome_id=runtime_data.genome_id,
                                   robot_id=runtime_data.robot_id,
                                   env_id=runtime_data.environment_id)
        # Process of artificial neuroembryogenesis that leads to connectome development
        develop_brain(reincarnation_mode=runtime_data.parameters[
            'Brain_Development']['reincarnation_mode'])

    if 'beacon_sub' in api_message:
        print("The following FEAGi beacon subscriber has been added:\n", api_message['beacon_sub'])
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

    api_message = {}
