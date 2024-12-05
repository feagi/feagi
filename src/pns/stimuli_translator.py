# -*- coding: utf-8 -*-

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


import logging
from src.evo.voxels import *
from src.pns import stimuli_processor
from src.npu.physiology import update_membrane_potential_fire_queue

from src.evo.genome_processor import is_memory_cortical_area

logger = logging.getLogger(__name__)

"""
Translates device specific data into neuronal stimulation

This module facilitates the translation of cortical stimulation information to actual stimulation within connectome.

Stimulation data received will have the following data structure:
{ 'stimulation': {
        'motor_opu': [[x1, y1, z1], [x2, y2, z2], .....],
        'IR_opu': [[x1, y1, z1], [x2, y2, z2], .....],
        ...
        ...
    }
}

"""


def stimulation_validator(stimulation):
    # Removes stimulation that are not part of genome
    for cortical_area in stimulation.copy():
        if cortical_area not in runtime_data.cortical_list:
            del stimulation[cortical_area]
            print(f"!! Manual stimulation container {cortical_area} which was not part of genome !!")
    return stimulation


def induce_manual_stimulation(stimulation):
    """
    Stimulation data received will have the following data structure:
    { 'stimulation': {
            'i__inf': [[x1, y1, z1], [x2, y2, z2], .....],
            'i__pro': [[x1, y1, z1], [x2, y2, z2], .....],
            ...
            ...
        }
    }
    """
    stimulation = stimulation_validator(stimulation=stimulation)
    # stimulation = convert_stimulation_to_dashed_format(stimulation_data=stimulation)
    stimulation_injector(stimulation_data=stimulation)


# def convert_stimulation_to_dashed_format(stimulation_data):
#     """
#     Converts stimulation from list format to dashed format
#     # todo: revisit the value of having dashed format in the first place and get rid of it
#     """
#
#     dashed_stimulation = {}
#     for cortical_area in stimulation_data:
#         if not is_memory_cortical_area(cortical_area=cortical_area):
#             dashed_stimulation[cortical_area] = list()
#             for neuron_block in stimulation_data[cortical_area]:
#                 dashed_stimulation[cortical_area].append(neuron_block)
#     return dashed_stimulation


def stimuli_generator(input_instruction, burst_count):
    """
    It fakes cortical stimulation for the purpose of testing

    The following data format is used for input_instruction as the function input:

    input_instructions receives a dictionary as input with keys as the name of the ipu cortical name and the value
    being a list of block locations that needs to be activated in the block-ref format e.g. xBlock-yBlock-zBlock.

    Note: all of the blocks outlined in the data structure will be activated at the same time during the same
    burst.

    input_instruction_example = {
        ir_ipu: ["0-0-0", "1-0-0"],
        proximity_ipu: ["0-0-0", "0-0-3", "0-0-10", "0-0-20"]
        led_opu: ["5-0-0"]
    }

    # todo: Currently we can only inject data from the first index on each burst. change it so it goes thru all
    """
    neuron_list = []

    for cortical_area_ in input_instruction[burst_count]:
        if cortical_area_ in runtime_data.voxel_dict:
            for block_ref in input_instruction[burst_count][cortical_area_]:
                if block_ref in runtime_data.voxel_dict[cortical_area_]:
                    for neuron in runtime_data.voxel_dict[cortical_area_][block_ref]:
                        neuron_list.append(neuron)
                else:
                    print("Warning: Block ref %s was not found for %s" % (block_ref, cortical_area_))
            # print("neuron list:", cortical_area_, neuron_list)
            runtime_data.fcl_queue.put({cortical_area_: set(neuron_list)})
            neuron_list = []
        else:
            runtime_data.logs["CNS"].add(f"Warning: Cortical area %s not found within the voxel_dict {cortical_area_}")


# todo: has overlap with function called generic_ipu_translator in this module
def stimulation_injector(stimulation_data):
    """
    TODO: define the stimulation_data format here

    """

    for cortical_area in stimulation_data:
        if not is_memory_cortical_area(cortical_area=cortical_area):
            if stimulation_data[cortical_area]:
                neuron_list = set()
                for voxel in stimulation_data[cortical_area]:
                    if type(voxel) is list:
                        voxel = tuple(voxel)
                    in_the_block = neurons_in_the_block(cortical_area=cortical_area, block_ref=voxel)
                    for neuron in in_the_block:
                        neuron_list.add(neuron)
                for _ in neuron_list:
                    if cortical_area not in runtime_data.fire_candidate_list:
                        runtime_data.fire_candidate_list[cortical_area] = set()
                    runtime_data.fire_candidate_list[cortical_area].add(_)


# @staticmethod
# def godot_injector(stimulation_data):
#     for cortical_area in stimulation_data:
#         print("stimulating...", cortical_area)
#         neuron_list = set()
#         for voxel in stimulation_data[cortical_area]:
#             voxel = block_ref_2_id(voxel)
#             relative_coords = \
#                 runtime_data.genome['blueprint'][cortical_area].get('relative_coordinate')
#             cortical_block_ref = [voxel[0] - relative_coords[0],
#                                   voxel[1] - relative_coords[1],
#                                   voxel[2] - relative_coords[2]]
#             print("FEAGI received stimulation from Godot and processing...", cortical_block_ref)
#             in_the_block = neurons_in_the_block(cortical_area=cortical_area,
#                                                 block_ref=block_reference_builder(cortical_block_ref))
#             for neuron in in_the_block:
#                 neuron_list.add(neuron)
#         runtime_data.fcl_queue.put({cortical_area: neuron_list})
#         print(">>> >> >> > > >> >>>>>>> Stimulation data from Godot has been injected in FCL!")


def cortical_area_in_genome(cortical_area):
    if cortical_area not in runtime_data.cortical_list:
        return False
    else:
        return True


def battery_translator(sensor_data):
    """
    This module will provide the methods to receive information about embodiment battery level and have it passed along
     to the artificial brain.

    Battery level is broken down into 10% increments and be represented in the form of a range in a single cortical
    block with the dimensions of 1x1x10 where x represents the battery index, y is unused, and z reflects the range.
    In the event that the embodiment consists of multiple battery backs the x axis will be used to capture it e.g.
    4x2x5 for the case of four battery packs.

    Translates battery related data to neuronal activity

    todo: place the battery data format here

    sensor_data = {



    }

    """

    cortical_area = 'i__bat'
    if cortical_area_in_genome(cortical_area):
        if sensor_data is not None:
            for sensor in sensor_data:
                detections = stimuli_processor.range_to_coords(
                    cortical_area=cortical_area,
                    range_data=int(float(sensor_data[sensor]) * 100),
                    range_min=0, range_max=100, threshold=10)

                neurons = stimuli_processor.coords_to_neuron_ids(detections, cortical_area=cortical_area)
                # TODO: Add proximity feeder function in fcl_injector
                if 'i__bat' not in runtime_data.fire_candidate_list:
                    runtime_data.fire_candidate_list['i__bat'] = set()
                for neuron in neurons:
                    runtime_data.fire_candidate_list['i__bat'].add(neuron)
                # runtime_data.fcl_queue.put({'i__bat': set(neurons)})
    else:
        runtime_data.logs["PNS"].add(f"Warning! Cortical stimulation received but genome missing {cortical_area}")


def convert_ir_to_fire_list(ir_data):
    """

    The keys in ir_data correlate to the index id of each Infrared Sensor

    ir_data = {
        0: True,
        1: True,
        2: False
    }
    """
    cortical_area = 'i__inf'
    if cortical_area_in_genome(cortical_area):
        fire_list = set()
        inverse_fire_list = set()
        # print("runtime_data.brain['i__inf']", runtime_data.brain['i__inf'])

        active_ir_indexes = []
        inverse_ir_indexes = []

        for sensor_idx in ir_data:
            if ir_data[sensor_idx]:
                for key in runtime_data.brain[cortical_area]:
                    if sensor_idx == runtime_data.brain[cortical_area][key]['soma_location'][0]:
                        active_ir_indexes.append(sensor_idx)
                        fire_list.add(key)

        if 'i__inf' not in runtime_data.fire_candidate_list:
            runtime_data.fire_candidate_list['i__inf'] = set()

        # todo: find a generalized way to support inverse IPU concept
        if cortical_area_in_genome('ii_inf'):
            if 'ii_inf' not in runtime_data.fire_candidate_list:
                runtime_data.fire_candidate_list['ii_inf'] = set()
            for index in range(runtime_data.genome['blueprint']['ii_inf']["block_boundaries"][0]):
                if index not in active_ir_indexes:
                    inverse_ir_indexes.append(index)

            for inv_sensor_idx in inverse_ir_indexes:
                for neuron in runtime_data.brain['ii_inf']:
                    if inv_sensor_idx == runtime_data.brain['ii_inf'][neuron]['soma_location'][0]:
                        active_ir_indexes.append(inv_sensor_idx)
                        inverse_fire_list.add(neuron)

        for neuron in fire_list:
            runtime_data.fire_candidate_list['i__inf'].add(neuron)

        for neuron in inverse_fire_list:
            runtime_data.fire_candidate_list['ii_inf'].add(neuron)

        # runtime_data.fcl_queue.put({cortical_area: fire_list})
    else:
        runtime_data.logs["PNS"].add(f"Warning! Cortical stimulation received but genome missing {cortical_area}")


def training_translator(stimulation):
    cortical_area = "i___id"
    if stimulation is not None:
        inject_stimuli_to_fcl(cortical_area=cortical_area, stimulation=stimulation)


# def lidar_translator(proximity_data):
#     """
#     Translate the lidar messages based on its type.
#
#     todo: add details here about the message format and expectations
#
#
#     Type is not needed at this point given the lidar vs sonar data is automatically differentiated within the func.
#     """
#
#     cortical_area = 'i__pro'
#     if cortical_area_in_genome(cortical_area):
#         if proximity_data is not None:
#             # print("SLOT_TYPES", message.SLOT_TYPES)
#             # print("angle_increment:", message.angle_increment)
#             # print("angle_max:", message.angle_max)
#             # print("angle_min:", message.angle_min)
#             # print("get_fields_and_field_types:", message.get_fields_and_field_types)
#             # print("header:", message.header)
#             # print("intensities:", message.intensities)
#             # print("range_max:", message.range_max)
#             # print("range_min:", message.range_min)
#             # print("ranges:", message.ranges)
#             # print("scan_time:", message.scan_time)
#             # print("time_increment:", message.time_increment)
#             # print("-----")
#
#             for sensor in proximity_data:
#                 # differentiate between LIDAR/SONAR data
#                 if hasattr(proximity_data[sensor], '__iter__'):
#                     detections = stimuli_processor.lidar_to_coords(proximity_data[sensor])
#                 else:
#                     detections = stimuli_processor.sonar_to_coords(proximity_data[sensor])
#
#                 neurons = stimuli_processor.coords_to_neuron_ids(
#                     detections, cortical_area=cortical_area
#                 )
#                 # TODO: Add proximity feeder function in fcl_injector
#                 if 'i__pro' not in runtime_data.fire_candidate_list:
#                     runtime_data.fire_candidate_list['i__pro'] = set()
#                 for neuron in neurons:
#                     runtime_data.fire_candidate_list['i__pro'].add(neuron)
#                 # runtime_data.fcl_queue.put({cortical_area: set(neurons)})
#     else:
#         runtime_data.logs["PNS"].add(f"Warning! Cortical stimulation received but genome missing {cortical_area}")


def gyro_translator(gyroscope_data):
    """
    Translate the gyro messages based on its type.

    todo: add details here about the message format and expectations
    """
    cortical_area = 'i__gyr'
    if cortical_area_in_genome(cortical_area):
        if gyroscope_data is not None:
            r = gyroscope_data['0']
            y = gyroscope_data['1']
            p = gyroscope_data['2']
            holder_position = 0
            for i in r, y, p:
                detections = stimuli_processor.gyro_to_coords(i, holder_position)
                holder_position += 1
                neurons = stimuli_processor.coords_to_neuron_ids(detections, cortical_area=cortical_area)
                # TODO: Add proximity feeder function in fcl_injector
                if 'i__gyr' not in runtime_data.fire_candidate_list:
                    runtime_data.fire_candidate_list['i__gyr'] = set()
                for neuron in neurons:
                    runtime_data.fire_candidate_list['i__gyr'].add(neuron)
                # runtime_data.fcl_queue.put({cortical_area: set(neurons)})
    else:
        runtime_data.logs["PNS"].add(f"Warning! Cortical stimulation received but genome missing {cortical_area}")


def accelerator_translator(accelerator_data):
    """
    Translate the accelerator messages based on its type.

    todo: add details here about the message format and expectations
    """
    cortical_area = 'i__acc'
    if cortical_area_in_genome(cortical_area):
        if accelerator_data is not None:
            if '0' in accelerator_data and '1' in accelerator_data and '2' in accelerator_data:
                x = accelerator_data['0']
                y = accelerator_data['1']
                z = accelerator_data['2']
                holder_position = 0
                for i in x, y, z:
                    detections = stimuli_processor.accelerator_to_coords(i, holder_position)
                    holder_position += 1
                    neurons = stimuli_processor.coords_to_neuron_ids(detections, cortical_area=cortical_area)
                    if 'i__acc' not in runtime_data.fire_candidate_list:
                        runtime_data.fire_candidate_list['i__acc'] = set()
                    for neuron in neurons:
                        runtime_data.fire_candidate_list['i__acc'].add(neuron)
                    # runtime_data.fcl_queue.put({cortical_area: set(neurons)})
    else:
        runtime_data.logs["PNS"].add(f"Warning! Cortical stimulation received but genome missing {cortical_area}")


def servo_position_translator(servo_data):
    """
    Translate the servo messages based on its type.

    todo: add details here about the message format and expectations
    """
    cortical_area = 'i_spos'
    if cortical_area_in_genome(cortical_area):
        if servo_data is not None:
            numbers_of_servo = len(servo_data)
            for sensor in range(numbers_of_servo):
                sensor = sensor + 1
                position = stimuli_processor.servo_positions_to_coords(servo_data[sensor], sensor)
                neurons = stimuli_processor.coords_to_neuron_ids(
                    position, cortical_area=cortical_area
                )
                # TODO: Add proximity feeder function in fcl_injector
                if cortical_area not in runtime_data.fire_candidate_list:
                    runtime_data.fire_candidate_list[cortical_area] = set()
                for neuron in neurons:
                    runtime_data.fire_candidate_list[cortical_area].add(neuron)
                # runtime_data.fcl_queue.put({cortical_area: set(neurons)})
    else:
        runtime_data.logs["PNS"].add(f"Warning! Cortical stimulation received but genome missing {cortical_area}")


def encoder_translator(encoder_data=0):
    """
    Translate the encoder messages based on its type.

    todo: add details here about the message format and expectations
    """
    cortical_area = 'i__enc'
    if cortical_area_in_genome(cortical_area):
        if encoder_data is not None:
            numbers_of_servo = len(encoder_data)
            for sensor in range(numbers_of_servo):
                sensor = sensor + 1
                position = stimuli_processor.encoder_to_coords(encoder_data[sensor], sensor)
                neurons = stimuli_processor.coords_to_neuron_ids(
                    position, cortical_area=cortical_area
                )
                # TODO: Add proximity feeder function in fcl_injector
                if 'i__enc' not in runtime_data.fire_candidate_list:
                    runtime_data.fire_candidate_list['i__enc'] = set()
                for neuron in neurons:
                    runtime_data.fire_candidate_list['i__enc'].add(neuron)
                # runtime_data.fcl_queue.put({cortical_area: set(neurons)})
    else:
        runtime_data.logs["PNS"].add(f"Warning! Cortical stimulation received but genome missing {cortical_area}")


def encoder_speed_translator(encoder_speed_data):
    """
    Translate the encoder messages based on its type.

    todo: add details here about the message format and expectations
    """
    cortical_area = 'i__esp'
    if cortical_area_in_genome(cortical_area):
        if encoder_speed_data is not None:
            encoder_speed = stimuli_processor.encoder_speed_to_coords(encoder_speed_data)
            neurons = stimuli_processor.coords_to_neuron_ids(
                encoder_speed, cortical_area=cortical_area
            )
            # TODO: Add proximity feeder function in fcl_injector
            if 'i__esp' not in runtime_data.fire_candidate_list:
                runtime_data.fire_candidate_list['i__esp'] = set()
            for neuron in neurons:
                runtime_data.fire_candidate_list['i__esp'].add(neuron)
            # runtime_data.fcl_queue.put({cortical_area: set(neurons)})
    else:
        runtime_data.logs["PNS"].add(f"Warning! Cortical stimulation received but genome missing {cortical_area}")


def inject_stimuli_to_fcl(cortical_area, stimulation):
    if cortical_area in runtime_data.genome["blueprint"]:
        for pixel_coordinate in stimulation:
            neuron_ids = set()
            block_neurons = neurons_in_the_block(cortical_area, pixel_coordinate)
            neuron_ids.update(neuron for neuron in block_neurons if neuron is not None)
            for neuron in neuron_ids:
                update_membrane_potential_fire_queue(cortical_area=cortical_area,
                                                     neuron_id=neuron,
                                                     mp_update_amount=stimulation[pixel_coordinate],
                                                     fcl_insertion=True)


def vision_translator(vision_data):
    """
    Translate the accelerator messages based on its type.
    todo: adopt a coding mechanism for multi camera support

    old_Visio_data = {
        'TL': {}, 'TM': {}, 'TR': {}, 'ML': {},
        'C': {
            '22-56-2': 96, '23-56-2': 73, '19-55-0': 190,
            # ... [other data] ...
            '23-31-2': 121, '20-30-1': 188, '20-30-2': 177, '23-30-2': 98, '63-0-0': 144, '63-0-1': 114
        },
        'MR': {}, 'LL': {}, 'LM': {}, 'LR': {}
    }

    new_Visio_data = {
        '00TL': {}, '00TM': {}, '00TR': {}, '00ML': {},
        '00_C': {
            '22-56-2': 96, '23-56-2': 73, '19-55-0': 190,
            # ... [other data] ...
            '23-31-2': 121, '20-30-1': 188, '20-30-2': 177, '23-30-2': 98, '63-0-0': 144, '63-0-1': 114
        },
        '00MR': {}, '00LL': {}, '00LM': {}, '00LR': {}
    }

    Note the following:
    xx__ camera index
    __xx region index

    """

    if vision_data is not None:
        for data_packet_key in vision_data:
            if data_packet_key == 'C':
                print("! " * 20)
                print("!! Warning !! : Update controller to use new vision id formats...")
                print("! " * 20)
                cortical_area = "iv00_C"
            else:
                cortical_area = "iv" + data_packet_key
            inject_stimuli_to_fcl(cortical_area=cortical_area, stimulation=vision_data[data_packet_key])


def generic_ipu_translator(ipu_data):
    if ipu_data is not None:
        for cortical_area in ipu_data:
            if not is_memory_cortical_area(cortical_area=cortical_area):
                inject_stimuli_to_fcl(cortical_area=cortical_area, stimulation=ipu_data[cortical_area])
