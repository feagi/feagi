#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Copyright 2016-2022 The FEAGI Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================
"""

from time import sleep
from datetime import datetime
from feagi_agent import pns_gateway as pns
from feagi_agent.version import __version__
from feagi_agent import feagi_interface as feagi
import traceback
from configuration import *
import os


def PRINT_FROM_IPU(message_from_feagi):
    print(message_from_feagi)


def pass_ipu_data_to_training(data):
    return data


if __name__ == "__main__":
    # Generate runtime dictionary
    runtime_data = {"vision": {}, "current_burst_id": None, "stimulation_period": None,
                    "feagi_state": None,
                    "feagi_network": None}
    print("retrying...")
    FEAGI_FLAG = False
    print("Waiting on FEAGI...")
    while not FEAGI_FLAG:
        FEAGI_FLAG = feagi.is_FEAGI_reachable(
            os.environ.get('FEAGI_HOST_INTERNAL', feagi_settings["feagi_host"]),
            int(os.environ.get('FEAGI_OPU_PORT', "3000")))
        sleep(2)
    # # # FEAGI registration # # # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    # - - - - - - - - - - - - - - - - - - #
    feagi_settings, runtime_data, api_address, feagi_ipu_channel, feagi_opu_channel = \
        feagi.connect_to_feagi(feagi_settings, runtime_data, agent_settings, capabilities,
                               __version__)
    # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    msg_counter = runtime_data["feagi_state"]['burst_counter']
    previous_genome_timestamp = 0

    if not pns.full_list_dimension:
        pns.full_list_dimension = pns.fetch_full_dimensions(
            api_address + '/v1/feagi/connectome/properties/dimensions')
        print(pns.full_list_dimension)
    genome_tracker = 0  # Temporarily

    while True:
        try:
            message_from_feagi = pns.efferent_signaling(feagi_opu_channel)
            if message_from_feagi is not None:
                # # Checking on refresh rate and genome status
                genome_changed = pns.detect_genome_change(message_from_feagi)
                if genome_changed != previous_genome_timestamp:
                    pns.full_list_dimension = pns.fetch_full_dimensions(
                        api_address + '/v1/feagi/connectome/properties/dimensions')
                    previous_genome_timestamp = message_from_feagi["genome_changed"]
                current_tracker = pns.obtain_genome_number(genome_tracker, message_from_feagi)
                if genome_tracker != current_tracker:
                    pns.full_list_dimension = pns.fetch_full_dimensions(
                        api_address + '/v1/feagi/connectome/properties/dimensions')
                    genome_tracker = current_tracker
                # # End

            message_to_feagi = pns.prepare_the_feagi_data('training',
                                                          pass_ipu_data_to_training({"0-0-0": 100}),
                                                          message_to_feagi)
            message_to_feagi['timestamp'] = datetime.now()
            message_to_feagi['counter'] = msg_counter
            feagi_settings['feagi_burst_speed'] = pns.check_refresh_rate(message_from_feagi,
                                                                         feagi_settings[
                                                                             'feagi_burst_speed'])
            pns.afferent_signaling(message_to_feagi, feagi_ipu_channel, agent_settings)
        except Exception as e:
            # pass
            print("ERROR! : ", e)
            traceback.print_exc()
            break
