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

from feagi_agent import pns_gateway as pns


def mode_testing(name_id, feagi_opu_channel, total, success, success_rate):
    ipu_name = list(name_id.keys())[0]
    message_from_feagi = pns.signals_from_feagi(feagi_opu_channel)
    if message_from_feagi is not None:
        data = pns.detect_ID_data(message_from_feagi)
        for key in data:
            print("COMPARING WITH ", key, " and ", ipu_name)
            total += 1
            if key == ipu_name:
                success += 1
            success_rate = (success / total) * 100
            print("success rate: ", success_rate, " success: ", success, " and total: ", total)
    return success_rate, success, total
