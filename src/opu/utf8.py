

# Copyright 2016-2022 The FEAGI Authors. All Rights Reserved.
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

from inf import runtime_data

"""
Output Processing Unit (OPU) functions responsible for translating neuronal activities into a read world 
event.
"""


def convert_neuron_activity_to_utf8_char(cortical_area, neuron_id):
    char = int(runtime_data.brain[cortical_area][neuron_id]['soma_location'][0][2])
    activity_history = list(runtime_data.brain[cortical_area][neuron_id]['activity_history'])
    # todo: move collection span to parameters
    collection_span_counter = len(activity_history) - 1
    membrane_potential_total = 0
    while collection_span_counter > 0:
        membrane_potential_total += activity_history[collection_span_counter][1]
        collection_span_counter -= 1

    activity_rank = membrane_potential_total / len(activity_history)
    return chr(char), int(activity_rank)


if __name__ == '__main__':
    import tkinter
    master = tkinter.Tk()

    text = "Comprehended Character is: " + runtime_data.parameters["Input"]["opu_char"]
    tkinter.Label(master, text=text, font=("Helvetica", 24)).grid(row=0)

    # master.update_idletasks()
    # master.update()

    master.mainloop()
