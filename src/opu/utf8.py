# Copyright (c) 2019 Mohammad Nadji-Tehrani <m.nadji.tehrani@gmail.com>
from inf import runtime_data

"""
Library of Output Proecssing Unit (OPU) functions responsible for translating neuronal activities into a read world 
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
