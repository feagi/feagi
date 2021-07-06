# Copyright (c) 2019 Mohammad Nadji-Tehrani <m.nadji.tehrani@gmail.com>
def neuron_update(float presynaptic_current,
                  int burst_count,
                  int last_membrane_potential_update,
                  float leak_coefficient,
                  float membrane_potential):
    """
    This function's only task is to update the membrane potential of the destination neuron.
    """

    # Leaky behavior
    if leak_coefficient > 0:
        if last_membrane_potential_update < burst_count:
            leak_window = burst_count - last_membrane_potential_update
            leak_value = leak_window * leak_coefficient
            membrane_potential -= leak_value
            if membrane_potential < 0:
                membrane_potential = 0

    # Increasing the cumulative counter on destination based on the received signal from upstream Axon
    # The following is considered as LTP or Long Term Potentiation of Neurons

    membrane_potential += presynaptic_current

    return membrane_potential
