import pytest
from evo import neuroembryogenesis
from inf import initialize, runtime_data


def test_feagi_initialization():
    # ensure runtime_data parameters are updated upon initialization
    initialize.initialize()
    assert runtime_data.brain_run_id
    assert runtime_data.parameters
    assert runtime_data.working_directory
    assert runtime_data.connectome_path
    
    # confirm genome creation using static genome
    assert runtime_data.parameters['Switches']['use_static_genome']
    assert runtime_data.genome
    assert runtime_data.genome_id

    # suppress connectome figure generation during testing
    runtime_data.parameters['Visualization']['connectome_visualizer'] = False

    # ensure completion of brain development following neuroembryogenesis
    neuroembryogenesis.develop_brain(reincarnation_mode=False)
    assert runtime_data.brain
