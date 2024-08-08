from enum import Enum
from typing import Optional, Literal, List
from pydantic import BaseModel, Field, conint, conlist, validator
from fastapi.staticfiles import StaticFiles
from src.inf import runtime_data


class Launch(BaseModel):
    existing_connectome: Optional[str] = ''


class Logs(BaseModel):
    print_cortical_activity_counters: Optional[bool]
    print_burst_info: Optional[bool]
    print_messenger_logs: Optional[bool]
    print_brain_gen_activities: Optional[bool]


class BurstEngine(BaseModel):
    burst_duration: Optional[float]
    burst_duration = 1


class MorphologyProperties(BaseModel):
    name: str
    type: str
    parameters: dict


class NewCorticalProperties(BaseModel):
    cortical_type: str
    cortical_id: str
    coordinates_2d: list
    coordinates_3d: list
    dev_count: Optional[int] = 1


class NewRegionProperties(BaseModel):
    title: str
    region_description: Optional[str]
    parent_region_id: str = Field(default="root")
    coordinates_2d: List[int] = Field(default=[0, 0])
    coordinates_3d: List[int] = Field(default=[0, 0, 0])
    areas: Optional[list] = Field(default=[])
    regions: Optional[list] = Field(default=[])


class UpdateRegionProperties(BaseModel):
    region_id: str
    title: Optional[str]
    region_description: Optional[str]
    parent_region_id: Optional[str]
    coordinates_2d: Optional[list]
    coordinates_3d: Optional[list]


class RegionAssociation(BaseModel):
    id: str = Field(default="")
    new_region_id: str = Field(default="")


class NewCustomCorticalProperties(BaseModel):
    cortical_name: str = Field(..., max_length=20, min_length=1)
    parent_region_id: Optional[str] = Field(default="root")
    coordinates_2d: Optional[list] = [0, 0]
    coordinates_3d: List[int] = Field(default=[0, 0, 0])
    cortical_dimensions: List[int] = Field(default=[1, 1, 1])
    sub_group_id: Optional[str] = ""
    copy_of: Optional[str] = ""


# class NewCorticalProperties_old(BaseModel):
#     cortical_id: str = Field(None, max_length=6, min_length=6)
#     cortical_name: str
#     cortical_group: str
#     cortical_neuron_per_vox_count: int
#     cortical_visibility: bool
#     cortical_coordinates: dict = {
#         'x': 0,
#         'y': 0,
#         'z': 0,
#     }
#     cortical_dimensions: dict = {
#         'x': 1,
#         'y': 1,
#         'z': 1,
#     }
#     cortical_destinations: dict = {
#     }
#     cortical_synaptic_attractivity: int
#     neuron_post_synaptic_potential: float
#     neuron_post_synaptic_potential_max: float
#     neuron_fire_threshold: float
#     neuron_refractory_period: int
#     neuron_leak_coefficient: float
#     neuron_leak_variability: int
#     neuron_consecutive_fire_count: int
#     neuron_snooze_period: int
#     neuron_degeneracy_coefficient: float
#     neuron_psp_uniform_distribution: bool


class UpdateCorticalProperties(BaseModel):
    cortical_id: str = Field(None, max_length=6, min_length=6)
    cortical_name: Optional[str] = None
    parent_region_id: Optional[str] = None
    cortical_neuron_per_vox_count: Optional[int] = None
    cortical_visibility: Optional[bool] = None
    coordinates_2d: Optional[conlist(int, min_items=2, max_items=2)] = None
    coordinates_3d: Optional[conlist(int, min_items=3, max_items=3)] = None
    cortical_dimensions: Optional[conlist(int, min_items=3, max_items=3)] = None
    cortical_dimensions_per_dev: Optional[conlist(int, min_items=3, max_items=3)] = None
    cortical_synaptic_attractivity: Optional[float] = None
    neuron_post_synaptic_potential: Optional[float] = None
    neuron_post_synaptic_potential_max: Optional[float] = None
    neuron_fire_threshold: Optional[float] = None
    neuron_fire_threshold_increment: Optional[list] = None
    neuron_firing_threshold_limit: Optional[float] = None
    neuron_refractory_period: Optional[int] = None
    neuron_leak_coefficient: Optional[float] = None
    neuron_leak_variability: Optional[float] = None
    neuron_consecutive_fire_count: Optional[int] = None
    neuron_snooze_period: Optional[int] = None
    neuron_degeneracy_coefficient: Optional[float] = None
    neuron_psp_uniform_distribution: Optional[bool] = None
    neuron_mp_charge_accumulation: Optional[bool] = None
    neuron_mp_driven_psp: Optional[bool] = None
    neuron_longterm_mem_threshold: Optional[int] = None
    neuron_lifespan_growth_rate: Optional[int] = None
    neuron_init_lifespan: Optional[int] = None
    neuron_excitability: Optional[float] = None
    dev_count: Optional[int] = None

    @validator('cortical_dimensions', 'cortical_dimensions_per_dev', each_item=True)
    def check_positive(cls, value):
        if value is not None and value <= 0:
            raise ValueError('All coordinates must be positive integers greater than 0')
        return value


class UpdateMultipleCorticalProperties(BaseModel):
    cortical_id_list: list
    parent_region_id: Optional[str] = None
    cortical_neuron_per_vox_count: Optional[int] = None
    cortical_visibility: Optional[bool] = None
    cortical_dimensions: Optional[conlist(int, min_items=3, max_items=3)] = None
    cortical_synaptic_attractivity: Optional[float] = None
    neuron_post_synaptic_potential: Optional[float] = None
    neuron_post_synaptic_potential_max: Optional[float] = None
    neuron_fire_threshold: Optional[float] = None
    neuron_fire_threshold_increment: Optional[list] = None
    neuron_firing_threshold_limit: Optional[float] = None
    neuron_refractory_period: Optional[int] = None
    neuron_leak_coefficient: Optional[float] = None
    neuron_leak_variability: Optional[float] = None
    neuron_consecutive_fire_count: Optional[int] = None
    neuron_snooze_period: Optional[int] = None
    neuron_degeneracy_coefficient: Optional[float] = None
    neuron_psp_uniform_distribution: Optional[bool] = None
    neuron_mp_charge_accumulation: Optional[bool] = None
    neuron_mp_driven_psp: Optional[bool] = None
    neuron_longterm_mem_threshold: Optional[int] = None
    neuron_lifespan_growth_rate: Optional[int] = None
    neuron_init_lifespan: Optional[int] = None
    neuron_excitability: Optional[float] = None

    @validator('cortical_dimensions', each_item=True)
    def check_positive(cls, value):
        if value is not None and value <= 0:
            raise ValueError('All coordinates must be positive integers greater than 0')
        return value

# class Network(BaseModel):
#     godot_host: Optional[str] = runtime_data.parameters['Sockets']['godot_host_name']
#     godot_data_port: Optional[int] = runtime_data.parameters['Sockets']['feagi_inbound_port_godot']
#     godot_web_port: Optional[int] = 6081
#     gazebo_host: Optional[str] = runtime_data.parameters['Sockets']['gazebo_host_name']
#     gazebo_data_port: Optional[int] = runtime_data.parameters['Sockets']['feagi_inbound_port_gazebo']
#     gazebo_web_port: Optional[int] = 6080


class CorticalId(BaseModel):
    cortical_id: str


class CorticalIdList(BaseModel):
    cortical_id_list: list


class CorticalName(BaseModel):
    cortical_name: str


class MorphologyName(BaseModel):
    morphology_name: str


class VizSkipRate(BaseModel):
    cortical_viz_skip_rate: int = runtime_data.cortical_viz_skip_rate


class VizThreshold(BaseModel):
    visualization_threshold: int = runtime_data.cortical_viz_sup_threshold


class MorphologyInput(BaseModel):
    morphology_name: str
    morphology_type: str
    morphology_parameters: dict


class UpdateCorticalMappingProperties(BaseModel):
    src_cortical_area: str
    dst_cortical_area: str
    mapping_string: list


class SuggestedMapping(BaseModel):
    brain_region_id: str
    mapping_type: str
    mapping_definitions: list = [{}]


class CorticalAreaSrcDst(BaseModel):
    src_cortical_area: str
    dst_cortical_area: str


class ConnectomePath(BaseModel):
    connectome_path: str


class Connectome(BaseModel):
    cortical_area: str


class Registration(BaseModel):
    source: str
    host: str
    capabilities: dict


class AgentRegistration(BaseModel):
    agent_type: str
    agent_id: str
    agent_data_port: int
    agent_version: str
    controller_version: str
    capabilities: dict = None


class Stats(BaseModel):
    neuron_stat_collection: Optional[bool] = False
    synapse_stat_collection: Optional[bool] = False


class Stimulation(BaseModel):
    stimulation_script: dict


class Shock(BaseModel):
    shock: tuple


class Intensity(BaseModel):
    intensity: conint(ge=0, le=9)


class SPAStaticFiles(StaticFiles):
    async def get_response(self, path: str, scope):
        response = await super().get_response(path, scope)
        print("<><><><><><>")
        if response.status_code == 404:
            print("-=-=-=-=-=-=-=-=-=-=")
            response = await super().get_response('.', scope)
        return response


class Subscriber(BaseModel):
    subscriber_address: str


class Id(BaseModel):
    id: str


class FitnessStats(BaseModel):
    fitness_stats: dict


class RobotController(BaseModel):
    motor_power_coefficient: float
    motor_power_coefficient = 0.5
    robot_starting_position: dict
    robot_starting_position = {
        0: [0, 0, 0],
        1: [0, 1, 1],
        2: [0, 0, 1],
        3: [0, 2, 1],
        4: [0, 1, 2]
    }


class RobotModel(BaseModel):
    robot_sdf_file_name: str
    robot_sdf_file_name = ""

    robot_sdf_file_name_path: str
    robot_sdf_file_name_path = "./lib/gazebo/robots/4WD_smart_car/"

    gazebo_floor_img_file: str
    gazebo_floor_img_file = ""

    gazebo_floor_img_file_path: str
    gazebo_floor_img_file_path = "./lib/gazebo/gazebo_maps/"

    mu: float
    mu = 1.0

    mu2: float
    mu2 = 50.0

    fdir: list
    fdir = [1, 1, 1]

    slip1: float
    slip1 = 1.0

    slip2: float
    slip2 = 1.0


class AmalgamationRequest(BaseModel):
    genome_id: Optional[str] = None
    genome_title: Optional[str] = None
    genome_payload: Optional[dict] = None


class RewiringMode(str, Enum):
    rewire_all = "all"
    rewire_system = "system"
    rewire_none = "none"


class UserPreferences(BaseModel):
    adv_mode: bool
    ui_magnification: float = 1.0
