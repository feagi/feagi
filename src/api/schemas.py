from typing import Optional, Literal, List
from pydantic import BaseModel, Field, conint
from fastapi.staticfiles import StaticFiles


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
    channel_count: Optional[int]


class NewCustomCorticalProperties(BaseModel):
    cortical_name: str = Field(None, max_length=20, min_length=1)
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
    cortical_name: Optional[str]
    cortical_neuron_per_vox_count: Optional[int]
    cortical_visibility: Optional[bool]
    cortical_coordinates: Optional[list]
    cortical_coordinates_2d: Optional[list]
    cortical_dimensions: Optional[list]
    cortical_synaptic_attractivity: Optional[int]
    neuron_post_synaptic_potential: Optional[float]
    neuron_post_synaptic_potential_max: Optional[float]
    neuron_fire_threshold: Optional[float]
    neuron_fire_threshold_increment: Optional[list]
    neuron_firing_threshold_limit: Optional[float]
    neuron_refractory_period: Optional[int]
    neuron_leak_coefficient: Optional[float]
    neuron_leak_variability: Optional[float]
    neuron_consecutive_fire_count: Optional[int]
    neuron_snooze_period: Optional[int]
    neuron_degeneracy_coefficient: Optional[float]
    neuron_psp_uniform_distribution: Optional[bool]
    neuron_mp_charge_accumulation: Optional[bool]
    neuron_mp_driven_psp: Optional[bool]
    neuron_longterm_mem_threshold: Optional[int]
    neuron_lifespan_growth_rate: Optional[int]
    neuron_init_lifespan: Optional[int]


# class Network(BaseModel):
#     godot_host: Optional[str] = runtime_data.parameters['Sockets']['godot_host_name']
#     godot_data_port: Optional[int] = runtime_data.parameters['Sockets']['feagi_inbound_port_godot']
#     godot_web_port: Optional[int] = 6081
#     gazebo_host: Optional[str] = runtime_data.parameters['Sockets']['gazebo_host_name']
#     gazebo_data_port: Optional[int] = runtime_data.parameters['Sockets']['feagi_inbound_port_gazebo']
#     gazebo_web_port: Optional[int] = 6080


class ConnectomePath(BaseModel):
    connectome_path: str


class Connectome(BaseModel):
    cortical_area: str


class Registration(BaseModel):
    source: str
    host: str
    capabilities: dict


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
