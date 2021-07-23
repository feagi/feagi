
from opu.destination.output_matrix import motor_list
from inf import runtime_data




def motor_operator(motor_brand, motor_model, motor_id, speed, power):
    if motor_brand == "Freenove":
        from feagi-core.3rd_party import arduino




