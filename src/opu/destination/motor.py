

from opu.destination.output_matrix import motor_list
from inf import runtime_data

hw_brand = runtime_data.genome['species']['brand']
hw_model = runtime_data.genome['species']['model']

print("Operating on %s %s" % (hw_model, hw_brand))

import_path  = '../third_part/' + hw_brand + '/' + hw_model

try:
    import time
    controller = __import__(import_path)

    def motor_operator(motor_brand, motor_model, motor_id, speed, power):
        print("Operating a motor on %s %s" % (hw_model, hw_brand))
        try:
            motor = controller.Motor()
            # todo: Generalize the following section. using specifics for test only

            motor.M3F()
            time.sleep(3)
            motor.stop()
        except:
            print("ERROR: Requested controller not available for %s %s" % (hw_model, hw_brand))
except:
    print("Error: Some Exception occurred")
