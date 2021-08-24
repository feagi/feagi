

# from opu.destination.output_matrix import motor_list
from inf import runtime_data

# hw_brand = runtime_data.genome['species']['brand']
# hw_model = runtime_data.genome['species']['model']
hw_brand = 'freenove'
hw_model = 'smart_car'

print("Operating on %s %s" % (hw_model, hw_brand))

import_path  = '../third_party/' + hw_brand + '/' + hw_model
print("IMPORT PATH: ", import_path)

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
except Exception as e:
    # print("Error: Some Exception occurred")
    print(e)
