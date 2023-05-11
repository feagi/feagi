def on_bluetooth_connected():
    basic.show_string("2")
bluetooth.on_bluetooth_connected(on_bluetooth_connected)

def on_bluetooth_disconnected():
    basic.show_string("0")
bluetooth.on_bluetooth_disconnected(on_bluetooth_disconnected)

def on_uart_data_received():
    global feagi_string, direction, power, timer
    feagi_string = bluetooth.uart_read_until(serial.delimiters(Delimiters.HASH))
    direction = feagi_string[0]
    power = int(feagi_string.slice(1, 3))
    timer = int(feagi_string.slice(3))
    if direction == "f":
        cuteBot.move_time(cuteBot.Direction.FORWARD, power, 0.5)
    elif direction == "b":
        cuteBot.move_time(cuteBot.Direction.BACKWARD, power, 0.5)
    elif direction == "r":
        cuteBot.move_time(cuteBot.Direction.RIGHT, power, 0.5)
    elif direction == "l":
        cuteBot.move_time(cuteBot.Direction.LEFT, power, 0.5)
bluetooth.on_uart_data_received(serial.delimiters(Delimiters.HASH), on_uart_data_received)

ultrasonic_data = 0
sound_level = ""
db_level = 0
acc_z = ""
acc_y = ""
acc_x = ""
R_flag = ""
L_flag = ""
timer = 0
power = 0
direction = ""
sound = ""
negative_data = ""
feagi_string = ""
bluetooth.start_uart_service()
basic.show_string("1")

def on_forever():
    global L_flag, R_flag, acc_x, acc_y, acc_z, db_level, sound_level, ultrasonic_data
    if cuteBot.tracking(cuteBot.TrackingState.L_LINE_R_UNLINE) == True:
        L_flag = "t"
    elif cuteBot.tracking(cuteBot.TrackingState.L_LINE_R_UNLINE) == False:
        L_flag = "f"
    if cuteBot.tracking(cuteBot.TrackingState.L_UNLINE_R_LINE) == True:
        R_flag = "t"
    elif cuteBot.tracking(cuteBot.TrackingState.L_UNLINE_R_LINE) == False:
        R_flag = "f"
    if cuteBot.tracking(cuteBot.TrackingState.L_R_LINE) == True:
        R_flag = "t"
        L_flag = "t"
    acc_x = "" + str((int(input.acceleration(Dimension.X)) + 1000))
    while len(acc_x) < 4:
        acc_x = "0" + acc_x
    acc_y = "" + str((int(input.acceleration(Dimension.Y)) + 1000))
    while len(acc_y) < 4:
        acc_y = "0" + acc_y
    acc_z = "" + str((int(input.acceleration(Dimension.Z)) + 1000))
    while len(acc_z) < 4:
        acc_z = "0" + acc_z
    db_level = input.sound_level()
    if db_level > 100:
        sound_level = "99"
    elif db_level < 10:
        sound_level = "0" + ("" + str(db_level))
    else:
        sound_level = "" + str(db_level)
    ultrasonic_data = cuteBot.ultrasonic(cuteBot.SonarUnit.CENTIMETERS)
    if ultrasonic_data > 99:
        ultrasonic_data = 99
    elif ultrasonic_data < 0:
        ultrasonic_data = 0
    bluetooth.uart_write_string("" + L_flag + R_flag + acc_x + acc_y + acc_z + ("" + str(ultrasonic_data)) + sound_level)
basic.forever(on_forever)
