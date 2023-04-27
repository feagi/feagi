def on_bluetooth_connected():
    basic.show_icon(IconNames.HEART)
bluetooth.on_bluetooth_connected(on_bluetooth_connected)

def on_bluetooth_disconnected():
    basic.show_leds("""
        # . . . #
                . # . # .
                . . # . .
                . # . # .
                # . . . #
    """)
bluetooth.on_bluetooth_disconnected(on_bluetooth_disconnected)

def on_uart_data_received():
    global feagi_string
    feagi_string = bluetooth.uart_read_until(serial.delimiters(Delimiters.HASH))
    if feagi_string == "f":
        cuteBot.move_time(cuteBot.Direction.FORWARD, 50, 0.5)
    elif feagi_string == "b":
        cuteBot.move_time(cuteBot.Direction.BACKWARD, 50, 0.5)
    elif feagi_string == "r":
        cuteBot.move_time(cuteBot.Direction.RIGHT, 50, 0.5)
    elif feagi_string == "l":
        cuteBot.move_time(cuteBot.Direction.LEFT, 50, 0.5)
bluetooth.on_uart_data_received(serial.delimiters(Delimiters.HASH), on_uart_data_received)

sound_level = ""
ultrasonic_data = 0
acc_z = ""
acc_y = ""
acc_x = ""
R_flag = ""
L_flag = ""
feagi_string = ""
negative_data = ""
bluetooth.start_uart_service()
basic.show_string("Hello!")

def on_forever():
    global L_flag, R_flag, acc_x, acc_y, acc_z, ultrasonic_data, sound_level
    if cuteBot.tracking(cuteBot.TrackingState.L_LINE_R_UNLINE) == True:
        L_flag = "t"
    elif cuteBot.tracking(cuteBot.TrackingState.L_LINE_R_UNLINE) == False:
        L_flag = "f"
    if cuteBot.tracking(cuteBot.TrackingState.L_UNLINE_R_LINE) == True:
        R_flag = "t"
    elif cuteBot.tracking(cuteBot.TrackingState.L_UNLINE_R_LINE) == False:
        R_flag = "f"
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
    sound = ""
    if db_level > 100:
        sound_level = "99"
    elif db_level < 10:
        sound_level = '0' + str(db_level)
    else:
        sound_level = str(db_level)
    ultrasonic_data = cuteBot.ultrasonic(cuteBot.SonarUnit.CENTIMETERS)
    if ultrasonic_data > 99:
        ultrasonic_data = 99
    elif ultrasonic_data < 0:
        ultrasonic_data = 0
    bluetooth.uart_write_string("" + L_flag + R_flag + acc_x + acc_y + acc_z + ultrasonic_data + str(sound_level) )
    basic.pause(1000)
basic.forever(on_forever)
