from pymycobot import MyCobotSocket

# Port 9000 is used by default
mc = MyCobotSocket("192.168.10.10", "9000")

mc.connect("/dev/ttyAMA0", "1000000")

res = mc.get_angles()
print(res)

mc.send_angles([0, 0, 0, 0, 0, 0], 20)
