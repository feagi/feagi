import time
import random
import itertools
import pycozmo

from PIL import Image, ImageDraw
from typing import Optional, Tuple
from pycozmo import protocol_encoder

WIDTH = 128
HEIGHT = 32
MAX_SPEED = 2
NUM_DOTS = 3
DOT_SIZE = 1
LINE_WIDTH = 1


def drive_wheels(self, lwheel_speed: float, rwheel_speed: float,
                 lwheel_acc: Optional[float] = 0.0, rwheel_acc: Optional[float] = 0.0,
                 duration: Optional[float] = None) -> None:
    pkt = protocol_encoder.DriveWheels(lwheel_speed_mmps=lwheel_speed,
                                       rwheel_speed_mmps=rwheel_speed,
                                       lwheel_accel_mmps2=0.0, rwheel_accel_mmps2=0.0)
    self.conn.send(pkt)


def stop_motor(self):
    self.stop_all_motors()


class Dot(object):
    def __init__(self, x: int, y: int, vx: int, vy: int):
        self.x = x
        self.y = y
        self.vx = vx
        self.vy = vy


def display_lines(cli):
    # Generate random dots.
    dots = []
    for i in range(NUM_DOTS):
        x = random.randint(0, WIDTH)
        y = random.randint(0, HEIGHT)
        vx = random.randint(-MAX_SPEED, MAX_SPEED)
        vy = random.randint(-MAX_SPEED, MAX_SPEED)
        dot = Dot(x, y, vx, vy)
        dots.append(dot)

    timer = pycozmo.util.FPSTimer(pycozmo.robot.FRAME_RATE)
    start = time.time()
    while (time.time() - start) < 20.0:
        # Create a blank image.
        im = Image.new("1", (128, 32), color=0)

        # Draw lines.
        draw = ImageDraw.Draw(im)
        for a, b in itertools.combinations(dots, 2):
            draw.line((a.x, a.y, b.x, b.y), width=LINE_WIDTH, fill=1)

        # Move dots.
        for dot in dots:
            dot.x += dot.vx
            dot.y += dot.vy
            if dot.x <= DOT_SIZE:
                dot.x = DOT_SIZE
                dot.vx = abs(dot.vx)
            elif dot.x >= WIDTH - DOT_SIZE:
                dot.x = WIDTH - DOT_SIZE
                dot.vx = -abs(dot.vx)
            if dot.y <= DOT_SIZE:
                dot.y = DOT_SIZE
                dot.vy = abs(dot.vy)
            elif dot.y >= HEIGHT - DOT_SIZE:
                dot.y = HEIGHT - DOT_SIZE
                dot.vy = -abs(dot.vy)

        cli.display_image(im)

        # Run with 30 FPS.
        timer.sleep()
