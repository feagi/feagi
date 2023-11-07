from pycozmo import protocol_encoder
import time
from typing import Optional, Tuple


def drive_wheels(self, lwheel_speed: float, rwheel_speed: float,
                 lwheel_acc: Optional[float] = 0.0, rwheel_acc: Optional[float] = 0.0,
                 duration: Optional[float] = None) -> None:
    pkt = protocol_encoder.DriveWheels(lwheel_speed_mmps=lwheel_speed,
                                       rwheel_speed_mmps=rwheel_speed,
                                       lwheel_accel_mmps2=0.0, rwheel_accel_mmps2=0.0)
    self.conn.send(pkt)


def stop_motor(self):
    self.stop_all_motors()
