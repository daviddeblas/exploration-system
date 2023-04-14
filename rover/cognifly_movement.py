from cognifly import Cognifly
import time
import zenoh
import math
import os

MAX_VELOCITY = 0.25
MAX_YAW = 1.0
BATTERY_CHARGE_100 = 100
LEVEL_TOO_LOW = 30

session = zenoh.open()


class MoveCognifly:
    def __init__(self) -> None:
        self.cf = Cognifly(
            drone_hostname=os.environ["COGNIFLY_HOSTNAME"], gui=False)
        self.finishing_mission = False
        self.started_mission = False
        self.battery = BATTERY_CHARGE_100
        self.COMMANDS = [
            lambda: self.cf.set_position_nonblocking(x=0.0, y=0.0, z=0.5, yaw=0.0,
                                                     max_velocity=MAX_VELOCITY, max_yaw_rate=MAX_YAW, max_duration=10.0, relative=False),
            lambda: self.cf.set_position_nonblocking(x=2.0, y=0.0, z=0.5, yaw=0.0,
                                                     max_velocity=MAX_VELOCITY, max_yaw_rate=MAX_YAW, max_duration=10.0, relative=False),
            lambda: self.cf.set_position_nonblocking(x=2.0, y=0.3, z=0.5, yaw=0.0,
                                                     max_velocity=MAX_VELOCITY, max_yaw_rate=MAX_YAW, max_duration=10.0, relative=False),
            lambda: self.cf.set_position_nonblocking(x=0.0, y=0.3, z=0.5, yaw=0.0,
                                                     max_velocity=MAX_VELOCITY, max_yaw_rate=MAX_YAW, max_duration=10.0, relative=False),
            lambda: self.cf.set_position_nonblocking(x=0.0, y=0.6, z=0.5, yaw=0.0,
                                                     max_velocity=MAX_VELOCITY, max_yaw_rate=MAX_YAW, max_duration=10.0, relative=False),
            lambda: self.cf.set_position_nonblocking(x=2.0, y=0.6, z=0.5, yaw=0.0,
                                                     max_velocity=MAX_VELOCITY, max_yaw_rate=MAX_YAW, max_duration=10.0, relative=False),
            lambda: self.cf.set_position_nonblocking(x=2.0, y=1.0, z=0.5, yaw=0.0,
                                                     max_velocity=MAX_VELOCITY, max_yaw_rate=MAX_YAW, max_duration=10.0, relative=False),
            lambda: self.cf.set_position_nonblocking(x=0.0, y=1.0, z=0.5, yaw=0.0,
                                                     max_velocity=MAX_VELOCITY, max_yaw_rate=MAX_YAW, max_duration=10.0, relative=False),
            lambda: self.cf.set_position_nonblocking(x=0.0, y=0.0, z=0.5, yaw=0.0,
                                                     max_velocity=MAX_VELOCITY, max_yaw_rate=MAX_YAW, max_duration=10.0, relative=False),
        ]

    def start_mission(self):
        if self.started_mission or self.battery <= LEVEL_TOO_LOW:
            return
        self.finishing_mission = False
        self.started_mission = True
        self.cf.arm()
        time.sleep(2)
        self.cf.takeoff_nonblocking()
        time.sleep(5)
        for command in self.COMMANDS:
            if self.finishing_mission:
                return
            if self.battery <= LEVEL_TOO_LOW:
                self.finish_mission()
                return
            command()
            time.sleep(10)
        self.cf.land_nonblocking()
        time.sleep(5)
        self.cf.disarm()

    def identify_cognifly(self):
        global session
        session.declare_publisher('cognifly_id').put('identify')

    def is_crashed(self):
        telemetry = self.cf.get_telemetry()
        return "BLOCKED_UAV_NOT_LEVEL" in telemetry[-1]

    def distance_calculation(self):
        cognifly_position = self.cf.get_position()
        return math.sqrt(cognifly_position[0]**2 + cognifly_position[1]**2)

    def finish_mission(self):
        self.finishing_mission = True
        self.cf.set_position_nonblocking(x=0.0, y=0.0, z=0.5, yaw=0.0,
                                         max_velocity=MAX_VELOCITY, max_yaw_rate=MAX_YAW, max_duration=10.0, relative=False),
        time.sleep(10)
        self.cf.land_nonblocking(),
        time.sleep(2)
        self.cf.disarm()
        self.finishing_mission = False
        self.started_mission = False

    def get_battery(self):
        voltage = float(self.cf.get_telemetry()[-2])
        percentage = int((voltage - 6.4) / (8.4 - 6.4) * 100)
        self.battery = percentage
        return self.battery

    def is_crashed(self):
        telemetry = self.cf.get_telemetry()
        return "BLOCKED_UAV_NOT_LEVEL" in telemetry[-1]
