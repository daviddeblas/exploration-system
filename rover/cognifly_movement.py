from cognifly import Cognifly
import time
import zenoh
import math
import os

MAX_VELOCITY = 0.25
MAX_YAW = 1.0
BATTERY_CHARGE_100 = 100
LEVEL_TOO_LOW = 30
DURATION = 10.0
HEIGHT = 1.0
INITIAL_VALUE = 0.0
MAX_DISTANCE = 2.0
SIDEWAYS_INCREMENT = 0.3
MIN_BATTERY = 6.4
MAX_BATTERY = 8.4

session = zenoh.open()

class MoveCognifly:
    def __init__(self) -> None:
        self.cf = Cognifly(
            drone_hostname=os.environ["COGNIFLY_HOSTNAME"], gui=False)
        self.finishing_mission = False
        self.started_mission = False
        self.battery = BATTERY_CHARGE_100
        self.COMMANDS = [
            lambda: self.cf.set_position_nonblocking(x=INITIAL_VALUE, y=INITIAL_VALUE, z=HEIGHT, yaw=INITIAL_VALUE,
                                                     max_velocity=MAX_VELOCITY, max_yaw_rate=MAX_YAW, max_duration=DURATION, relative=False),
            lambda: self.cf.set_position_nonblocking(x=MAX_DISTANCE, y=INITIAL_VALUE, z=HEIGHT, yaw=INITIAL_VALUE,
                                                     max_velocity=MAX_VELOCITY, max_yaw_rate=MAX_YAW, max_duration=DURATION, relative=False),
            lambda: self.cf.set_position_nonblocking(x=MAX_DISTANCE, y=SIDEWAYS_INCREMENT, z=HEIGHT, yaw=INITIAL_VALUE,
                                                     max_velocity=MAX_VELOCITY, max_yaw_rate=MAX_YAW, max_duration=DURATION, relative=False),
            lambda: self.cf.set_position_nonblocking(x=INITIAL_VALUE, y=SIDEWAYS_INCREMENT, z=HEIGHT, yaw=INITIAL_VALUE,
                                                     max_velocity=MAX_VELOCITY, max_yaw_rate=MAX_YAW, max_duration=DURATION, relative=False),
            lambda: self.cf.set_position_nonblocking(x=INITIAL_VALUE, y=SIDEWAYS_INCREMENT*2, z=HEIGHT, yaw=INITIAL_VALUE,
                                                     max_velocity=MAX_VELOCITY, max_yaw_rate=MAX_YAW, max_duration=DURATION, relative=False),
            lambda: self.cf.set_position_nonblocking(x=MAX_DISTANCE, y=SIDEWAYS_INCREMENT*2, z=HEIGHT, yaw=INITIAL_VALUE,
                                                     max_velocity=MAX_VELOCITY, max_yaw_rate=MAX_YAW, max_duration=DURATION, relative=False),
            lambda: self.cf.set_position_nonblocking(x=MAX_DISTANCE, y=HEIGHT, z=HEIGHT, yaw=INITIAL_VALUE,
                                                     max_velocity=MAX_VELOCITY, max_yaw_rate=MAX_YAW, max_duration=DURATION, relative=False),
            lambda: self.cf.set_position_nonblocking(x=INITIAL_VALUE, y=SIDEWAYS_INCREMENT*3, z=HEIGHT, yaw=INITIAL_VALUE,
                                                     max_velocity=MAX_VELOCITY, max_yaw_rate=MAX_YAW, max_duration=DURATION, relative=False),
            lambda: self.cf.set_position_nonblocking(x=INITIAL_VALUE, y=INITIAL_VALUE, z=HEIGHT, yaw=INITIAL_VALUE,
                                                     max_velocity=MAX_VELOCITY, max_yaw_rate=MAX_YAW, max_duration=DURATION, relative=False),
        ]

    def start_mission(self):
        if self.started_mission or self.battery <= LEVEL_TOO_LOW or self.finishing_mission:
            return
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
            time.sleep(DURATION)
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
        if not self.started_mission:
            return
        self.finishing_mission = True
        self.started_mission = False
        self.cf.set_position_nonblocking(x=INITIAL_VALUE, y=INITIAL_VALUE, z=HEIGHT, yaw=INITIAL_VALUE,
                                         max_velocity=MAX_VELOCITY, max_yaw_rate=MAX_YAW, max_duration=DURATION, relative=False),
        time.sleep(10)
        self.cf.land_nonblocking(),
        time.sleep(2)
        self.cf.disarm()
        self.finishing_mission = False

    def get_battery(self):
        voltage = float(self.cf.get_telemetry()[-2])
        percentage = int((voltage - MIN_BATTERY) / (MAX_BATTERY - MIN_BATTERY) * BATTERY_CHARGE_100)
        self.battery = percentage
        return self.battery
