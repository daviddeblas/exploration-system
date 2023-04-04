from cognifly import Cognifly
import time

MAX_VELOCITY = 0.25
MAX_YAW = 1.0


class MoveCognifly:
    def __init__(self) -> None:
        self.cf = Cognifly(drone_hostname="Cognifly2.lan", gui=False)
        self.finishing_mission = False
        self.started_mission = False
        self.battery = 100
        self.COMMANDS = [
            # lambda : self.cf.set_position_nonblocking(x=0.0, y=0.0, z=1.0, yaw=0.0,
            #                 max_velocity=MAX_VELOCITY, max_yaw_rate=MAX_YAW, max_duration=10.0, relative=False),
            # lambda : self.cf.set_position_nonblocking(x=2.0, y=0.0, z=1.0, yaw=0.0,
            #                 max_velocity=MAX_VELOCITY, max_yaw_rate=MAX_YAW, max_duration=10.0, relative=False),
            # lambda : self.cf.set_position_nonblocking(x=2.0, y=-0.3, z=1.0, yaw=0.0,
            #                 max_velocity=MAX_VELOCITY, max_yaw_rate=MAX_YAW, max_duration=10.0, relative=False),
            # lambda : self.cf.set_position_nonblocking(x=0.0, y=-0.3, z=1.0, yaw=0.0,
            #                 max_velocity=MAX_VELOCITY, max_yaw_rate=MAX_YAW, max_duration=10.0, relative=False),
            # lambda : self.cf.set_position_nonblocking(x=0.0, y=-0.6, z=1.0, yaw=0.0,
            #                 max_velocity=MAX_VELOCITY, max_yaw_rate=MAX_YAW, max_duration=10.0, relative=False),
            # lambda : self.cf.set_position_nonblocking(x=2.0, y=-0.6, z=1.0, yaw=0.0,
            #                 max_velocity=MAX_VELOCITY, max_yaw_rate=MAX_YAW, max_duration=10.0, relative=False),
            # lambda : self.cf.set_position_nonblocking(x=2.0, y=-0.9, z=1.0, yaw=0.0,
            #                 max_velocity=MAX_VELOCITY, max_yaw_rate=MAX_YAW, max_duration=10.0, relative=False),
            # lambda : self.cf.set_position_nonblocking(x=0.0, y=-0.9, z=1.0, yaw=0.0,
            #                 max_velocity=MAX_VELOCITY, max_yaw_rate=MAX_YAW, max_duration=10.0, relative=False),
            # lambda : self.cf.set_position_nonblocking(x=0.0, y=0.0, z=1.0, yaw=0.0,
            #                 max_velocity=MAX_VELOCITY, max_yaw_rate=MAX_YAW, max_duration=10.0, relative=False),
            lambda: self.cf.land_nonblocking(),
            lambda: self.cf.disarm()
        ]

    def start_mission(self):
        if self.started_mission or self.battery <= 30:
            return
        self.finishing_mission = False
        self.started_mission = True
        self.cf.arm()
        time.sleep(2.0)
        self.cf.takeoff_nonblocking()
        time.sleep(5)
        for command in self.COMMANDS:
            if self.finishing_mission:
                return
            if self.battery <= 30:
                self.finish_mission()
                return
            command()
            time.sleep(10)
        self.cf.land_nonblocking()
        time.sleep(5)
        self.cf.disarm()

    def identify_cognifly(self):
        if not self.started_mission and not self.finishing_mission:
            self.cf.arm()
            time.sleep(2)
            self.cf.disarm()

    def finish_mission(self):
        self.finishing_mission = True
        self.cf.set_position_nonblocking(x=0.0, y=0.0, z=1.0, yaw=0.0,
                                         max_velocity=MAX_VELOCITY, max_yaw_rate=MAX_YAW, max_duration=10.0, relative=False),
        time.sleep(10)
        self.cf.land_nonblocking(),
        time.sleep(2)
        self.cf.disarm()
        self.finishing_mission = False
        self.started_mission = False

    def get_battery(self):
        voltage = float(self.cf.get_telemetry[-2])
        percentage = int((voltage - 6) / (8.4 - 6) * 100)
        self.battery = percentage
        return self.battery
