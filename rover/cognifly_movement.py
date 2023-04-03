from cognifly import Cognifly
import time

MAX_VELOCITY=0.25
MAX_YAW=1.0

class MoveCognifly:
    def __init__(self) -> None:
        self.cf = Cognifly(drone_hostname="Cognifly2.lan", gui=False)
        self.finishing_mission = False
        self.started_mission = False
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
            lambda : self.cf.land_nonblocking(),
            lambda : self.cf.disarm()
        ]
    
    def start_mission(self):
        if self.started_mission:
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
    
    def is_crashed(self):
        telemetry = self.cf.get_telemetry()
        return "BLOCKED_UAV_NOT_LEVEL" in telemetry[-1]
