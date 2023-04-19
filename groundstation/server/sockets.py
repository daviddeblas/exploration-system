import socketio
import zenoh
import asyncio
import time
from constants import TIMEOUT_ROBOT, BATTERY_CHARGE_100, LEVEL_TOO_LOW
import database
import asyncio
import models
import datetime
import json
import pytz

logger_queue = asyncio.Queue()
in_mission_sem = asyncio.Semaphore()
mission = None
map_rover = None
map_drone = None

sio = socketio.AsyncServer(
    async_mode='asgi',
    cors_allowed_origins="*"
)

sio_app = socketio.ASGIApp(
    socketio_server=sio,
    socketio_path=''
)

session = zenoh.open()
pub_identify = session.declare_publisher('identify')
pub_start_rover = session.declare_publisher('start_rover')
pub_start_drone = session.declare_publisher('start_drone')
pub_finish = session.declare_publisher('finish')
return_home_finish = session.declare_publisher('return_home')
return_home_rover = session.declare_publisher('return_home_rover')
p2p_trigger = session.declare_publisher('p2p')


def log_sub(sample):
    message = sample.payload.decode('utf-8')
    logger_queue.put_nowait(message)


logger_sub = session.declare_subscriber("logger", log_sub)


async def logger_task():
    global mission
    while True:
        message = await logger_queue.get()
        if mission is None:
            continue
        data = message.split(";;", 2)
        db = database.SessionLocal()
        log_entry = models.LogEntry(
            mission_id=mission.id,
            time=datetime.datetime.now(),
            robot=data[0],
            category=data[1],
            data=data[2])
        db.add(log_entry)
        db.commit()
        db.refresh(log_entry)
        db.close()

        await sio.emit('logger', json.dumps(log_entry.as_dict(), default=str))


is_sim = False


def set_is_sim_true(sample):
    global is_sim
    is_sim = True


is_sim_sub = session.declare_subscriber("simulation_mission", set_is_sim_true)


async def in_mission_task():
    global mission
    global is_sim
    global map_rover
    global map_drone

    etc_timezone = pytz.timezone('US/Eastern')
    while True:
        await in_mission_sem.acquire()

        in_mission = False
        if rover.in_mission is not None or drone.in_mission is not None:
            in_mission = rover.in_mission == "True" or drone.in_mission == "True"

        if mission is not None:
            if rover.in_mission == "True":
                mission.has_rover = True
            if drone.in_mission == "True":
                mission.has_drone = True
            asyncio.create_task(
                sio.emit('mission_update', json.dumps(mission.as_dict(), default=str)))

        if mission is None and in_mission:
            mission = models.Mission(
                start=datetime.datetime.now(tz=etc_timezone))
            db = database.SessionLocal()
            db.add(mission)
            db.commit()
            db.refresh(mission)
            db.close()
        elif mission is not None and not in_mission:
            mission.end = datetime.datetime.now(tz=etc_timezone)
            mission.distance_drone = drone.distance_traveled
            mission.distance_rover = rover.distance_traveled
            mission.is_sim = is_sim
            mission.map_rover = map_rover
            mission.map_drone = map_drone
            asyncio.create_task(
                sio.emit('mission_update', json.dumps(mission.as_dict(), default=str)))
            db = database.SessionLocal()
            db.add(mission)
            db.commit()
            db.close()
            mission = None
            is_sim = False


class TaskManager:
    def __init__(self):
        self.tasks = []

    async def start(self):
        self.tasks = [
            asyncio.create_task(rover.send_robot_state()),
            asyncio.create_task(drone.send_robot_state()),
            asyncio.create_task(logger_task()),
            asyncio.create_task(in_mission_task()),
        ]


task_manager = None


@sio.event
async def connect(sid, environ, auth):
    global task_manager
    print(f'{sid}: connected')
    logger_queue.put_nowait(f"groundstation;;connect;;{sid}")
    if task_manager is None:
        task_manager = TaskManager()
        await task_manager.start()


class RobotCommunication:
    def __init__(self, name):
        self.name = name
        self.in_mission = None
        self.last_updated = None
        self.distance_traveled = 0.0
        self.sub = session.declare_subscriber(
            f'{self.name}_state', self.robot_state)
        self.dist_sub = session.declare_subscriber(
            f'{self.name}_distance_traveled', self.robot_distance_traveled)
        self.battery = BATTERY_CHARGE_100
        self.sub_battery = session.declare_subscriber(
            f'{self.name}_battery', self.battery_state)

    async def send_robot_state(self):
        while True:
            if self.in_mission is not None:
                await sio.emit(f'{self.name}_state', self.in_mission)
                await asyncio.sleep(1)

                if time.time() - self.last_updated > TIMEOUT_ROBOT:
                    self.in_mission = None
            else:
                await sio.emit(f'{self.name}_state', self.in_mission)
                await asyncio.sleep(1)

    def robot_state(self, sample):
        self.in_mission = sample.payload.decode('utf-8')
        self.last_updated = time.time()
        in_mission_sem.release()

    def robot_distance_traveled(self, sample):
        self.distance_traveled = float(sample.payload.decode('utf-8'))

    def battery_state(self, sample):
        battery = sample.payload.decode('utf-8')
        try:
            battery = int(sample.payload.decode('utf-8'))
        except (ValueError, AttributeError):
            return
        self.battery = battery

        asyncio.run(sio.emit(f'{self.name}_battery', self.battery))

        if (self.battery <= LEVEL_TOO_LOW and self.name == "rover"):
            return_home_rover.put("return_home_rover")

    def get_battery(self):
        return self.battery


def handle_map_update(sample):
    global map_rover
    png_bytes = sample.payload
    asyncio.run(sio.emit('map_update', png_bytes))
    map_rover = png_bytes


def handle_map_cognifly_update(sample):
    global map_drone
    png_bytes = sample.payload
    asyncio.run(sio.emit('map_cognifly_update', png_bytes))
    map_drone = png_bytes


rover = RobotCommunication('rover')
drone = RobotCommunication('drone')
sub_map_updates = session.declare_subscriber('map_image', handle_map_update)
sub_map_cognifly_updates = session.declare_subscriber(
    'map_image_cognifly', handle_map_cognifly_update)


@ sio.event
async def identify(data, _):
    pub_identify.put("identify")
    logger_queue.put_nowait("groundstation;;identify;;")


@ sio.event
async def start(data, _):
    if (rover.get_battery() <= LEVEL_TOO_LOW and drone.get_battery() <= LEVEL_TOO_LOW):
        return

    elif (rover.get_battery() <= LEVEL_TOO_LOW):
        pub_start_drone.put("start_drone")
        logger_queue.put_nowait("groundstation;;start;;")

    elif (drone.get_battery() <= LEVEL_TOO_LOW):
        pub_start_rover.put("start_rover")
        logger_queue.put_nowait("groundstation;;start;;")

    else:
        pub_start_rover.put("start_rover")
        pub_start_drone.put("start_drone")
        logger_queue.put_nowait("groundstation;;start;;")


@ sio.event
async def finish(data, _):
    pub_finish.put("finish")
    logger_queue.put_nowait("groundstation;;finish;;")


@ sio.event
async def return_home(data, _):
    return_home_finish.put("return_home")


@ sio.event
async def p2p(data, _):
    p2p_trigger.put("p2p")


@ sio.event
async def disconnect(sid):
    print(f'{sid}: disconnected')
    logger_queue.put_nowait(f"groundstation;;disconnect;;{sid}")
