import socketio
import zenoh
import asyncio
import time
from constants import TIMEOUT_ROBOT
import database
import asyncio
import models
import datetime
import json

logger_queue = asyncio.Queue()
in_mission_sem = asyncio.Semaphore()
mission = None

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
pub_start = session.declare_publisher('start')
pub_finish = session.declare_publisher('finish')
return_home_finish = session.declare_publisher('return_home')
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


async def in_mission_task():
    global mission
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

        if mission is None and in_mission:
            mission = models.Mission(start=datetime.datetime.now())
            db = database.SessionLocal()
            db.add(mission)
            db.commit()
            db.refresh(mission)
            db.close()
        elif mission is not None and not in_mission:
            mission.end = datetime.datetime.now()
            mission.distance_drone = drone.distance_traveled
            mission.distance_rover = rover.distance_traveled
            db = database.SessionLocal()
            db.add(mission)
            db.commit()
            db.close()
            mission = None


@sio.event
async def connect(sid, environ, auth):
    print(f'{sid}: connected')
    logger_queue.put_nowait(f"groundstation;;connect;;{sid}")


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


def handle_map_update(sample):
    png_bytes = sample.payload
    asyncio.run(sio.emit('map_update', png_bytes))

def handle_map_cognifly_update(sample):
    png_bytes = sample.payload
    asyncio.run(sio.emit('map_cognifly_update', png_bytes))


rover = RobotCommunication('rover')
drone = RobotCommunication('drone')
sub_map_updates = session.declare_subscriber('map_image', handle_map_update)
sub_map_cognifly_updates = session.declare_subscriber('map_image_cognifly', handle_map_cognifly_update)

asyncio.create_task(rover.send_robot_state())
asyncio.create_task(drone.send_robot_state())
asyncio.create_task(logger_task())
asyncio.create_task(in_mission_task())


@ sio.event
async def identify(data, _):
    pub_identify.put("identify")
    logger_queue.put_nowait("groundstation;;identify;;")


@ sio.event
async def start(data, _):
    pub_start.put("start")
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
