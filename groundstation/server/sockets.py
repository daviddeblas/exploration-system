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

def log_sub(sample):
    message = sample.payload.decode('utf-8')
    logger_queue.put_nowait(message)


logger_sub = session.declare_subscriber("logger", log_sub)


async def logger_task():
    while True:
        message = await logger_queue.get()
        data = message.split(";;", 2)
        db = database.SessionLocal()
        log_entry = models.LogEntry(
            mission_id=models.mission.id,
            time=datetime.datetime.now(),
            robot=data[0],
            category=data[1],
            data=data[2])
        db.add(log_entry)
        db.commit()
        db.refresh(log_entry)
        db.close()

        await sio.emit('logger', json.dumps(log_entry.as_dict(), default=str))


@sio.event
async def connect(sid, environ, auth):
    print(f'{sid}: connected')
    asyncio.create_task(rover.send_robot_state())
    asyncio.create_task(drone.send_robot_state())
    asyncio.create_task(logger_task())
    logger_queue.put_nowait(f"groundstation;;connect;;{sid}")


class RobotCommunication:
    def __init__(self, name):
        self.name = name
        self.in_mission = None
        self.last_updated = None
        self.sub = session.declare_subscriber(
            f'{self.name}_state', self.robot_state)

    async def send_robot_state(self):
        while True:
            if self.in_mission is not None:
                await sio.emit(f'{self.name}_state', eval(self.in_mission))
                await asyncio.sleep(1)

                if time.time() - self.last_updated > TIMEOUT_ROBOT:
                    self.in_mission = None
            else:
                await sio.emit(f'{self.name}_state', self.in_mission)
                await asyncio.sleep(1)

    def robot_state(self, sample):
        self.in_mission = sample.payload.decode('utf-8')
        self.last_updated = time.time()


def handle_map_update(sample):
    png_bytes = sample.payload
    asyncio.run(sio.emit('map_update', png_bytes))

rover = RobotCommunication('rover')
drone = RobotCommunication('drone')
sub_map_updates = session.declare_subscriber('map_image', handle_map_update)

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
async def disconnect(sid):
    print(f'{sid}: disconnected')
    logger_queue.put_nowait(f"groundstation;;disconnect;;{sid}")
