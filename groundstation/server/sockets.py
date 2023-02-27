import socketio
import zenoh
import asyncio
import time
from constants import TIMEOUT_ROBOT

sio = socketio.AsyncServer(
    async_mode='asgi',
    cors_allowed_origins=[]
)

sio_app = socketio.ASGIApp(
    socketio_server=sio,
    socketio_path=''
)

session = zenoh.open()
pubIdentify = session.declare_publisher('identify')
pubStart = session.declare_publisher('start')
pubFinish = session.declare_publisher('finish')


@sio.event
async def connect(sid, environ, auth):
    print(f'{sid}: connected')
    asyncio.create_task(rover.send_robot_state())
    asyncio.create_task(drone.send_robot_state())


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


rover = RobotCommunication('rover')
drone = RobotCommunication('drone')


@ sio.event
async def identify(data, _):
    pubIdentify.put("identify")


@ sio.event
async def start(data, _):
    pubStart.put("start")


@ sio.event
async def finish(data, _):
    pubFinish.put("finish")


@ sio.event
async def disconnect(sid):
    print(f'{sid}: disconnected')
