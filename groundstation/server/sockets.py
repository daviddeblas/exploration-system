import socketio
import zenoh
import asyncio
import time

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


in_mission = None
last_updated = None
TIMEOUT = 4


@sio.event
async def connect(sid, environ, auth):
    print(f'{sid}: connected')
    asyncio.create_task(send_robot_state())


@sio.event
async def send_robot_state():
    global in_mission, last_updated
    while True:
        if (in_mission is not None):
            await sio.emit('robot_state', eval(in_mission))
            await asyncio.sleep(1)

            # Vérifie si le robot est toujours connecté, si ce n'est pas le cas, on met à jour l'état du robot
            if time.time() - last_updated > TIMEOUT:
                in_mission = None

        elif (in_mission == None):
            await sio.emit('robot_state', in_mission)
            await asyncio.sleep(1)


def update_robot_state(new_data):
    global in_mission, last_updated
    in_mission = new_data
    last_updated = time.time()


def robot_state(sample):
    global msg
    msg = sample.payload.decode('utf-8')
    update_robot_state(msg)


sub1 = session.declare_subscriber('robot_state', robot_state)


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
