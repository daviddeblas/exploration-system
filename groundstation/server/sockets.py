import socketio
import zenoh

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
    await sio.emit('join', {'sid': sid})

    
@sio.event
async def identify(data, _):
    pubIdentify.put("identify")

@sio.event
async def start(data, _):
    pubStart.put("start")
    
@sio.event
async def finish(data, _):
    pubFinish.put("finish")

@sio.event
async def disconnect(sid):
    print(f'{sid}: disconnected')
