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
pub = session.declare_publisher('identify')

@sio.event
async def connect(sid, environ, auth):
    print(f'{sid}: connected')
    await sio.emit('join', {'sid': sid})

@sio.event
async def identify(data, _):
    pub.put("identify")


@sio.event
async def disconnect(sid):
    print(f'{sid}: disconnected')