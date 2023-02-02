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
key = 'some_key'
pub = session.declare_publisher(key)

@sio.event
async def connect(sid, environ, auth):
    print(f'{sid}: connected')
    await sio.emit('join', {'sid': sid})

@sio.event
async def beep(data, _):
    print('Sending beep with zenoh!')
    pub.put("beep")


@sio.event
async def disconnect(sid):
    print(f'{sid}: disconnected')