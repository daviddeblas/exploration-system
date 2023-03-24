import datetime
import json
import time
import unittest
import asyncio
from unittest import mock
import zenoh
from unittest.mock import patch, MagicMock
from sockets import sio_app, rover, drone, handle_map_update, pubIdentify, pubStart, pubFinish, returnHomeFinish, logger_queue, RobotCommunication, sio, logger_task
import socketio

class TestLoggerTask(unittest.TestCase):
    def setUp(self):
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)

        self.queue = asyncio.Queue()

    def tearDown(self):
        self.loop.close()

    @patch('sockets.database.SessionLocal')
    @patch('sockets.models.LogEntry')
    async def test_logger_task(self, mock_LogEntry, mock_SessionLocal):
        message = 'test_robot;;test_category;;test_data'
        self.queue.put_nowait(message)

        db = MagicMock()
        mock_SessionLocal.return_value = db
        log_entry = MagicMock()
        mock_LogEntry.return_value = log_entry
        await logger_task()

        mock_SessionLocal.assert_called_once()
        db.add.assert_called_once()
        db.commit.assert_called_once()
        db.refresh.assert_called_once()
        self.assertTrue(mock_LogEntry.called)
        log_entry.as_dict.assert_called_once()
        self.assertEqual(self.queue.qsize(), 0)
class TestSocketIO(unittest.TestCase):

    async def emit_sio_event(self, event_name, data=None):
        sid = "test_sid"
        with patch.object(socketio.AsyncServer, 'emit') as mock_emit:
            await sio._trigger_event_handlers(event_name, sid, data)
            mock_emit.assert_called_once_with(event_name, data, room=sid)

    async def setUp(self):
        self.session = zenoh.open()
        self.pubIdentify = self.session.declare_publisher('identify')
        self.pubStart = self.session.declare_publisher('start')
        self.pubFinish = self.session.declare_publisher('finish')
        self.returnHomeFinish = self.session.declare_publisher('return_home')
        self.logger_sub = self.session.declare_subscriber("logger")

        self.rover = RobotCommunication('rover')
        self.drone = RobotCommunication('drone')
        self.subMapUpdates = self.session.declare_subscriber('map_image')

        self.loop = asyncio.get_running_loop()

    async def tearDown(self):
        self.session.close()
        await sio.disconnect_all()

    async def test_connect(self):
        await self.emit_sio_event('connect')
        await asyncio.sleep(0.1)
        message = await self.logger_sub.take()
        self.assertEqual(message.payload.decode('utf-8'), "groundstation;;connect;;test_sid")

    async def test_identify(self):
        await self.emit_sio_event('identify')
        message = await self.pubIdentify.take()
        self.assertEqual(message.payload.decode('utf-8'), "identify")

        await asyncio.sleep(0.1)
        message = await self.logger_sub.take()
        self.assertEqual(message.payload.decode('utf-8'), "groundstation;;identify;;")

    async def test_start(self):
        await self.emit_sio_event('start')
        message = await self.pubStart.take()
        self.assertEqual(message.payload.decode('utf-8'), "start")

        await asyncio.sleep(0.1)
        message = await self.logger_sub.take()
        self.assertEqual(message.payload.decode('utf-8'), "groundstation;;start;;")

    async def test_finish(self):
        await self.emit_sio_event('finish')
        message = await self.pubFinish.take()
        self.assertEqual(message.payload.decode('utf-8'), "finish")

        await asyncio.sleep(0.1)
        message = await self.logger_sub.take()
        self.assertEqual(message.payload.decode('utf-8'), "groundstation;;finish;;")

    async def test_return_home(self):
        await self.emit_sio_event('return_home')
        message = await self.returnHomeFinish.take()
        self.assertEqual(message.payload.decode('utf-8'), "return_home")

    async def test_disconnect(self):
        await self.emit_sio_event('disconnect')
        await asyncio.sleep(0.1)
        message = await self.logger_sub.take()
        self.assertEqual(message.payload.decode('utf-8'), "groundstation;;disconnect;;test_sid")
        
    async def test_rover_communication(self):
        rover = RobotCommunication('rover')
        rover.in_mission = 'move(1, 2, 3)'
        rover.last_updated = time.time() - 2

        with mock.patch('socketio.AsyncServer.emit') as mock_emit:
            asyncio.run(rover.send_robot_state())
            mock_emit.assert_called_with('rover_state', 'move(1, 2, 3)')

        rover.in_mission = None

        with mock.patch('socketio.AsyncServer.emit') as mock_emit:
            asyncio.run(rover.send_robot_state())
            mock_emit.assert_called_with('rover_state', None)


class TestSockets(unittest.TestCase):
    
    def setUp(self):
        self.test_client = TestClient(sio_app)
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        
    def tearDown(self):
        self.loop.close()

    @patch('sockets.sio.emit')
    async def test_identify_event(self, mock_emit):
        data = {}
        await self.test_client.emit('identify', data)
        pubIdentify.assert_called_once()
        logger_queue.put_nowait.assert_called_once_with("groundstation;;identify;;")

    @patch('sockets.sio.emit')
    async def test_start_event(self, mock_emit):
        data = {}
        await self.test_client.emit('start', data)
        pubStart.assert_called_once()
        logger_queue.put_nowait.assert_called_once_with("groundstation;;start;;")

    @patch('sockets.sio.emit')
    async def test_finish_event(self, mock_emit):
        data = {}
        await self.test_client.emit('finish', data)
        pubFinish.assert_called_once()
        logger_queue.put_nowait.assert_called_once_with("groundstation;;finish;;")

    @patch('sockets.sio.emit')
    async def test_return_home_event(self, mock_emit):
        data = {}
        await self.test_client.emit('return_home', data)
        returnHomeFinish.assert_called_once()

    @patch('sockets.sio.emit')
    async def test_rover_send_robot_state(self, mock_emit):
        rover.in_mission = True
        await rover.send_robot_state()
        mock_emit.assert_called_once_with('rover_state', eval(rover.in_mission))

    @patch('sockets.sio.emit')
    async def test_drone_send_robot_state(self, mock_emit):
        drone.in_mission = None
        await drone.send_robot_state()
        mock_emit.assert_called_once_with('drone_state', None)

    @patch('sockets.sio.emit')
    async def test_handle_map_update(self, mock_emit):
        png_bytes = b'123'
        sample = MagicMock(payload=png_bytes)
        await handle_map_update(sample)
        mock_emit.assert_called_once_with('map_update', png_bytes)

class TestClient:
    def __init__(self, app):
        self.app = app

    async def emit(self, event, data):
        response = await self.app(scope={
            'type': 'test',
            'http_version': '1.1',
            'method': 'POST',
            'headers': [],
            'path': '/socket.io/',
            'query_string': b'',
            'raw_path': '/socket.io/',
            'client': ('127.0.0.1', 5000),
            'server': ('127.0.0.1', 5000),
            'asgi': {
                'version': '3.0',
                'spec_version': '2.1',
            },
        }, receive=asyncio.Queue().get, send=MagicMock())
        return response
