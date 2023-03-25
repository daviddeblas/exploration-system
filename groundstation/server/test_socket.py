import unittest
from unittest import mock
from unittest.mock import Mock, patch, AsyncMock
import asyncio
import time
from sockets import RobotCommunication, handle_map_update, identify, start, finish, return_home


class TestRobotCommunication(unittest.IsolatedAsyncioTestCase):

    @mock.patch('sockets.sio.emit')
    async def test_send_robot_state(self, mock_emit):
        rover = RobotCommunication('rover')
        rover.in_mission = 'True'
        rover.last_updated = time.time()
        try:
            await asyncio.wait_for(rover.send_robot_state(), timeout=2)
        except asyncio.exceptions.TimeoutError:
            mock_emit.assert_called_with('rover_state', True)
        except Exception as e:
            raise e
        
    async def test_robot_state(self):
        rover = RobotCommunication('rover')
        sample = Mock(payload=b'True')
        rover.robot_state(sample)
        assert rover.in_mission == 'True'
        assert rover.last_updated is not None

    def test_handle_map_update(self):
        png_bytes = b'test_png_bytes'
        sample = Mock(payload=png_bytes)
        sio_emit_mock = AsyncMock()
        with patch('sockets.sio.emit', sio_emit_mock):

            handle_map_update(sample)
            sio_emit_mock.assert_called_with('map_update', png_bytes)
            
class TestSocketEvents(unittest.IsolatedAsyncioTestCase):
    async def test_identify(self):
        sid = '1234'
        data = 'test_data'
        with patch('sockets.pub_identify.put') as mock_pub_identify:
            with patch('sockets.logger_queue.put_nowait') as mock_logger_queue:
                await identify(data, sid)
                mock_pub_identify.assert_called_once_with('identify')
                mock_logger_queue.assert_called_once_with('groundstation;;identify;;')

    async def test_start(self):
        sid = '1234'
        data = 'test_data'
        with patch('sockets.pub_start.put') as mock_pub_start:
            with patch('sockets.logger_queue.put_nowait') as mock_logger_queue:
                await start(data, sid)
                mock_pub_start.assert_called_once_with('start')
                mock_logger_queue.assert_called_once_with('groundstation;;start;;')

    async def test_finish(self):
        sid = '1234'
        data = 'test_data'
        with patch('sockets.pub_finish.put') as mock_pub_finish:
            with patch('sockets.logger_queue.put_nowait') as mock_logger_queue:
                await finish(data, sid)
                mock_pub_finish.assert_called_once_with('finish')
                mock_logger_queue.assert_called_once_with('groundstation;;finish;;')

    async def test_return_home(self):
        sid = '1234'
        data = 'test_data'
        with patch('sockets.return_home_finish.put') as mock_return_home_finish:
            await return_home(data, sid)
            mock_return_home_finish.assert_called_once_with('return_home')


if __name__ == '__main__':
    unittest.main()