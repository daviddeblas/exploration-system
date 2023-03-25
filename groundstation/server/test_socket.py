import unittest
from unittest import mock
from unittest.mock import Mock, patch, MagicMock, AsyncMock
import asyncio
import time

from sockets import RobotCommunication, logger_task, handle_map_update


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
    
        
    def test_handle_map_update(self):
        # set up the sample and mock out the sio.emit method
        png_bytes = b'test_png_bytes'
        sample = Mock(payload=png_bytes)
        sio_emit_mock = AsyncMock()
        with patch('sockets.sio.emit', sio_emit_mock):

            # test that the map_update event is emitted with the correct payload
            handle_map_update(sample)
            sio_emit_mock.assert_called_with('map_update', png_bytes)

if __name__ == '__main__':
    unittest.main()
