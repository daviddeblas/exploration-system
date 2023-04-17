import unittest
from unittest import mock
from unittest.mock import Mock, patch, AsyncMock, ANY
import asyncio
import time
from sockets import RobotCommunication, handle_map_update, identify, start, finish, return_home, connect, disconnect, logger_task, logger_queue
from constants import BATTERY_CHARGE_100, LEVEL_TOO_LOW
import warnings
import models


class TestRobotCommunication(unittest.IsolatedAsyncioTestCase):

    @ mock.patch('sockets.sio.emit')
    async def test_send_robot_state(self, mock_emit):
        rover = RobotCommunication('rover')
        rover.in_mission = 'True'
        rover.last_updated = time.time()
        try:
            await asyncio.wait_for(rover.send_robot_state(), timeout=2)
        except asyncio.exceptions.TimeoutError:
            mock_emit.assert_called_with('rover_state', 'True')
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

    def test_battery_state(self):
        warnings.filterwarnings("ignore")
        rover = RobotCommunication('rover')
        sample = Mock(payload=b'80')
        with patch('sockets.sio.emit', new_callable=AsyncMock) as sio_emit_mock:
            rover.battery_state(sample)
            sio_emit_mock.assert_called_with('rover_battery', 80)
        self.assertEqual(rover.get_battery(), 80)

    def test_battery_state_invalid_value(self):
        rover = RobotCommunication('rover')
        sample = Mock(payload=b'invalid_value')
        with patch('sockets.sio.emit', new_callable=AsyncMock) as mock_sio_emit:
            rover.battery_state(sample)
            mock_sio_emit.assert_not_called()
        self.assertEqual(rover.get_battery(), BATTERY_CHARGE_100)

    def test_battery_state_too_low(self):
        rover = RobotCommunication('rover')
        sample = Mock(payload=b'10')
        with patch('sockets.sio.emit', new_callable=AsyncMock) as mock_sio_emit:
            with patch('sockets.return_home_rover.put') as mock_return_home_rover:
                rover.battery_state(sample)
                mock_sio_emit.assert_called_with('rover_battery', 10)
                mock_return_home_rover.assert_called_once_with(
                    'return_home_rover')
        self.assertEqual(rover.get_battery(), 10)


class TestSocketEvents(unittest.IsolatedAsyncioTestCase):
    async def test_identify(self):
        sid = '1234'
        data = 'test_data'
        with patch('sockets.pub_identify.put') as mock_pub_identify:
            with patch('sockets.logger_queue.put_nowait') as mock_logger_queue:
                await identify(data, sid)
                mock_pub_identify.assert_called_once_with('identify')
                mock_logger_queue.assert_called_once_with(
                    'groundstation;;identify;;')

    async def test_start(self):
        sid = '1234'
        data = 'test_data'

        with patch('sockets.rover.get_battery') as mock_rover_battery, \
                patch('sockets.drone.get_battery') as mock_drone_battery, \
                patch('sockets.pub_start_rover.put') as mock_pub_start_rover, \
                patch('sockets.pub_start_drone.put') as mock_pub_start_drone, \
                patch('sockets.logger_queue.put_nowait') as mock_logger_queue:

            # Test du rover avec la batterie trop basse (inférieur à 30%)
            mock_rover_battery.return_value = LEVEL_TOO_LOW - 1
            mock_drone_battery.return_value = BATTERY_CHARGE_100
            await start(data, sid)
            mock_pub_start_drone.assert_called_once_with(
                'start_drone')
            mock_logger_queue.assert_called_with(
                'groundstation;;start;;')
            mock_pub_start_rover.reset_mock()
            mock_pub_start_drone.reset_mock()
            mock_logger_queue.reset_mock()

            # Test du drone avec la batterie trop basse (inférieur à 30%)
            mock_rover_battery.return_value = BATTERY_CHARGE_100
            mock_drone_battery.return_value = LEVEL_TOO_LOW - 1
            await start(data, sid)
            mock_pub_start_rover.assert_called_once_with(
                'start_rover')
            mock_logger_queue.assert_called_with(
                'groundstation;;start;;')
            mock_pub_start_rover.reset_mock()
            mock_pub_start_drone.reset_mock()
            mock_logger_queue.reset_mock()

            # Test des 2 batteries trop basse (inférieur à 30%)
            mock_rover_battery.return_value = LEVEL_TOO_LOW - 1
            mock_drone_battery.return_value = LEVEL_TOO_LOW - 1
            await start(data, sid)
            mock_pub_start_rover.assert_not_called()
            mock_pub_start_drone.assert_not_called()
            mock_logger_queue.assert_not_called()

            # Test des 2 batteries chargées (supérieur à 30%)
            mock_rover_battery.return_value = BATTERY_CHARGE_100
            mock_drone_battery.return_value = BATTERY_CHARGE_100
            await start(data, sid)
            mock_pub_start_rover.assert_called_once_with(
                'start_rover')
            mock_pub_start_drone.assert_called_once_with(
                'start_drone')
            mock_logger_queue.assert_called_with(
                'groundstation;;start;;')

    async def test_finish(self):
        sid = '1234'
        data = 'test_data'
        with patch('sockets.pub_finish.put') as mock_pub_finish:
            with patch('sockets.logger_queue.put_nowait') as mock_logger_queue:
                await finish(data, sid)
                mock_pub_finish.assert_called_once_with('finish')
                mock_logger_queue.assert_called_once_with(
                    'groundstation;;finish;;')

    async def test_return_home(self):
        sid = '1234'
        data = 'test_data'
        with patch('sockets.return_home_finish.put') as mock_return_home_finish:
            await return_home(data, sid)
            mock_return_home_finish.assert_called_once_with('return_home')

    async def test_connect(self):
        sid = '1234'
        environ = None
        auth = None

        with patch('sockets.asyncio.create_task') as mock_create_task:
            with patch('sockets.logger_queue.put_nowait') as mock_logger_queue:
                await connect(sid, environ, auth)
                mock_logger_queue.assert_called_once_with(
                    'groundstation;;connect;;1234')

    async def test_disconnect(self):
        sid = '1234'
        with patch('sockets.logger_queue.put_nowait') as mock_logger_queue:
            await disconnect(sid)
            mock_logger_queue.assert_called_once_with(
                'groundstation;;disconnect;;1234')


class TestLoggerTask(unittest.IsolatedAsyncioTestCase):

    warnings.filterwarnings("ignore", category=RuntimeWarning)

    async def test_logger_task(self):
        with patch('sockets.database.SessionLocal') as mock_db_session:
            with patch('sockets.models.LogEntry') as mock_log_entry:
                with patch('sockets.models.Mission') as mock_mission:
                    with patch('sockets.mission', mock_mission):
                        db_instance = Mock()
                        mock_db_session.return_value = db_instance
                        db_instance.add.return_value = None
                        db_instance.commit.return_value = None
                        db_instance.refresh.return_value = None
                        db_instance.close.return_value = None

                        mock_log_entry.return_value = None

                        # Simule un message dans logger_queue
                        logger_queue.put_nowait('groundstation;;test;;data')

                        # On lance logger_task() et on l'annule au bout de 2 secondes
                        task = asyncio.create_task(logger_task())
                        await asyncio.sleep(2)
                        task.cancel()

                        # Vérification des appels
                        # mock_sio_emit.assert_called_once()
                        mock_db_session.assert_called_once()
                        mock_log_entry.assert_called_once_with(
                            mission_id=mock_mission.id,
                            time=ANY,
                            robot='groundstation',
                            category='test',
                            data='data')
                        db_instance.add.assert_called_once()
                        db_instance.commit.assert_called_once()
                        db_instance.refresh.assert_called_once()
                        db_instance.close.assert_called_once()


if __name__ == '__main__':
    unittest.main()
