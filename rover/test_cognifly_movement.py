import unittest
from unittest.mock import MagicMock, patch
from cognifly import Cognifly
from cognifly_movement import MoveCognifly, MAX_VELOCITY, MAX_YAW
import os


class TestMoveCognifly(unittest.TestCase):

    def setUp(self):
        os.environ["COGNIFLY_HOSTNAME"] = "localhost"
        Cognifly.__init__ = MagicMock(return_value=None)
        Cognifly.arm = MagicMock()
        Cognifly.disarm = MagicMock()
        Cognifly.takeoff_nonblocking = MagicMock()
        Cognifly.land_nonblocking = MagicMock()
        Cognifly.set_position_nonblocking = MagicMock()
        Cognifly.get_telemetry = MagicMock(
            return_value=["", "BLOCKED_UAV_NOT_LEVEL"])
        Cognifly.get_position = MagicMock(return_value=[1.0, 2.0, 0.5])

        # On ignore les time.sleep pour que les tests se fassent plus rapidement
        self.sleep_patcher = patch("time.sleep", MagicMock())
        self.sleep_patcher.start()
        self.move_cognifly = MoveCognifly()

    def tearDown(self):
        self.sleep_patcher.stop()

    def test_start_mission(self):
        self.move_cognifly.start_mission()
        self.assertTrue(self.move_cognifly.started_mission)
        Cognifly.arm.assert_called_once()
        Cognifly.takeoff_nonblocking.assert_called_once()
        Cognifly.set_position_nonblocking.assert_called()
        Cognifly.land_nonblocking.assert_called_once()
        Cognifly.disarm.assert_called_once()

    def test_start_mission_already_started(self):
        self.move_cognifly.started_mission = True
        self.move_cognifly.start_mission()
        Cognifly.arm.assert_not_called()
        Cognifly.takeoff_nonblocking.assert_not_called()
        Cognifly.set_position_nonblocking.assert_not_called()
        Cognifly.land_nonblocking.assert_not_called()
        Cognifly.disarm.assert_not_called()

    def test_start_mission_battery_low(self):
        self.move_cognifly.battery = 20
        self.move_cognifly.start_mission()
        Cognifly.arm.assert_not_called()
        Cognifly.takeoff_nonblocking.assert_not_called()
        Cognifly.set_position_nonblocking.assert_not_called()
        Cognifly.land_nonblocking.assert_not_called()
        Cognifly.disarm.assert_not_called()

    def test_start_mission_finishing_mission(self):
        self.move_cognifly.finishing_mission = True
        self.move_cognifly.start_mission()
        Cognifly.arm.assert_not_called()
        Cognifly.takeoff_nonblocking.assert_not_called()
        Cognifly.set_position_nonblocking.assert_not_called()
        Cognifly.land_nonblocking.assert_not_called()
        Cognifly.disarm.assert_not_called()

    def test_identify_cognifly(self):
        # Créez un MagicMock pour la méthode declare_publisher
        declare_publisher_mock = MagicMock()
        put_mock = MagicMock()
        declare_publisher_mock.return_value.put = put_mock

        # On remplace zenoh.Session.declare_publisher par un mock
        with patch("zenoh.Session.declare_publisher", declare_publisher_mock):
            self.move_cognifly.identify_cognifly()

        # On vérifie si declare_publisher a été appelé
        self.assertTrue(declare_publisher_mock.called)
        put_mock.assert_called_once_with('identify')

    def test_is_crashed(self):
        self.assertTrue(self.move_cognifly.is_crashed())

    def test_distance_calculation(self):
        distance = self.move_cognifly.distance_calculation()
        self.assertAlmostEqual(distance, 2.236, places=3)

    def test_finish_mission(self):
        # Simuler le démarrage de la mission
        self.move_cognifly.started_mission = True

        self.move_cognifly.finish_mission()
        self.assertFalse(self.move_cognifly.started_mission)
        Cognifly.set_position_nonblocking.assert_called_once_with(x=0.0, y=0.0, z=1.0, yaw=0.0,
                                                                  max_velocity=MAX_VELOCITY,
                                                                  max_yaw_rate=MAX_YAW,
                                                                  max_duration=10.0, relative=False)
        Cognifly.land_nonblocking.assert_called_once()
        Cognifly.disarm.assert_called_once()

    def test_finish_mission_not_started(self):
        self.move_cognifly.started_mission = False
        self.move_cognifly.finish_mission()
        Cognifly.set_position_nonblocking.assert_not_called()
        Cognifly.land_nonblocking.assert_not_called()
        Cognifly.disarm.assert_not_called()

    def test_get_battery(self):
        Cognifly.get_telemetry = MagicMock(return_value=["", "7.4", ""])
        expected_battery = 50
        battery = self.move_cognifly.get_battery()
        self.assertEqual(battery, expected_battery)


if __name__ == '__main__':
    unittest.main()
