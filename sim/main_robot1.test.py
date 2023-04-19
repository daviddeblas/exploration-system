import unittest
from unittest.mock import patch, MagicMock, call
import main_robot1
import rospy
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalID


class TestMainRobot1(unittest.TestCase):
    def setUp(self):
        self.sample = type('Sample', (object,), {
                           'payload': b'test'})

    def test_start_listener(self):
        with patch('builtins.print') as mocked_print:
            main_robot1.start_listener(self.sample)
            mocked_print.assert_called_with('test')
            self.assertTrue(main_robot1.start_exploration)

    def test_finish_listener(self):
        with patch('builtins.print') as mocked_print:
            with patch.object(main_robot1.launch_exploration, 'shutdown') as mocked_shutdown:
                main_robot1.finish_listener(self.sample)
                mocked_print.assert_called_with('test')
                mocked_shutdown.assert_called_once()
                self.assertFalse(main_robot1.exploration_running)

    def test_identify_listener(self):
        with patch('builtins.print') as mocked_print:
            main_robot1.identify_listener(self.sample)
            mocked_print.assert_called_with('test')

    def test_odom_callback(self):
        data = MagicMock()
        main_robot1.odom_callback(data)
        self.assertEqual(main_robot1.last_position, data.pose.pose.position)

    def test_scan_callback(self):
        data = MagicMock()
        main_robot1.scan_callback(data)
        self.assertEqual(main_robot1.last_scan, data)

    def test_map_callback(self):
        data = MagicMock()
        with patch("main_robot1.map_generation_limo") as mocked_map_gen:
            png_mock = MagicMock()
            png_mock.tobytes.return_value = b"mocked_png"
            mocked_map_gen.return_value = png_mock
            with patch.object(main_robot1.session, "declare_publisher") as mocked_declare_publisher:
                main_robot1.map_callback(data)
                mocked_map_gen.assert_called_once_with(
                    data, "map", main_robot1.tfBuffer, main_robot1.cognifly_exists, True
                )
                mocked_declare_publisher.assert_called_once_with("map_image")
                main_robot1.session.declare_publisher().put.assert_called_once_with(b"mocked_png")

    def test_return_home_listener(self):
        with patch('builtins.print') as mocked_print:
            with patch.object(main_robot1, 'PoseStamped', return_value=PoseStamped()) as mocked_pose_stamped:
                with patch.object(rospy, 'Publisher', autospec=True) as mocked_publisher:
                    main_robot1.return_home_listener(self.sample)
                    mocked_print.assert_called_with('test')

                    # Vérifier si publish() a été appelé exactement deux fois
                    assert mocked_publisher.return_value.publish.call_count == 2

                    # Vérification du premier appel
                    args1, kwargs1 = mocked_publisher.return_value.publish.call_args_list[0]
                    cancel_msg1 = args1[0]
                    self.assertIsInstance(cancel_msg1, GoalID)

                    # Vérification du deuxième appel
                    args2, kwargs2 = mocked_publisher.return_value.publish.call_args_list[1]
                    sent_msg2 = args2[0]
                    self.assertIsInstance(sent_msg2, PoseStamped)

    def test_receive_existence(self):
        main_robot1.cognifly_exists = False
        main_robot1.receive_existence(self.sample)
        self.assertTrue(main_robot1.cognifly_exists)

    def test_send_existence(self):
        with patch.object(main_robot1.session, "declare_publisher") as mocked_declare_publisher:
            main_robot1.send_existence()
            mocked_declare_publisher.assert_called_once_with("limo_exists")
            main_robot1.session.declare_publisher().put.assert_called_once_with(True)


if __name__ == '__main__':
    unittest.main()
