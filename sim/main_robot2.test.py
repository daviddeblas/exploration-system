import unittest
from unittest.mock import MagicMock, patch, call
import main_robot2 as mr2
from nav_msgs.msg import Odometry, OccupancyGrid
import warnings


class TestMainRobot2(unittest.TestCase):
    @patch("main_robot2.rospy.sleep")
    def test_rotation_left(self, mock_sleep):
        with patch("main_robot2.pub") as mock_pub:
            mr2.rotation_left()
            mock_pub.publish.assert_called()

    @patch("main_robot2.rospy.sleep")
    def test_rotation_right(self, mock_sleep):
        with patch("main_robot2.pub") as mock_pub:
            mr2.rotation_right()
            mock_pub.publish.assert_called()

    @patch("main_robot2.rospy.sleep")
    def test_forward(self, mock_sleep):
        with patch("main_robot2.pub") as mock_pub:
            mr2.forward()
            mock_pub.publish.assert_called()

    @patch("main_robot2.rospy.sleep")
    def test_backward(self, mock_sleep):
        warnings.filterwarnings("ignore")
        with patch("main_robot2.pub") as mock_pub:
            mr2.backward()
            mock_pub.publish.assert_called()

    @patch("main_robot2.rospy.sleep")
    def test_lateral_movement(self, mock_sleep):
        with patch("main_robot2.pub") as mock_pub:
            mr2.lateral_movement("up")
            mock_pub.publish.assert_called()

            mr2.lateral_movement("down")
            mock_pub.publish.assert_called()

    @patch("main_robot2.rospy.sleep")
    def test_back_to_base(self, mock_sleep):
        with patch("main_robot2.pub") as mock_pub, patch("main_robot2.lateral_movement") as mock_lateral_movement:
            mr2.line_counter = 1
            mr2.forward_counter = 2
            mr2.backward_counter = 1
            mr2.back_to_base()
            mock_lateral_movement.assert_called()
            mock_pub.publish.assert_called()

    @patch("main_robot2.rospy.sleep")
    def test_drone_movement_forward(self, mock_sleep):
        with patch("main_robot2.forward") as mock_forward, patch("main_robot2.back_to_base") as mock_back_to_base, patch("main_robot2.stop_event") as mock_stop_event, patch("main_robot2.lateral_movement") as mock_lateral_movement:
            mr2.line_counter = 0
            # Configurez stop_event.is_set pour retourner False
            mock_stop_event.is_set.return_value = False

            mr2.drone_movement(2, 0)
            mock_forward.assert_called()
            mock_lateral_movement.assert_called_with("up")
            mock_back_to_base.assert_called()

    @patch("main_robot2.rospy.sleep")
    def test_drone_movement_backward(self, mock_sleep):
        with patch("main_robot2.backward") as mock_backward, patch("main_robot2.back_to_base") as mock_back_to_base, patch("main_robot2.stop_event") as mock_stop_event, patch("main_robot2.lateral_movement") as mock_lateral_movement:
            mr2.line_counter = 1
            # Configurez stop_event.is_set pour retourner False
            mock_stop_event.is_set.return_value = False

            mr2.drone_movement(2, 0)
            mock_backward.assert_called()
            mock_lateral_movement.assert_called_with("up")
            mock_back_to_base.assert_called()

    @patch("main_robot2.rospy.sleep")
    def test_drone_movement_stop_event(self, mock_sleep):
        with patch("main_robot2.lateral_movement") as mock_lateral_movement, patch("main_robot2.back_to_base") as mock_back_to_base, patch("main_robot2.stop_event") as mock_stop_event:
            mr2.line_counter = 0

            # Configurez stop_event.is_set pour retourner True
            mock_stop_event.is_set.return_value = True

            mr2.drone_movement(2, 0)
            mock_lateral_movement.assert_not_called()
            mock_back_to_base.assert_not_called()

    def test_stop_drone_movement(self):
        with patch("main_robot2.pub") as mock_pub:
            mr2.stop_drone_movement()
            mock_pub.publish.assert_called()

    # On teste odom_callback en vérifiant si les variables globales sont mises à jour correctement.
    def test_odom_callback(self):
        mock_data = MagicMock()
        mock_data.pose.pose.position = "mock_position"
        mr2.odom_callback(mock_data)
        self.assertEqual(mr2.last_position, "mock_position")

    def test_start_listener(self):
        sample = MagicMock()
        sample.payload.decode.return_value = "start_drone"

        with patch("main_robot2.threading.Thread") as mock_thread:
            mr2.start_listener(sample)
            self.assertTrue(mr2.exploration_running)
            mock_thread.assert_called()

    def test_identify_listener(self):
        sample = MagicMock()
        sample.payload.decode.return_value = "identify_message"

        with patch("main_robot2.print") as mock_print:
            mr2.identify_listener(sample)
            mock_print.assert_called_with("identify_message")

    def test_finish_listener(self):
        sample = MagicMock()
        sample.payload.decode.return_value = "finish_message"

        with patch("main_robot2.stop_drone_movement") as mock_stop_drone_movement, patch.object(mr2, 'drone_thread', MagicMock()) as mock_drone_thread:
            mr2.finish_listener(sample)
            self.assertFalse(mr2.exploration_running)
            mock_stop_drone_movement.assert_called()
            mock_drone_thread.join.assert_called()

    def test_return_home_listener(self):
        sample = MagicMock()
        sample.payload.decode.return_value = "return_home_message"

        with patch("main_robot2.back_to_base") as mock_back_to_base, patch.object(mr2, 'drone_thread', MagicMock()) as mock_drone_thread:
            mr2.exploration_running = True
            mr2.return_home_listener(sample)
            self.assertFalse(mr2.exploration_running)
            mock_drone_thread.join.assert_called()
            mock_back_to_base.assert_called()

            mr2.exploration_running = False
            mr2.return_home_listener(sample)
            mock_back_to_base.assert_called()

    def mock_sleep(time):
        pass

    @patch("main_robot2.rospy.sleep", mock_sleep)
    @patch("main_robot2.time.sleep", mock_sleep)
    @patch("main_robot2.session.declare_subscriber")
    @patch("main_robot2.session.declare_publisher")
    @patch("main_robot2.rospy.Subscriber")
    def test_main(self, mock_subscriber, mock_declare_publisher, mock_declare_subscriber):
        warnings.filterwarnings("ignore")
        # Limiter le nombre d'itérations dans la boucle while
        iteration_limit = 3
        iteration_counter = [0]

        # Remplacer la fonction 'time.sleep' par une fonction qui compte les itérations
        def count_iterations(time):
            iteration_counter[0] += 1
            if iteration_counter[0] >= iteration_limit:
                raise KeyboardInterrupt

        with patch("main_robot2.time.sleep", side_effect=count_iterations):
            with self.assertRaises(KeyboardInterrupt):
                mr2.main()

            # Vérifiez que les fonctions sont appelées avec les bons arguments
            mock_declare_subscriber.assert_has_calls([
                call('start_drone', mr2.start_listener),
                call('identify', mr2.identify_listener),
                call('finish', mr2.finish_listener),
                call('return_home', mr2.return_home_listener)
            ])
            mock_declare_publisher.assert_has_calls([
                call('drone_state'),
                call('logger')
            ])
            mock_subscriber.assert_has_calls([
                call("/robot2/odom", Odometry, mr2.odom_callback),
                call('/robot2/map', OccupancyGrid, mr2.map_callback)
            ])

    def test_receive_existence(self):
        sample = MagicMock()
        with patch("main_robot2.send_existence") as mock_send_existence:
            mr2.limo_exists = False
            mr2.receive_existence(sample)
            self.assertTrue(mr2.limo_exists)
            mock_send_existence.assert_called()

            mr2.limo_exists = True
            mr2.receive_existence(sample)
            mock_send_existence.assert_called_once()

    def test_send_existence(self):
        with patch("main_robot2.session") as mock_session:
            mock_publisher = MagicMock()
            mock_session.declare_publisher.return_value = mock_publisher
            mr2.send_existence()
            mock_publisher.put.assert_called_with(True)


if __name__ == "__main__":
    unittest.main()
