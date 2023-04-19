import unittest
from unittest.mock import patch
import main

class TestMain(unittest.TestCase):
    def setUp(self):
        self.sample = type('Sample', (object,), {'payload': b'start exploration'})

    # Test de start_listener et que le booléen soit bien modifié
    def test_start_listener(self):
        with patch('builtins.print') as mocked_print:
            with patch.object(main.cognifly, 'start_mission') as mocked_cognifly_start:
                main.start_listener(self.sample)
                mocked_print.assert_called_with('start exploration')
                mocked_cognifly_start.assert_called_once()
                self.assertTrue(main.start_exploration)

    # Test de identify_rover 
    # def test_identify_rover(self):
            # main.identify_rover()
    
    # Test de identify_listener for rover and cognifly
    def test_identify_listener(self):
        with patch('identify_rover') as mocked_id_rover:
            with patch.object(main.cognifly, 'identify_cognifly') as mocked_id_cognifly:
                main.identify_listener(self.sample)
                mocked_id_rover.assert_called_once()
                mocked_id_cognifly.assert_called_once()
        

    # Test de finish_listener et que le booléen soit bien modifié
    def test_finish_listener(self):
        with patch('builtins.print') as mocked_print:
            with patch.object(main.launch_exploration, 'shutdown') as mocked_shutdown:
                main.finish_listener(self.sample)
                mocked_print.assert_called_with('start exploration')
                mocked_shutdown.assert_called_once()
                self.assertFalse(main.exploration_running)

    # Test de farthest_robot_trigger is cognifly
    def test_farthest_robot_trigger_drone():
        with patch.object(main.cognifly, 'identify_cognifly') as mocked_id_cognifly:
            with patch.object(main.cognifly, 'distance_calculation', 10 ) as mocked_cognifly_distance:
                main.last_position = {x: 0, y: 1}
                main.farthest_robot_trigger()
                mocked_cognifly_distance.assert_called_once()
                mocked_id_cognifly.assert_called_once()

    # Test de farthest_robot_trigger is rover
    def test_farthest_robot_trigger_rover():
        with patch('identify_rover') as mocked_id_rover:
            with patch.object(main.cognifly, 'distance_calculation', 0 ) as mocked_cognifly_distance:
                main.last_position = {x: 5, y: 5}
                main.farthest_robot_trigger()
                mocked_cognifly_distance.assert_called_once()
                mocked_id_rover.assert_called_once()

    # Test de p2p_trigger
    def test_p2p_trigger(self):
        main.p2p_trigger(self.sample)
        self.assertTrue(main.activate_p2p)

    # Test de odom_callback
    def test_odom_callback(self, data):
        with patch('publish_cognifly_odom') as mocked_odom:
            main.odom_callback(data)
            mocked_odom.assert_called_once()
            self.assertTrue(main.last_position)

    # Test de scan_callback
    def test_scan_callback(self,data):
        main.scan_callback(data)
        self.assertTrue(main.last_scan)
    # Test de return_home_listener
    def test_return_home_listener(self):
        with patch.object(main.cognifly, 'finish_mission') as mocked_cognifly_finish:
            main.return_home_listener(self.sample)
            mocked_cognifly_finish.assert_called_once()
            self.assertTrue(main.initial_data)
            self.assertTrue(main.exploration_running)

       
    # Test que la fonction main() change bien la valeur des booléens lorsqu'ils sont prêts à démarrer l'exploration
    @patch('main.zenoh.open')
    def test_main(self, mocked_open):
        main.start_exploration = True
        main.exploration_running = False

if __name__ == '__main__':
    unittest.main()