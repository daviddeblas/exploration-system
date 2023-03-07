import unittest
from unittest.mock import patch
import main_robot1

class TestMainRobot1(unittest.TestCase):
    def setUp(self):
        self.sample = type('Sample', (object,), {'payload': b'start exploration'})

    # Test de start_listener et que le booléen soit bien modifié
    def test_start_listener(self):
        with patch('builtins.print') as mocked_print:
            main_robot1.start_listener(self.sample)
            mocked_print.assert_called_with('start exploration')
            self.assertTrue(main_robot1.start_exploration)

    # Test de finish_listener et que le booléen soit bien modifié
    def test_finish_listener(self):
        with patch('builtins.print') as mocked_print:
            with patch.object(main_robot1.launch_exploration, 'shutdown') as mocked_shutdown:
                main_robot1.finish_listener(self.sample)
                mocked_print.assert_called_with('start exploration')
                mocked_shutdown.assert_called_once()
                self.assertFalse(main_robot1.exploration_running)
    
    # Test que la fonction main() change bien la valeur des booléens lorsqu'ils sont prêts à démarrer l'exploration
    @patch('main_robot1.zenoh.open')
    def test_main(self, mocked_open):
        main_robot1.start_exploration = True
        main_robot1.exploration_running = False

        with patch('main_robot1.time.sleep') as mocked_sleep:
            def side_effect(*args, **kwargs):
                if mocked_sleep.call_count >= 2:
                    raise StopIteration
                return None
            mocked_sleep.side_effect = side_effect

            with patch.object(main_robot1.launch_exploration, 'start') as mocked_start:
                with self.assertRaises(StopIteration):
                    main_robot1.main()
                mocked_open.assert_called_once()
                mocked_sleep.assert_called_with(1)
                self.assertFalse(main_robot1.start_exploration)
                self.assertTrue(main_robot1.exploration_running)
                mocked_start.assert_called_once()

if __name__ == '__main__':
    unittest.main()
