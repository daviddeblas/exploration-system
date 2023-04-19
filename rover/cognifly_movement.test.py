import unittest
from unittest.mock import patch
from cognifly_movement import MoveCognifly

class TestCogniflyMouvement(unittest.TestCase):
    def setUp(self):
        self.cognifly_movement = MoveCognifly()
        

    # Test de start_mission
    def test_start_mission(self):
        with patch.object(self.cognifly_movement.cf, 'arm') as mock_arm, \
             patch.object(self.cognifly_movement.cf, 'takeoff_nonblocking') as mock_takeoff, \
             patch.object(self.cognifly_movement.cf, 'land_nonblocking') as mock_land, \
             patch.object(self.cognifly_movement.cf, 'disarm') as mock_disarm:   
            
            self.cognifly_movement.start_mission()
            
            self.assertTrue(mock_arm.called)
            self.assertTrue(mock_takeoff.called)
            self.assertTrue(mock_land.called)
            self.assertTrue(mock_disarm.called)
            self.assertTrue(self.cognifly_movement.started_mission)
            self.assertFalse(self.cognifly_movement.finishing_mission)
            

    # Test de identify_cognifly 
    # def identify_cognifly(self):

    # Test de is_crashed 
    # def is_crashed(self):    

    
    # Test de distance_calculation 
    def test_distance_calculation(self):
        distance = self.cognifly_movement.distance_calculation()
        self.assertIsInstance(distance, float)     

    # Test de finish_listener
    def test_finish_mission(self):
        with patch.object(self.cognifly_movement.cf, 'set_position_nonblocking') as mock_set_position, \
             patch.object(self.cognifly_movement.cf, 'land_nonblocking') as mock_land, \
             patch.object(self.cognifly_movement.cf, 'disarm') as mock_disarm:   
            
            self.cognifly_movement.finish_mission()
            
            self.assertTrue(mock_set_position.called)
            self.assertTrue(mock_land.called)
            self.assertTrue(mock_disarm.called)
            self.assertFalse(self.cognifly_movement.finishing_mission)
            self.assertFalse(self.cognifly_movement.started_mission)
        
if __name__ == '__main__':
    unittest.main()