import pytest
import socketio
from unittest import mock
from sockets import RobotCommunication
from unittest.mock import MagicMock, patch, AsyncMock
import time


@pytest.fixture
def rover():
    return RobotCommunication('rover')


@pytest.fixture
def drone():
    return RobotCommunication('drone')


def test_RobotCommunication_init(rover):
    assert rover.name == 'rover'
    assert rover.in_mission is None
    assert rover.last_updated is None
    assert rover.sub is not None


def test_robot_state(rover):
    sample = MagicMock()
    sample.payload.decode.return_value = 'True'

    rover.robot_state(sample)
    assert rover.in_mission == 'True'
    assert rover.last_updated is not None
    assert rover.last_updated == time.time()

    sample.payload.decode.return_value = 'False'
    rover.robot_state(sample)
    assert rover.in_mission == 'False'
    assert rover.last_updated is not None
    assert rover.last_updated == time.time()
