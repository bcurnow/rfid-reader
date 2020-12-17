from types import SimpleNamespace

import pytest
from unittest.mock import patch

import evdev

from rfidreader.impl.evdev import EvdevReader


@pytest.fixture()
def evdev_reader():
    with patch('rfidreader.impl.evdev.evdev') as mock_evdev, patch('rfidreader.impl.evdev.select'):
        # Make sure to that KeyEvent is a type so the isinstance check works
        mock_evdev.events.KeyEvent = evdev.events.KeyEvent
        # Make sure that the categorize method is not mock so we don't have to mock out the calls
        mock_evdev.categorize = evdev.categorize
        # Make sure that the device return implements the fileno method
        device = mock_evdev.InputDevice.return_value
        device.fileno.return_value = 4
        yield EvdevReader('/dev/input/rfid', SimpleNamespace())
