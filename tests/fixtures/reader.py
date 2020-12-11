import pytest
from unittest.mock import patch

import evdev

from rfidreader.reader import RFIDReader


@pytest.fixture()
def reader():
    with patch('rfidreader.reader.evdev') as mock_evdev, patch('rfidreader.reader.select'):
        # Make sure to that KeyEvent is a type so the isinstance check works
        mock_evdev.events.KeyEvent = evdev.events.KeyEvent
        # Make sure that the categorize method is not mock so we don't have to mock out the calls
        mock_evdev.categorize = evdev.categorize
        # Make sure that the device return implements the fileno method
        device = mock_evdev.InputDevice.return_value
        device.fileno.return_value = 4
        yield RFIDReader('/dev/input/rfid')
