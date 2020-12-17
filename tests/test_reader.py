import pytest
from unittest.mock import patch

from rfidreader.reader import RFIDReader, RFIDReaderTypeException


class MockReaderImpl:
    def __init__(self, device_name, ctx):
        self.device_name = device_name
        self.ctx = ctx

    def read(self, timeout):
        self.read_called = True
        self.read_timeout = timeout
        return 'Event'


DEVICE_NAME = 'test'
CTX = {}
MOCK_READER = MockReaderImpl(DEVICE_NAME, CTX)
READERS = {'mock': MOCK_READER}


@patch('rfidreader.reader.register_readers')
def test_RFIDReader___init__(register_readers):
    register_readers.return_value = READERS
    reader = RFIDReader('mock', DEVICE_NAME, CTX)
    assert reader.readers == READERS
    assert reader.reader == MOCK_READER
    register_readers.assert_called_once_with(DEVICE_NAME, CTX)


@patch('rfidreader.reader.register_readers')
def test_RFIDReader___init___missing_type(register_readers):
    register_readers.return_value = READERS
    with pytest.raises(RFIDReaderTypeException):
        RFIDReader('notmock', DEVICE_NAME, CTX)
    register_readers.assert_called_once_with(DEVICE_NAME, CTX)


@patch('rfidreader.reader.register_readers')
def test_RFIDReader_read(register_readers):
    register_readers.return_value = READERS
    reader = RFIDReader('mock', DEVICE_NAME, CTX)
    event = reader.read(25)
    assert event == 'Event'
    assert MOCK_READER.read_called is True
    assert MOCK_READER.read_timeout == 25
