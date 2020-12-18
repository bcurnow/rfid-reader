import pytest
from unittest.mock import patch

from rfidreader.reader import RFIDReader, RFIDReaderTypeException


class MockReaderImpl:
    def __init__(self, ctx):
        self.ctx = ctx

    def read(self, timeout):
        self.read_called = True
        self.read_timeout = timeout
        return 'Event'


CONFIG = {}
MOCK_READER = MockReaderImpl
READERS = {'mock': MOCK_READER}


@patch('rfidreader.reader.register_readers')
def test_RFIDReader___init__(register_readers):
    register_readers.return_value = READERS
    reader = RFIDReader('mock', CONFIG)
    assert reader.config == CONFIG
    assert reader.readers == READERS
    assert isinstance(reader.reader, MockReaderImpl)
    register_readers.assert_called_once_with()


@patch('rfidreader.reader.register_readers')
def test_RFIDReader___init___missing_type(register_readers):
    register_readers.return_value = READERS
    with pytest.raises(RFIDReaderTypeException):
        RFIDReader('notmock', CONFIG)
    register_readers.assert_called_once_with()


@patch('rfidreader.reader.register_readers')
def test_RFIDReader_read(register_readers):
    register_readers.return_value = READERS
    reader = RFIDReader('mock', CONFIG)
    event = reader.read(25)
    assert event == 'Event'
    assert reader.reader.read_called is True
    assert reader.reader.read_timeout == 25
