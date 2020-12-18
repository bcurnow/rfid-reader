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


@patch('rfidreader.reader.load_impl')
def test_RFIDReader___init__(load_impl):
    load_impl.return_value = MockReaderImpl({})
    reader = RFIDReader('mock', {})
    assert reader.config == {}
    assert isinstance(reader.reader, MockReaderImpl)
    load_impl.assert_called_once_with('mock', {})


@patch('rfidreader.reader.load_impl')
def test_RFIDReader___init___missing_type(load_impl):
    load_impl.return_value = None
    with pytest.raises(RFIDReaderTypeException):
        RFIDReader('notmock', {})
    load_impl.assert_called_once_with('notmock', {})


@patch('rfidreader.reader.load_impl')
def test_RFIDReader_read(load_impl):
    load_impl.return_value = MockReaderImpl({})
    reader = RFIDReader('mock', {})
    event = reader.read(25)
    assert event == 'Event'
    assert reader.reader.read_called is True
    assert reader.reader.read_timeout == 25
