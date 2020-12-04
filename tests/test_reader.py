import select
import time

import pytest
from unittest.mock import call, patch

import evdev

from rfidreader.reader import RFIDReader, EVENT_READY_TIMEOUT

timeout = 10

@patch('rfidreader.reader.select')
@patch('rfidreader.reader.evdev')
def test___init__(evdev, select):
    device = evdev.InputDevice.return_value
    poller = select.poll.return_value
    reader = RFIDReader('/dev/test', 500)
    assert device == reader.device
    assert reader.event_ready_timeout == 500
    assert poller == reader.poller
    poller.register.assert_called_once_with(device, select.POLLIN)
    evdev.InputDevice.assert_called_once_with('/dev/test')


@patch('rfidreader.reader.evdev')
def test___del__(evdev):
    device = evdev.InputDevice.return_value
    device.fileno.return_value = 4
    reader = RFIDReader('/dev/test', 500)
    reader.__del__()
    evdev.InputDevice.assert_called_once_with('/dev/test')
    device.close.assert_called_once()


@patch('rfidreader.reader.evdev')
def test___del___invalid_device(evdev):
    device = evdev.InputDevice.return_value
    device.fileno.return_value = 4
    reader = RFIDReader('/dev/test', 500)
    # Remove the device attribute to simulate the state that happens when an exception is thrown in __init__
    delattr(reader, 'device')
    reader.__del__()
    evdev.InputDevice.assert_called_once_with('/dev/test')
    device.close.assert_not_called()


@patch('rfidreader.reader.evdev')
def test___del___RuntimeError(evdev):
    device = evdev.InputDevice.return_value
    device.fileno.return_value = 4
    reader = RFIDReader('/dev/test', 500)
    device.close.side_effect = RuntimeError
    reader.__del__()
    evdev.InputDevice.assert_called_once_with('/dev/test')
    device.close.assert_called_once()


def test_read(reader):
    poller = reader.poller
    poller.poll.side_effect = [[1], []]
    generator = reader.device.read.return_value
    raw_events = [_raw_event(), _raw_event(code=9)]
    generator.__iter__.return_value = iter(raw_events)

    assert reader.read(timeout) == '08'

    poller.poll.assert_has_calls([call(timeout * 1000), call(EVENT_READY_TIMEOUT)], any_order=True)
    reader.device.read.assert_called_once()
    generator.__iter__.assert_called_once()


def test_read_returns_none(reader):
    poller = reader.poller
    poller.poll.side_effect = [[1], []]
    generator = reader.device.read.return_value
    raw_events = [_raw_event(code=34), _raw_event(code=35)]
    generator.__iter__.return_value = iter(raw_events)

    assert not reader.read(timeout)

    poller.poll.assert_has_calls([call(timeout * 1000), call(EVENT_READY_TIMEOUT)], any_order=True)
    reader.device.read.assert_called_once()
    generator.__iter__.assert_called_once()


def test_read_timeout_none(reader):
    poller = reader.poller
    poller.poll.side_effect = [[1], []]
    generator = reader.device.read.return_value
    raw_events = [_raw_event(), _raw_event(code=9)]
    generator.__iter__.return_value = iter(raw_events)

    assert reader.read(None) == '08'

    poller.poll.assert_has_calls([call(None), call(EVENT_READY_TIMEOUT)], any_order=True)
    reader.device.read.assert_called_once()
    generator.__iter__.assert_called_once()


def test_read_timeout_negative(reader):
    poller = reader.poller
    poller.poll.side_effect = [[1], []]
    generator = reader.device.read.return_value
    raw_events = [_raw_event(), _raw_event(code=9)]
    generator.__iter__.return_value = iter(raw_events)

    assert reader.read(-1) == '08'

    poller.poll.assert_has_calls([call(-1), call(EVENT_READY_TIMEOUT)], any_order=True)
    reader.device.read.assert_called_once()
    generator.__iter__.assert_called_once()


def test_read_timeout(reader):
    poller = reader.poller
    poller.poll.return_value = []

    assert reader.read(timeout) is None

    poller.poll.assert_called_once_with(timeout * 1000)


def test__read_all_available_events_BlockingIOError(reader):
    reader.device.read.side_effect = BlockingIOError
    data = []
    reader._read_all_available_events(data)
    assert data == []


def test__read_all_available_events_translate_returns_none(reader):
    generator = reader.device.read.return_value
    # Code 34 will translate to KEY_G which we don't have a mapping for
    raw_events = [_raw_event(code=34)]
    generator.__iter__.return_value = iter(raw_events)

    data = []
    reader._read_all_available_events(data)
    assert data == []

    reader.device.read.assert_called_once()
    generator.__iter__.assert_called_once()


def test__translate_event_wrong_event_type(reader):
    assert not reader._translate_event('this is not an event type')


def test__translate_event_wrong_keystate(reader):
    assert not reader._translate_event(evdev.events.KeyEvent(_raw_event(value=0)))


def _raw_event(value=1, code=11):
    class Event:
        def __init__(self, value, code):
            self.value = value
            self.code = code
            self.type = 0x01

        def timestamp(self):
            return time.time()

    return Event(value, code)
