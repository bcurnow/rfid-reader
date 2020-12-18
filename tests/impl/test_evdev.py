import time

import evdev
import pytest
from unittest.mock import call, patch

from rfidreader.impl.evdev import EvdevReader, register


def _raw_event(value=1, code=11):
    class Event:
        def __init__(self, value, code):
            self.value = value
            self.code = code
            self.type = 0x01

        def timestamp(self):
            return time.time()

    return Event(value, code)


@pytest.mark.parametrize(
    ('event_ready_timeout', 'device_name'),
    [
        (None, None),
        (500, None),
        (None, '/dev/test'),
        (500, '/dev/test'),
    ],
    ids=['no config', 'timeout only', 'device_name only', 'timeout and device_name']
    )
@patch('rfidreader.impl.evdev.select')
@patch('rfidreader.impl.evdev.evdev')
def test_EvdevReader___init__(evdev, select, event_ready_timeout, device_name):
    config = {}
    if event_ready_timeout:
        config['event_ready_timeout'] = event_ready_timeout
    if device_name:
        config['device_name'] = device_name
    device = evdev.InputDevice.return_value
    poller = select.poll.return_value
    reader = EvdevReader(config)
    if not device_name:
        device_name = '/dev/input/event0'
    if not event_ready_timeout:
        event_ready_timeout = 100
    assert reader.device_name == device_name
    assert reader.event_ready_timeout == event_ready_timeout
    assert poller == reader.poller
    poller.register.assert_called_once_with(device, select.POLLIN)
    evdev.InputDevice.assert_called_once_with(device_name)


@pytest.mark.parametrize(
    ('delete_attr', 'close_side_effect'),
    [
        (None, None),
        ('device', None),
        (None, RuntimeError)
    ],
    ids=['device exists', 'device is missing', 'close throws error']
    )
@patch('rfidreader.impl.evdev.evdev')
def test_EvdevReader___del__(evdev, delete_attr, close_side_effect):
    config = {
        'event_ready_timeout': 500,
        'device_name': '/dev/test'
        }
    device = evdev.InputDevice.return_value
    device.fileno.return_value = 4
    reader = EvdevReader(config)
    if delete_attr:
        delattr(reader, delete_attr)
    if close_side_effect:
        device.close.side_effect = close_side_effect
    reader.__del__()
    evdev.InputDevice.assert_called_once_with('/dev/test')
    if not delete_attr and not close_side_effect:
        device.close.assert_called_once()


@pytest.mark.parametrize(
    ('poll_side_effect', 'raw_events', 'expected', 'timeout', 'timeout_call'),
    [
        ([[1], []], [_raw_event(), _raw_event(code=9)], '08', 10, 10 * 1000),
        ([[1], []], [_raw_event(code=34), _raw_event(code=35)], None, 10, 10 * 1000),
        ([[1], []], [_raw_event(), _raw_event(code=9)], '08', None, None),
        ([[1], []], [_raw_event(), _raw_event(code=9)], '08', -1, -1),
        ([[]], [], None, 10, 10 * 1000)
    ],
    ids=[
     'default',
     'returns None',
     'timeout None',
     'timeout negative value',
     'timed out'
     ]
    )
def test_EvdevReader_read(evdev_reader, poll_side_effect, raw_events, expected, timeout, timeout_call):
    poller = evdev_reader.poller
    poller.poll.side_effect = poll_side_effect
    generator = evdev_reader.device.read.return_value
    if raw_events:
        generator.__iter__.return_value = iter(raw_events)

    assert evdev_reader.read(timeout) == expected

    if poll_side_effect[0]:
        poller.poll.assert_has_calls([call(timeout_call), call(EvdevReader.EVENT_READY_TIMEOUT)], any_order=True)
        evdev_reader.device.read.assert_called_once()
        generator.__iter__.assert_called_once()
    else:
        poller.poll.assert_called_once_with(timeout_call)
        evdev_reader.device.read.assert_not_called()
        generator.__iter__.assert_not_called()


def test_EvdevReader__read_all_available_events_BlockingIOError(evdev_reader):
    evdev_reader.device.read.side_effect = BlockingIOError
    data = []
    evdev_reader._read_all_available_events(data)
    assert data == []


def test_EvdevReader__read_all_available_events_translate_returns_none(evdev_reader):
    generator = evdev_reader.device.read.return_value
    # Code 34 will translate to KEY_G which we don't have a mapping for
    raw_events = [_raw_event(code=34)]
    generator.__iter__.return_value = iter(raw_events)

    data = []
    evdev_reader._read_all_available_events(data)
    assert data == []

    evdev_reader.device.read.assert_called_once()
    generator.__iter__.assert_called_once()


@pytest.mark.parametrize(
    ('expected', 'event'),
    [
        (None, 'this is not an event type'),
        (None, evdev.events.KeyEvent(_raw_event(value=0))),
    ],
    ids=['Translate a non-event', 'Translate an event there is no mapping for']
    )
def test_EvdevReader__translate_event_wrong_event_type(evdev_reader, expected, event):
    assert evdev_reader._translate_event(event) is expected


@patch('rfidreader.impl.evdev.select')
@patch('rfidreader.impl.evdev.evdev')
def test_register(evdev, select):
    reader = register({})
    assert isinstance(reader, EvdevReader)
