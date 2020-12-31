import RPi.GPIO as GPIO
import pytest
from unittest.mock import patch

from rfidreader.impl.mfrc522 import MFRC522Reader, register


@pytest.mark.parametrize(
    ('config',),
    [
        ({},),
        ({
            'bus': 0,
            'device': 0,
            'gpio_mode': GPIO.BOARD,
            'pin_rst': None,
        },),
    ],
    ids=['no config', 'all config']
    )
@patch('rfidreader.impl.mfrc522.MFRC522')
def test_MFRC522Reader___init__(MFRC522, config):
    internal_reader = MFRC522.return_value
    reader = MFRC522Reader(config)
    assert reader.reader == internal_reader
    MFRC522.assert_called_once_with(config)


@pytest.mark.parametrize(
    ('expected', 'timeout'),
    [
        (None, 10),
        (0x80607A22532304, 10),
        (None, -1),
        (None, None)
    ],
    )
@patch('rfidreader.impl.mfrc522.MFRC522')
def test_MFRC522Reader_read(MFRC522, expected, timeout):
    internal_reader = MFRC522.return_value
    internal_reader.read_uid.return_value = expected
    reader = MFRC522Reader({})
    actual = reader.read(timeout)
    assert actual == expected
    internal_reader.read_uid.assert_called_once_with(timeout)


@patch('rfidreader.impl.mfrc522.MFRC522')
def test_register(MFRC522):
    reader = register({})
    assert isinstance(reader, MFRC522Reader)
