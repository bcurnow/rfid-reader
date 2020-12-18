import atexit
import time

import RPi.GPIO as GPIO
from mfrc522 import MFRC522


def register():
    return ('mfrc522', MFRC522Reader)


class MFRC522Reader:
    def __init__(self, config):
        self.reader = MFRC522(**config)
        atexit.register(GPIO.cleanup)

    def read(self, timeout=None):
        """
        Reads the next RFID tag presented. Will only wait timeout seconds.
        Returns None if timeout expires.
        If timeout is not provided, negative or None, will wait forever.
        """
        # Default to no timeout
        stop_time = None
        if timeout and timeout >= 0:
            stop_time = time.time() + timeout

        id = self._read_id_no_block()
        print(id)
        while not id and stop_time != time.time():
            id = self._read_id_no_block()
            print(id)
        return id

    def _read_id_no_block(self):
        (status, TagType) = self.reader.MFRC522_Request(self.reader.PICC_REQIDL)
        print(status, self.reader.MI_OK)
        if status != self.reader.MI_OK:
            return None
        (status, uid) = self.reader.MFRC522_Anticoll()
        print(status, uid)
        if status != self.reader.MI_OK:
            return None
        return self._uid_to_num(uid)

    def _uid_to_num(self, uid):
        n = 0
        for i in range(0, 5):
            n = n * 256 + uid[i]
        return n