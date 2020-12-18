import atexit
import time


def register(config):
    return MFRC522Reader(config)


class MFRC522Reader:
    def __init__(self, config):
        # Import inside __init__ in case our dependencies aren't installed
        import RPi.GPIO as GPIO
        from mfrc522 import MFRC522
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
        while not id and stop_time != time.time():
            id = self._read_id_no_block()
        return id

    def _read_id_no_block(self):
        (status, TagType) = self.reader.MFRC522_Request(self.reader.PICC_REQIDL)
        if status != self.reader.MI_OK:
            return None
        (status, uid) = self.reader.MFRC522_Anticoll()
        if status != self.reader.MI_OK:
            return None
        return self._uid_to_num(uid)

    def _uid_to_num(self, uid):
        n = 0
        for i in range(0, 5):
            n = n * 256 + uid[i]
        return n
