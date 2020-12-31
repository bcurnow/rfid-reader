from mfrc522reader import MFRC522

def register(config):
    return MFRC522Reader(config)


class MFRC522Reader:
    def __init__(self, config):
        self.reader = MFRC522(**config)

    def read(self, timeout=None):
        """
        Reads the next RFID tag presented. Will only wait timeout seconds.
        Returns None if timeout expires.
        If timeout is not provided, negative or None, will wait forever.
        """
        return self.reader.read_uid(timeout)
