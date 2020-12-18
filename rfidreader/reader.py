from rfidreader.exception import RFIDReaderTypeException
from rfidreader.impl import load_impl


class RFIDReader:
    """
    Wraps up the basic reading function and registers all the implementations.
    config:      A dictionary of configuration options which can be implementation dependent.
                 Will be passed passed un-modified to the implementation.
    """
    def __init__(self, reader_type, config):
        self.config = config
        self.reader =load_impl(reader_type, config)
        if not self.reader:
            raise RFIDReaderTypeException(f'Could not find a registered reader for "{reader_type}"')

    def read(self, timeout=None):
        """
        Reads the next RFID tag presented. Will only wait timeout seconds.
        Returns None if timeout expires.
        If timeout is not provided, negative or None, will wait forever.
        """
        return self.reader.read(timeout)
