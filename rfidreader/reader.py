from rfidreader.exception import RFIDReaderTypeException
from rfidreader.impl import register_readers


class RFIDReader:
    """
    Wraps up the basic reading function and registers all the implementations.
    device_name: The name of the device the reader is connected to (e.g. /dev/input/event0)
    ctx: A SimpleNamespace of configuration options which can be implementation dependent.
         Will be passed to the implementation's register method as-is
    """
    def __init__(self, reader_type, device_name, ctx):
        self.readers = register_readers(device_name, ctx)
        if reader_type in self.readers:
            self.reader = self.readers[reader_type]
        else:
            raise RFIDReaderTypeException(f'Could not find a registered reader for "{reader_type}"')

    def read(self, timeout=None):
        """
        Reads the next RFID tag presented. Will only wait timeout seconds.
        Returns None if timeout expires.
        If timeout is not provided, negative or None, will wait forever.
        """
        return self.reader.read(timeout)
