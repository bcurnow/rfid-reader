import select
import evdev


def register(config):
    return EvdevReader(config)


class EvdevReader:
    """ This class allows reading for a keyboard presented device using evdev."""

    """ The timeout to use once we've determined there are events ready """
    EVENT_READY_TIMEOUT = 100

    """ Maps the hex key codes (0-9A-F) to characters so we can translate events from the reader."""
    KEY_MAP = {
        'KEY_0': '0',
        'KEY_1': '1',
        'KEY_2': '2',
        'KEY_3': '3',
        'KEY_4': '4',
        'KEY_5': '5',
        'KEY_6': '6',
        'KEY_7': '7',
        'KEY_8': '8',
        'KEY_9': '9',
        'KEY_A': 'A',
        'KEY_B': 'B',
        'KEY_C': 'C',
        'KEY_D': 'D',
        'KEY_E': 'E',
        'KEY_F': 'F',
    }

    def __init__(self, config):
        if 'event_ready_timeout' in config:
            self.event_ready_timeout = int(config['event_ready_timeout'])
        else:
            self.event_ready_timeout = EvdevReader.EVENT_READY_TIMEOUT

        if 'device_name' in config:
            self.device_name = config['device_name']
        else:
            self.device_name = '/dev/input/event0'

        self.device = evdev.InputDevice(self.device_name)
        self.poller = select.poll()
        self.poller.register(self.device, select.POLLIN)

    def __del__(self):
        try:
            if hasattr(self, 'device'):
                self.device.close()
        except RuntimeError:
            # Need to catch RuntimeError due to https://github.com/gvalkov/python-evdev/issues/120
            pass

    def read(self, timeout=None):
        """
        Reads the next RFID tag presented. Will only wait timeout seconds.
        Returns None if timeout expires.
        If timeout is not provided, negative or None, will wait forever.
        """
        if timeout and timeout >= 0:
            timeout = timeout * 1000
        # Use the supplied timeout to wait for a tag to be read, this timeout is longer than EVENT_READY_TIMEOUT
        # because we're waiting on a human to take an action
        if self.poller.poll(timeout):
            return self._read_rfid()

    def _read_rfid(self):
        keep_reading = True
        data = []

        while keep_reading:
            self._read_all_available_events(data)
            keep_reading = self.poller.poll(self.event_ready_timeout)

        # All possible events have been read, convert the data to a string
        if data:
            return ''.join(data)

    def _read_all_available_events(self, data):
        """ Reads all current ready events from device and adds the translated data to the data list."""
        try:
            for raw_event in self.device.read():
                event = evdev.categorize(raw_event)
                translated_event = self._translate_event(event)
                if translated_event:
                    data.append(translated_event)
        except BlockingIOError:
            # This indicates there are no more events to read
            # In theory this will never happen as we check before we read
            # If it happens, ignore it
            pass

    def _translate_event(self, event):
        """ Handle KeyEvents and specifically key_down events as these indicate data we care about. """
        if isinstance(event, evdev.events.KeyEvent) and event.keystate == evdev.events.KeyEvent.key_down:
            if event.keycode in EvdevReader.KEY_MAP:
                return EvdevReader.KEY_MAP[event.keycode]
