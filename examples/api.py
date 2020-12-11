#!/usr/bin/env python
"""
This script is simulating what we want the API to do for the occasional read.
This code will grab the device which will make it exclusive to that process.
Then it will initiate a read with a timeout (to pause for human usage).
Once it's read a single event, it will ungrab the device which will let
the normal reader process continue.
"""

from rfidreader import RFIDReader

if __name__ == '__main__':
    reader = RFIDReader('/dev/input/rfid')
    reader.device.grab()
    print('acquired lock')
    event = reader.read(20)
    if event:
        print(event)
    reader.device.ungrab()
    print('released lock')
    reader.__del__()
