#!/usr/bin/env python
""" This script uses the rfidereader.RFIDReader to loop forever and read events from /dev/input/rfid and prints the resulting strings to stdout """
from rfidreader import RFIDReader

if __name__ == '__main__':
    reader = RFIDReader('/dev/input/rfid')
    while True:
        event = reader.read()
        if event:
            print(event)
