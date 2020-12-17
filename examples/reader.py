#!/usr/bin/env python
""" This script uses the rfidereader.RFIDReader to loop forever and read events from /dev/input/rfid and prints the resulting strings to stdout """
import sys

from rfidreader import RFIDReader

if __name__ == '__main__':
    reader = RFIDReader(sys.argv[1], {})
    print('Awaiting input...')
    while True:
        event = reader.read()
        if event:
            print(event)
