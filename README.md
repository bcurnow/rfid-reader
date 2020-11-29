# rfid-reader
Common Python library for reading from the RFID reader that presents itself as a keyboard. Wraps evdev.

## RFIDReader Usage

```
reader = RFIDReader('/dev/input/rfid')
  while True:
      event = reader.read()
      if event:
          print(event)
```

The read method reads a tag and returns the info as a string or None if the timeout was exceeded. If the timeout is not specified or is `None` then it will wait infinitely using `select.poll()`.

## Examples
See the `examples` folder for more examples.
