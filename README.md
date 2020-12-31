<!-- MDTOC maxdepth:6 firsth1:1 numbering:0 flatten:0 bullets:1 updateOnSave:1 -->

- [rfid-reader](#rfid-reader)   
   - [RFIDReader Usage](#rfidreader-usage)   
   - [Implementations](#implementations)   
      - [evdev](#evdev)   
         - [Additional Configuration](#additional-configuration)   
      - [mfrc522](#mfrc522)   
         - [Additional Configuration](#additional-configuration)   
   - [Examples](#examples)   

<!-- /MDTOC -->
# rfid-reader
Common Python library for reading from various RFID readers (e.g. evdev, mfrc522).

## RFIDReader Usage

```
reader = RFIDReader(<type>, <config>)
  while True:
      event = reader.read()
      if event:
          print(event)
```
* `<type>` - A supported implementation type. This library supplies implementations with types `evdev` and `mfrc522`.
* `<config>` - A dictionary of configuration parameters. These will be passed to the implementation without modification.

The read method reads a tag and returns the id as a string or None if the timeout was exceeded. If the timeout is not specified or is `None` then it will wait infinitely.

## Implementations

### evdev
This implementation is based on [evdev](https://pypi.org/project/evdev/). This is used for RFID readers that show up as input devices and can be read like keyboards.

#### Additional Configuration
This implementation supports the following configuration parameters passed in via `<config>`:
* `device_name` - The device name the reader exists as. Default: `/dev/input/event0`
* `event_ready_timeout` - The amount of time to wait for the next set of data once the `select` call has indicated there is data. In normal operations, this should not need to be changed. Default: `100`

### mfrc522
This implementation is based on [mfrc522-reader](https://github.com/bcurnow/mfrc522-reader). This is used for MFRC522 RFID readers that connect via SPI.

#### Additional Configuration
This implementation supports the following configuration parameters passed in via `<config>`:
* `bus` - The bus index to connect to. This is used to construct the name of the device. For example, `/dev/spidev<bus>.<device>`. Default: `0`
* `device` - The device index to connect to. This is used to construcct the name of the device. For example, `/dev/spidev<bus>.<device>`. Default: `0`
* `gpio_mode` -  The numbering system used. Default: `10` (GPIO.BOARD)
* `rst_pin` - The output pin. If set to `-1` and `pin_mode` is 11 (GPIO.BCM) then this is set to 15, if `-1` and `pin_mode` is 10 (GPIO.BOARD) this is set to 22. If set to a value other than `-1` then that value will be used as the output pin. Default: `-1`

## Examples
See the `examples` folder for more examples.
