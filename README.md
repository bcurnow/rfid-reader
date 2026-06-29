<!-- MDTOC maxdepth:6 firsth1:1 numbering:0 flatten:0 bullets:1 updateOnSave:1 -->

- [rfid-reader](#rfid-reader)   
   - [RFIDReader Usage](#rfidreader-usage)   
   - [Implementations](#implementations)   
      - [evdev](#evdev)   
         - [Additional Configuration](#additional-configuration)   
      - [mfrc522](#mfrc522)   
         - [Additional Configuration](#additional-configuration)   
   - [Development](#development)   
      - [Makefile](#makefile)   
      - [Local Python Environment](#local-python-environment)   
      - [Docker Environment](#docker-environment)   
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

## Development

### Makefile

A `Makefile` is provided with targets for common development tasks:

```
make setup        Create .venv and install all dependencies
make install      Install dependencies into the active environment
make test         Run the test suite
make coverage     Run tests and report coverage
make lint         Check code with ruff
make format       Format code with ruff
make build        Build the Python wheel
make clean        Remove build artifacts (CLEAN_VENV=1 also removes .venv)
make release      Create a GitHub release (VERSION=x.y.z)
make docker-build Build the Docker image
make docker-run   Run a shell in the Docker container
```

Run `make help` to see this list at any time.

### Local Python Environment

On Linux, ensure the following system packages are installed before running `make setup`:

```bash
sudo apt-get install build-essential python3-dev
```

Use the `setup` target to create a virtual environment and install all dependencies:

```bash
make setup
source .venv/bin/activate
```

After activating, use `make test`, `make lint`, etc. directly — they resolve to the venv Python automatically.

### Docker Environment

Docker is the preferred environment when testing against real hardware (evdev input devices or SPI-based MFRC522 readers), as it handles group membership for `/dev/input` access.

Build the image:

```bash
make docker-build
```

Run a shell inside the container (mounts the project root and `/dev`):

```bash
make docker-run
```

Inside the container, run tests or start an interactive session with full hardware access. The container runs as a non-root user that belongs to the `input` group, matching your host's group IDs.

## Examples
See the `examples` folder for more examples.
