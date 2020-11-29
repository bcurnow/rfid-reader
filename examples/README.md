This directory contains an example that shows the intended implementation for rfidsecurity. `reader.py` will run an infinite loop of reading and printing to stdout. `api` will make a one-time read and print it but it will `grab` the device first meaning that its read will only show up in its' stdout as the `reader.py` process won't be able to see it.

It's helpful to run each of these with unbuffered stdout:

* `python -u reader.py`
* `python -u api.py`
