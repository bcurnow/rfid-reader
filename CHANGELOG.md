<!-- MDTOC maxdepth:6 firsth1:1 numbering:0 flatten:0 bullets:1 updateOnSave:1 -->

- [2.0.0](#200)   
- [1.0.1](#101)   
- [1.0.0](#100)   

<!-- /MDTOC -->

# 2.0.0
NOTE: Breaking Changes

* Updated the RFIDReader class to simply be a wrapper around a specific implementation. This allows support for different RFID readers such as evdev and mfrc522 (which are included).
* RFIDReader now only takes a dict of configuration options which are passed, as-is, to the implementation classes, this means that for evdev (for example), you need to pass the device_name via the dict.
* Added MFRC522 implementation

# 1.0.1
* Fixed issue with `__del__` where if the device passed in doesn't exist there is no `device` attribute and this causes an error when destructing.

# 1.0.0
* Initial release
