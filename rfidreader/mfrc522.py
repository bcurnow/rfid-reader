"""
This code is heavily influenced by https://github.com/pimylifeup/MFRC522-python and https://github.com/miguelbalboa/rfid but has been adjusted
so that it's better commented and suited specifically for what this library requires (e.g. reading the uid off a tag).
The additional documentation comes from NXP's data-sheet on the MFRC522: https://www.nxp.com/docs/en/data-sheet/MFRC522.pdf
"""
import atexit
from enum import IntEnum, unique

import RPi.GPIO as GPIO
import spidev



class MFRC522:
    @unique
    class Register(IntEnum):
        CommandReg = 0x01  # starts and stops command execution
        ComIEnReg = 0x02  # enable and disable interrupt request control bits
        ComIrqReg = 0x04  # interrupt request bits
        ErrorReg = 0x06  # error bits showing the error status of the last command executed
        FIFODataReg = 0x09  # input and output of 64 byte FIFO buffer
        FIFOLevelReg = 0x0A  # number of bytes stored in the FIFO buffer
        ControlReg = 0x0C  # miscellaneous control registers
        BitFramingReg = 0x0D  # adjustments for bit-oriented frames
        ModeReg = 0x11  # defines general modes for transmitting and receiving
        TxControlReg = 0x14  # controls the logical behavior of the antenna driver pins TX1 and TX2
        TxASKReg = 0x15  # controls the setting of the transmission modulation
        TModeReg = 0x2A  # timer settings
        TPrescalerReg = 0x2B  # defines settings for the internal timer
        TReloadRegH = 0x2C  # defines the 16-bit timer reload MSB value
        TReloadRegL = 0x2D  # defines the 16-bit timer reload LSB value

        def read(self):
            """
            Registers are represented by a 6-bit value where the LSB is always 0 (zero)
            and the MSB indicates the mode: 0 for write, 1 for read.
            We use the write method first and then bitwise OR with 1000 0000
            to ensure that the MSB is 1.
            """
            return self.write() | 0b10000000

        def write(self):
            """
            Registers are represented by a 6-bit value where the LSB is always 0 (zero)
            and the MSB indicates the mode: 0 for write, 1 for read.
            To ensure that the LSB is always zero, we bit shift 1 place to the left.
            To ensure that the MSB is 0, we bitwise AND with 0111 1110
            Example:
              register = 0x03 = 0000 0011
              bit shift << 1 = 0000 0110
              AND with 0111 1110

                0000 0110
              & 0111 1110
              -----------
                0000 0110

            If you now drop the MSB and LSB you're left with 000011 which is still 0x03
            """
            return (self << 1) & 0b01111110


    @unique
    class PCDCommand(IntEnum):
        """ proximity coupling device (PCD) commands """
        IDLE = 0x00
        TRANSCEIVE = 0x0C
        SOFT_RESET = 0x0F


    @unique
    class ErrorCode(IntEnum):
        OK = 0  # everything is OK
        NOTAGERR = 1  # There was no tag to read
        ERR = 2  # Something went wrong


    @unique
    class PICCCommand(IntEnum):
        """ proximity inductive coupling card (PICC) commands """
        REQIDL = 0x26  # Is there a card in the field?
        ANTICOLL = 0x93  # Anti-collision

    MAX_SPEED_HZ = 106000  # 106 kBd - see initialize_card()
    FIFO_BUFFER_MAX_SIZE = 64  # The size (in bytes) of the FIFO buffer
    THIRTY_SECONDS = 2000  # The internal time is configured to ~ 0.015 seconds, 2000 * 0.015 = 30 (seconds)
    BITS_IN_BYTE = 8  # The number of bits in a byte


    def __init__(self, bus=0, device=0, gpio_mode=GPIO.BOARD, rst_pin=None):
        self.spi = spidev.SpiDev()
        self.spi.open(bus, device)
        self.spi.max_speed_hz = MFRC522.MAX_SPEED_HZ

        GPIO.setmode(gpio_mode)

        if not rst_pin:  # auto set based on GPIO mode
            if gpio_mode == GPIO.BCM:
                rst_pin = 15  # Defaults to GPIO 15 (BOARD pin 10)
            else:
                rst_pin = 22  # Defaults to GPIO 25 (BOARD pin 22)

        # Setup the reset pin as an output
        GPIO.setup(rst_pin, GPIO.OUT)
        GPIO.output(rst_pin, GPIO.HIGH)

        # Make sure we cleanup before we exit
        atexit.register(self.close)
        self.initialize_card()

    def close(self):
        if self.spi:
            self.spi.close()
        GPIO.cleanup()

    def initialize_card(self):
        self.soft_reset()

        # Setup the timer TPrescaler value and reload interval
        # The two time sections below will set the timer value to: ~0.015 seconds due to the following equation:
        # (TPrescaler * 2 + 1) * (TReload + 1)
        # ------------------------------------
        #        13,560,000 (13.56 Mhz)

        # Set the 4 high bits of the TPrescaler to 1101 (13) and setting TAuto to start the timer automatically at the end of the transmission
        self.write(MFRC522.Register.TModeReg, 0b10001101)

        # Set the lower 8 bits of the TPrescaler to 62. This makes the TPrescaler value (12-bits) = 3390
        self.write(MFRC522.Register.TPrescalerReg, 62)

        # Setup the timer reload value
        # Sets the TReload value to 30
        self.write(MFRC522.Register.TReloadRegL, 30)
        self.write(MFRC522.Register.TReloadRegH, 0)

        # force a 100% ASK modulation by setting [6] = 1
        self.write(MFRC522.Register.TxASKReg, 0b01000000)


        # Set the CRC preset value to 0x6363 [0]-[1]
        # Set the polarity of MFIN to HIGH [3]
        # Ensure the transmitter can only be started if an RF field is generated [5]
        # Set MSBFirst to false
        # Other bits are reserved and therefore left at 0
        self.write(MFRC522.Register.ModeReg, 0b00111101)

        self.antenna_on()

    def soft_reset(self):
        """ Executes a soft reset on the card to reset all registers to default values (internal buffer is not changed). """
        self.write(MFRC522.Register.CommandReg, MFRC522.PCDCommand.SOFT_RESET)

    def read(self, register):
        """
        Reads a value from the card.
        register: The register to read from
        """
        val = self.spi.xfer2([register.read(), 0])
        return val[1]

    def write(self, register, value):
        """
        Writes to the card.
        register: The register to write to
        value: The value to write
        """
        self.spi.xfer2([register.write(), value])

    def antenna_on(self):
        # Read the current configuration from the register
        current = self.read(MFRC522.Register.TxControlReg)

        # Determine if the antenna is currently off, if it is, turn it back on
        # To determine the state of the antenna, we only need [0] and [1] (Tx1RFEn, Tx2RFEn) so AND with 3
        # If either of the first two bits are 0 (zero), then the antenna is off
        # By bitwise NOT'ing the value, we can check if the result is zero or not, if not zero, then it wasn't on
        if (~(current & 0b00000011)):
            # Set both Tx1RFEn, Tx2RFEn to 1 to turn the antenna on
            self.set_bits(MFRC522.Register.TxControlReg, 0b00000011)

    def set_bits(self, register, mask):
        """ Reads the current value and then sets the specified bits to 1 """
        current = self.read(register)
        # bitwise or (|) will ensure that all bit posision set to 1 in the mask are set to 1 in the current value
        self.write(register, current | mask)

    def unset_bits(self, register, mask):
        """ Reads the current value and then sets the specified bits to 0 """
        current = self.read(register)
        # bitwise and (&) with a NOT'd (~) maks will ensure that all bit positions set to 1 in the mask are set to 0 in the current value
        self.write(register, current & (~mask))

    def card_present(self):
        # Set TxLastBits to 7 ensuring that all the bits will be transmitted
        self.write(MFRC522.Register.BitFramingReg, 0b00000111)

        (status, results, results_len) = self.transceive([MFRC522.PICCCommand.REQIDL])

        # TODO covert status to boolean, determine if we need the results or len for anything else
        # We need both an error code of OK and for the lenth of the results to be 16 in order to proceedA
        print('REQIDL', results, results_len)
        if ((status != MFRC522.ErrorCode.OK) | (results_len != 16)):
            status = MFRC522.ErrorCode.ERR

        return (status, results, results_len)

    def transceive(self, data):
        # Turn on all interrupts except for HiAlertIEn ()
        irq_bitmask = 0b11110111
        self.write(MFRC522.Register.ComIEnReg, irq_bitmask)

        # Set the Set1 bit ([7]) to 0 which clears the register
        self.unset_bits(MFRC522.Register.ComIrqReg, 0b10000000)

        # Flush the FIFO buffer (set FlushBuffer ([7]) to 1)
        self.set_bits(MFRC522.Register.FIFOLevelReg, 0b10000000)

        # Idle the card, cancelling any current commands
        self.write(MFRC522.Register.CommandReg, MFRC522.PCDCommand.IDLE)

        # Write the data to the FIFO Data register
        for datum in data:
            self.write(MFRC522.Register.FIFODataReg, datum)

        # Run the transceive command
        self.write(MFRC522.Register.CommandReg, MFRC522.PCDCommand.TRANSCEIVE)

        # Set StartSend to 1 to start the transmission of data
        self.set_bits(MFRC522.Register.BitFramingReg, 0b10000000)

        countdown = THIRTY_SECONDS
        tx_rx_bitmask = 0b00110000

        while True:
            # Read the current status of the interrupts from the register
            interrupts = self.read(MFRC522.Register.ComIrqReg)
            print(countdown)
            countdown -= 1
            #print(countdown != 0, interrupts & 0x01, interrupts & tx_rx_bitmask)
            #print(~((countdown != 0) and (interrupts & 0x01) and (interrupts & tx_rx_bitmask)))
            # The LSB in interrupts is the timer IRQ, if set, indicates the timer finshed a cycle
            # Bits [4] and [5] indicate that TX and RX are complete
            # while ((i!=0) && !(n&0x01) && !(n&waitIRq));
            print(interrupts)
            if ~((countdown != 0) and ~(interrupts & 0b00000001) and ~(interrupts & tx_rx_bitmask)):
                break

        # Stop sending (set StartSend to 0)
        self.unset_bits(MFRC522.Register.BitFramingReg, 0b10000000)

        status = MFRC522.ErrorCode.ERR
        results = []
        results_len = 0
        # Check if we timed out (countdown == 0)
        if countdown != 0:
            # Check to see if there are any protocol [0], partity [1], collision[3] or buffer overflow errors[4]
            if (self.read(MFRC522.Register.ErrorReg) & 0b00011011):
                status = MFRC522.ErrorCode.OK

                # Check to see if the timer counted down
                # Filter the interrupts for only the ones we care about (& irq_bitmaks)
                # Then specifically look at the LSB as it indicates a timer interrupt
                if interrupts & irq_bitmask & 0b00000001:
                    status = MFRC522.ErrorCode.NOTAGERR

                # Check how many bytes were written to the FIFO
                bytes_written = self.read(MFRC522.Register.FIFOLevelReg)
                # Check [0][1][2] to see how many valid bits in the last byte (0 (zero) indicates the whole byte is valid)
                bits_in_last_byte = self.read(MFRC522.Register.ControlReg) & 0b00000111
                if bits_in_last_byte != 0:
                    results_len = (bytes_written - 1) * BITS_IN_BYTE + bits_in_last_byte
                else:
                    results_len = bytes_written * BITS_IN_BYTE

                if bytes_written == 0:
                    # Always the current value of the buffer
                    bytes_written = 1
                if bytes_written > MFRC522.FIFO_BUFFER_MAX_SIZE:
                    bytes_written = MFRC522.FIFO_BUFFER_MAX_SIZE

                for i in range(bytes_written):
                    # Read the data from the FIFO
                    results.append(self.read(MFRC522.Register.FIFODataReg))
            else:
                status = MFRC522.ErrorCode.ERR

        return (status, results, results_len)

    def anticollision(self):
        # Reset the the bit-oriented frame settings
        self.write(MFRC522.Register.BitFramingReg, 0b00000000)

        # TODO why 0x20?
        data = [MFRC522.PICCCommand.ANTICOLL, 0x20]
        (status, results, results_len) = self.transceive(data)

        if (status == MFRC522.ErrorCode.OK):
            i = 0
            # TODO Support longer uids
            if len(results) == 5:
                serial_number_check = 0
                for i in range(4):
                    serial_number_check = serial_number_check ^ results[i]
                if serial_number_check != results[4]:
                    status = MFRC522.ErrorCode.ERR
            else:
                status = MFRC522.ErrorCode.ERR

        return (status, results, results_len)
