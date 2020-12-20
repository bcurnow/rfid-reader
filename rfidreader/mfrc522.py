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
        DivIrqReg = 0x05  # interrupt request bits for CRC
        ErrorReg = 0x06  # error bits showing the error status of the last command executed
        Status1Reg = 0x07  # Status register for various commands
        FIFODataReg = 0x09  # input and output of 64 byte FIFO buffer
        FIFOLevelReg = 0x0A  # number of bytes stored in the FIFO buffer
        ControlReg = 0x0C  # miscellaneous control registers
        BitFramingReg = 0x0D  # adjustments for bit-oriented frames
        ModeReg = 0x11  # defines general modes for transmitting and receiving
        TxControlReg = 0x14  # controls the logical behavior of the antenna driver pins TX1 and TX2
        TxASKReg = 0x15  # controls the setting of the transmission modulation
        CRCResultRegH = 0x21  # The MSB of the CRC result
        CRCResultRegL = 0x22  # The LSB of the CRC result
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
            return self.write() | MFRC522.BIT_MASK_MSB

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
            return (self << 1) & MFRC522.BIT_MASK_REGISTER


    @unique
    class PCDCommand(IntEnum):
        """ proximity coupling device (PCD) commands """
        IDLE = 0x00  # Stop any in process commands and idle the board
        CALCCRC = 0x03  # Perform a CRC check
        TRANSCEIVE = 0x0C  # Tranceive data
        SOFT_RESET = 0x0F  # Perform a soft reset of the board


    @unique
    class ErrorCode(IntEnum):
        OK = 0  # everything is OK
        NOTAGERR = 1  # There was no tag to read
        ERR = 2  # Something went wrong


    @unique
    class PICCCommand(IntEnum):
        """ proximity inductive coupling card (PICC) commands """
        REQA = 0x26  # Is there a card in the field?
        ANTICOLL = 0x93  # Anti-collision

    MAX_SPEED_HZ = 106000  # 106 kBd - see initialize_card()
    FIFO_BUFFER_MAX_SIZE = 64  # The size (in bytes) of the FIFO buffer
    BITS_IN_BYTE = 8  # The number of bits in a byte
    NVD_DEFAULT = 0x20  # 0010 0000 -  This is the default starting point for the NVD bit.
                        # The upper 4 bits (byte count) indicate that number of bytes we're sending (set to 2)
                        # The lower 4 bits (bit count) indicates the number of bits we sending modulo 8
    BIT_MASK_ANTENNA_POWER = 0b00000011  # A bit mask for [0] and [1] (Tx1RFEn, Tx2RFEn) which indicate the current power state of the antenna
    BIT_MASK_COMIRQ_TX_RX = 0b00110000  # Bit mask for the TX and RX interrupts
    BIT_MASK_DIVIRQ_CRCIRQ = 0b00000100  # Bit mask for the CRCIRq flag in the DivIrqReg
    BIT_MASK_LSB = 0b00000001  # The least signficant bit
    BIT_MASK_MSB = 0b10000000  # The most significant bit
    BIT_MASK_REGISTER = 0b01111110  # A bit mask for the six-digit register values (dropping MSB and LSB)
    BIT_MASK_TRANCEIVE_ERRORS = 0b00011011  # A bit maks for protocol [0], partity [1], collision [3] or buffer overflow [4] errors
    BIT_MASK_TRANSCEIVE_IRQ = 0b11110111  # Turns on/maks all the interrupts except for HiAlertIEn ([3])

    GPIO_BCM_RST_DEFAULT = 15  # The default RST pin (GPIO 15) when gpio_mode is set to GPIO.BCM
    GPIO_BOARD_RST_DEFAULT = 22  # The default RST pin (GPIO 25) when gpio_mode is set to GPIO.BOARD (the default)

    # The following values configure the timer so the result is a timer delay of 0.025 seconds (~25 miliseconds/)
    # This done by setting the TPrescaler (12-bits) first 4 bits to 0 and the second 8 to 169 which results in a total value of 169
    # Then the TReload (16-bits) first 8 bits are set to 3 and the second 8 bits are set 232 which results in a total value of 1000
    # To get to the delay seconds, the following equation is used:
    # (TPrescaler * 2 + 1) * (TReload + 1)
    # ------------------------------------
    #     13.56 Mhz (13,560,000)
    # For our values this ends up as:
    # (169 * 2 + 1) * (1000 + 1) OR
    # 339 * 1001 OR
    # 339,339
    # ----------
    # 13560000
    # Which is 0.025025 of a second or 25 miliseconds
    # NOTE: According to the NXP docs, 339 clock cycles results in a delay of 25 microseconds
    #       A value of 169 for TPrescaler results in 339 in the equation so the way to think about this is:
    #       The TPrescaler represents 25 micro seconds the TReload represents how many 25 microsecond time-slots
    #       to count before triggering the timer IRQ
    TPRESCALER_HIGH_FOUR = 0b10000000  # Sets the value of TPrescaler's high 4 bits (out of 12) (value = 0)
    TPRESCLER_LOW_EIGHT = 0b10101001  # Sets the value of the TPrescaler's low 8 bits (out of 12) (value = 169)
    TRELOAD_HIGH_EIGHT = 0b00000011  # The high 8 bits of the TReload value (value = 3)
    TRELOAD_LOW_EIGHT = 0b11101000  # The low 8 bits of the TReload value (value = 232)
    ASK_MODULATION = 0b01000000  # Force a 100% ASK modulation by setting [6] = 1
    MODE_DEFAULTS = 0b00111101  # Set the CRC preset value to 0x6363 [0]-[1]
                                # Set the polarity of MFIN to HIGH [3]
                                # Ensure the transmitter can only be started if an RF field is generated [5]
                                # Set MSBFirst to false
                                # Other bits are reserved and therefore left at 0
    BIT_FRAMING_SEND_ALL_BITS = 0b00000111  # Set TxLastBits to 7 ensuring that all the bits will be transmitted
    IRQ_CHECKS = 2* ((TRELOAD_HIGH_EIGHT << 8) + TRELOAD_LOW_EIGHT)  # This is the number of IRQ checks to make for each command (e.g. TRANSCEIVE, CALCCRC)
                                                                     # This value is based on the TReload value as this indicates how many 25 microsecond delays
                                                                     # We want to check the IRQ twice any many times to ensure that our IRQ checking loop is
                                                                     # longer than the timer time as each time through the look should be ~ 25 microsecond
                                                                     # To use the TReload value, we need to combine the high and low bits so we shift the
                                                                     # high eight bits 8 places to the left and then add in the low bits. This results in
                                                                     # the total value of TReload (e.g. 1000) and allows us to base this constant on a multiple
                                                                     # of that.


    def __init__(self, bus=0, device=0, gpio_mode=GPIO.BOARD, rst_pin=None):
        self.spi = spidev.SpiDev()
        self.spi.open(bus, device)
        self.spi.max_speed_hz = MFRC522.MAX_SPEED_HZ

        GPIO.setmode(gpio_mode)

        if not rst_pin:  # auto set based on GPIO mode
            if gpio_mode == GPIO.BCM:
                rst_pin = MFRC522.GPIO_BCM_RST_DEFAULT
            else:
                rst_pin = MFRC522.GPIO_BOARD_RST_DEFAULT

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
        self.write(MFRC522.Register.TModeReg, MFRC522.TPRESCALER_HIGH_FOUR)

        # Set the lower 8 bits of the TPrescaler to 62. This makes the TPrescaler value (12-bits) = 3390
        self.write(MFRC522.Register.TPrescalerReg, MFRC522.TPRESCLER_LOW_EIGHT)

        # Setup the timer reload value
        # Sets the TReload value to 30
        self.write(MFRC522.Register.TReloadRegL, MFRC522.TRELOAD_LOW_EIGHT)
        self.write(MFRC522.Register.TReloadRegH, MFRC522.TRELOAD_HIGH_EIGHT)
        self.write(MFRC522.Register.TxASKReg, MFRC522.ASK_MODULATION)
        self.write(MFRC522.Register.ModeReg, MFRC522.MODE_DEFAULTS)

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
        if (~(current & MFRC522.BIT_MASK_ANTENNA_POWER)):
            # Set both Tx1RFEn, Tx2RFEn to 1 to turn the antenna on
            self.set_bits(MFRC522.Register.TxControlReg, MFRC522.BIT_MASK_ANTENNA_POWER)

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
        self.write(MFRC522.Register.BitFramingReg, MFRC522.BIT_FRAMING_SEND_ALL_BITS)

        (status, results, results_len) = self.transceive([MFRC522.PICCCommand.REQA])

        # According to the NXP docs the result of REQA should be ignored
        # We need both an error code of OK and for the lenth of the results to be 16 (2 bytes) in order to proceedA
        if status == MFRC522.ErrorCode.OK and results_len == 16:
            return True
        return False

    def transceive(self, data):
        self.write(MFRC522.Register.ComIEnReg, MFRC522.BIT_MASK_TRANSCEIVE_IRQ)

        # Set the Set1 bit ([7]) to 0 which clears the register
        self.unset_bits(MFRC522.Register.ComIrqReg, MFRC522.BIT_MASK_MSB)

        # Flush the FIFO buffer (set FlushBuffer ([7]) to 1)
        self.set_bits(MFRC522.Register.FIFOLevelReg, MFRC522.BIT_MASK_MSB)

        # Idle the card, cancelling any current commands
        self.write(MFRC522.Register.CommandReg, MFRC522.PCDCommand.IDLE)

        self._write_data_to_fifo(data)

        # Run the transceive command
        self.write(MFRC522.Register.CommandReg, MFRC522.PCDCommand.TRANSCEIVE)

        # Set StartSend to 1 to start the transmission of data
        self.set_bits(MFRC522.Register.BitFramingReg, MFRC522.BIT_MASK_MSB)

        countdown = MFRC522.IRQ_CHECKS

        while True:
            # Read the current status of the interrupts from the register
            interrupts = self.read(MFRC522.Register.ComIrqReg)
            countdown -= 1
            # The LSB in interrupts is the timer IRQ, if set, indicates the timer finshed a cycle
            if ~((countdown != 0) and ~(interrupts & MFRC522.BIT_MASK_LSB) and ~(interrupts & MFRC522.BIT_MASK_COMIRQ_TX_RX)):
                break

        # Stop sending (set StartSend to 0)
        self.unset_bits(MFRC522.Register.BitFramingReg, MFRC522.BIT_MASK_MSB)

        status = MFRC522.ErrorCode.ERR
        results = []
        results_len = 0
        # Check if we timed out (countdown == 0)
        if countdown != 0:
            # Check to see if there are any protocol, partity, collision or buffer overflow errors
            if (self.read(MFRC522.Register.ErrorReg) & MFRC522.BIT_MASK_TRANCEIVE_ERRORS) == 0:
                status = MFRC522.ErrorCode.OK

                # Check to see if the timer counted down
                # Filter the interrupts for only the ones we care about (& BIT_MASK_TRANSCEIVE_IRQ)
                # Then specifically look at the LSB as it indicates a timer interrupt
                if interrupts & MFRC522.BIT_MASK_TRANSCEIVE_IRQ & MFRC522.BIT_MASK_LSB:
                    status = MFRC522.ErrorCode.NOTAGERR

                # Check how many bytes were written to the FIFO
                bytes_written = self.read(MFRC522.Register.FIFOLevelReg)
                # Check [0][1][2] to see how many valid bits in the last byte (0 (zero) indicates the whole byte is valid)
                bits_in_last_byte = self.read(MFRC522.Register.ControlReg) & 0b00000111
                if bits_in_last_byte != 0:
                    results_len = (bytes_written - 1) * MFRC522.BITS_IN_BYTE + bits_in_last_byte
                else:
                    results_len = bytes_written * MFRC522.BITS_IN_BYTE

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

        data = [MFRC522.PICCCommand.ANTICOLL, MFRC522.NVD_DEFAULT]
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

        # Make a SAK call
        self.transceive([])
        return (status, results, results_len)


    def calculate_crc(self, data):
        # Clear the CRC IRQ bit
        self.unset_bits(MFRC522.Register.DivIrqReg, MFRC522.BIT_MASK_DIVIRQ_CRCIRQ)
        # Flush the FIFO buffer (set FlushBuffer ([7]) to 1)
        self.set_bits(MFRC522.Register.FIFOLevelReg, MFRC522.BIT_MASK_MSB)
        self._write_data_to_fifo(data)

        # Calculate the CRC
        self.write(MFRC522.Register.CommandReg, MFRC522.PCDCommand.CALCCRC)

        countdown = MFRC522.IRQ_CHECKS
        while True:
            interrupts = self.read(MFRC522.Register.DivIrqReg)
            countdown -= 1
            if not ((countdown != 0) and not (n & MFRC522.BIT_MASK_DIVIRQ_CRCIRQ)):
                break
        rv = []
        rv.append(self.read(MFRC522.Register.CRCResultRegL))
        rv.append(self.read(MFRC522.Register.CRCResultRegH))
    return rv

    def _write_data_to_fifo(self, data):
        # Write the data to the FIFO Data register
        for datum in data:
            self.write(MFRC522.Register.FIFODataReg, datum)
