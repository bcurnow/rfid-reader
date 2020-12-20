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
        ComIrqReg = 0x04  # interrupt request bits
        DivIrqReg = 0x05  # interrupt request bits for CRC
        ErrorReg = 0x06  # error bits showing the error status of the last command executed
        Status1Reg = 0x07  # Status register for various commands
        FIFODataReg = 0x09  # input and output of 64 byte FIFO buffer
        FIFOLevelReg = 0x0A  # number of bytes stored in the FIFO buffer
        ControlReg = 0x0C  # miscellaneous control registers
        BitFramingReg = 0x0D  # adjustments for bit-oriented frames
        CollReg = 0x0E  # bit-collision detection
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
        """ The actual values don't matter. """
        OK = 0  # everything is OK
        NOTAGERR = 1  # There was no tag to read
        ERR = 2  # Something went wrong (e.g. parity error, buffer oveflow, protocol error)
        TIMEOUT = 3  # The timer completed before we were done
        COUNTDOWN_TIMEOUT = 4  # Our countdown completed without the timer going off
        COLLISION = 5  # A collision error happened
        UNKNOWN_COLLISION_ERROR = 6  # A collision happened but the position is within the data we already know which shouldn't be possible



    @unique
    class PICCCommand(IntEnum):
        """ proximity inductive coupling card (PICC) commands """
        REQA = 0x26  # Is there a card in the field?
        ANTICOLL_CS1 = 0x93  # Anti-collision cascade level 1
        ANTICOLL_CS2 = 0x95  # Anti-collision cascade level 2
        ANTICOLL_CS3 = 0x95  # Anti-collision cascade level 3

    MAX_SPEED_HZ = 106000  # 106 kBd - see initialize_card()
    FIFO_BUFFER_MAX_SIZE = 64  # The size (in bytes) of the FIFO buffer
    BITS_IN_BYTE = 8  # The number of bits in a byte
    BIT_MASK_ANTENNA_POWER = 0b00000011  # A bit mask for [0] and [1] (Tx1RFEn, Tx2RFEn) which indicate the current power state of the antenna
    BIT_MASK_COLLREG_POSITION_NOT_VALID = 0b00100000  # A bit mask that pulls CollPosNotValid [5]
    BIT_MASK_COLLREG_POSITION = 0b00011111  # A bit amsk that pulls CollPos [0]-[4]
    BIT_MASK_COMIRQ_RX_AND_IDLE = 0b00110000  # Bit mask for the RX (receive) and Idle (command complete) interrupts
    BIT_MASK_DIVIRQ_CRCIRQ = 0b00000100  # Bit mask for the CRCIRq flag in the DivIrqReg
    BIT_MASK_LSB = 0b00000001  # The least signficant bit
    BIT_MASK_MSB = 0b10000000  # The most significant bit
    BIT_MASK_REGISTER = 0b01111110  # A bit mask for the six-digit register values (dropping MSB and LSB)
    BIT_MASK_TRANCEIVE_ERRORS = 0b00010011  # A bit mask for protocol [0], partity [1], or buffer overflow [4] errors
    BIT_MASK_COLLISION_ERRORS = 0b00001000  # A bit mask for collision [3] errors
    BIT_MASK_TRANSCEIVE_IRQ = 0b11110111  # Turns on/masks all the interrupts except for HiAlertIEn ([3])

    GPIO_BCM_RST_DEFAULT = 15  # The default RST pin (GPIO 15) when gpio_mode is set to GPIO.BCM
    GPIO_BOARD_RST_DEFAULT = 22  # The default RST pin (GPIO 25) when gpio_mode is set to GPIO.BOARD (the default)

    # The following values configure the timer so the result is a timer delay of 0.025 seconds (~25 miliseconds)
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
    BIT_FRAMING_SHORT_FRAME_FORMAT = 0b00000111  # Set TxLastBits to 7 which is the short frame format
    TRANSCEIVE_CHECKS = 2 * ((TRELOAD_HIGH_EIGHT << 8) + TRELOAD_LOW_EIGHT)  # This is the number of IRQ checks to make for each command (e.g. TRANSCEIVE, CALCCRC)
                                                                     # This value is based on the TReload value as this indicates how many 25 microsecond delays
                                                                     # We want to check the IRQ twice any many times to ensure that our IRQ checking loop is
                                                                     # longer than the timer time as each time through the look should be ~ 25 microsecond
                                                                     # To use the TReload value, we need to combine the high and low bits so we shift the
                                                                     # high eight bits 8 places to the left and then add in the low bits. This results in
                                                                     # the total value of TReload (e.g. 1000) and allows us to base this constant on a multiple
                                                                     # of that.
    CRC_CHECKS = 5 * ((TRELOAD_HIGH_EIGHT << 8) + TRELOAD_LOW_EIGHT)  # See TRANSCEIVE_CHECKS for an explanation (we want more time for CRC)
    CASCADE_TAG = 0x88  # The value to pass when performing anticollision. Used by cascade levels 1 and 2
    COLLISION_POSITION_32 = 0b00000000  # Indicates a collision in the 32nd bit
    COLLISION_POSITION_1 = 0b00000001  # Indicates a collision in the 1st bit
    COLLISION_POSITION_8 = 0b00001000  # Indicates a collision in the 8th bit
    NVB_SEVEN_BYTES = 0b01110000  # We will be transferring all 7 possible bytes (command, NVB, plus all 5 uid bits)


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

    def card_present(self):
        """ Returns true if there are any type A PICCs in the field, False otherwise. """
        status, results, results_len = self.req_type_a()

        # According to the NXP docs the content of the ATQA should be ignored
        # However, we will check both the status and that we received 16 bits (2 bytes)
        if status == MFRC522.ErrorCode.OK and results_len == 16:
            return True
        return False

    def read_uid(self):
        """
        Checks for the presence of a type A PICC and, if found, executes anticollision and returns the uid
        If type A PICC not found or anticollision fails for any reason, returns None
        """
        pass

    def close(self):
        if self.spi:
            self.spi.close()
        GPIO.cleanup()

    def initialize_card(self):
        self.soft_reset()
        self.write(MFRC522.Register.TModeReg, MFRC522.TPRESCALER_HIGH_FOUR)
        self.write(MFRC522.Register.TPrescalerReg, MFRC522.TPRESCLER_LOW_EIGHT)
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
        cur_val = self.read(MFRC522.Register.TxControlReg)

        # Determine if the antenna is currently off, if it is, turn it back on
        # To determine the state of the antenna, we only need [0] and [1] (Tx1RFEn, Tx2RFEn) so AND with 3
        # If either of the first two bits are 0 (zero), then the antenna is off
        # By bitwise NOT'ing the value, we can check if the result is zero or not, if not zero, then it wasn't on
        if (~(cur_val & MFRC522.BIT_MASK_ANTENNA_POWER)):
            # Set both Tx1RFEn, Tx2RFEn to 1 to turn the antenna on
            self.set_bits(MFRC522.Register.TxControlReg, MFRC522.BIT_MASK_ANTENNA_POWER)

    def set_bits(self, register, mask):
        """ Reads the current value and then sets the specified bits to 1 """
        cur_val = self.read(register)
        # bitwise or (|) will ensure that all bit posision set to 1 in the mask are set to 1 in the current value
        self.write(register, cur_val | mask)

    def unset_bits(self, register, mask):
        """ Reads the current value and then sets the specified bits to 0 """
        cur_val = self.read(register)
        # bitwise and (&) with a NOT'd (~) maks will ensure that all bit positions set to 1 in the mask are set to 0 in the current value
        self.write(register, cur_val & (~mask))

    def req_type_a(self):
        """ Check to see if type A PICC's are in the field. """
        self._clear_bits_after_collision()
        # Setup short frame format bit framing
        self.write(MFRC522.Register.BitFramingReg, MFRC522.BIT_FRAMING_SHORT_FRAME_FORMATS)
        return self.transceive([MFRC522.PICCCommand.REQA])

    def transceive(self, data):
        # Idle the card, cancelling any current commands
        self.write(MFRC522.Register.CommandReg, MFRC522.PCDCommand.IDLE)

        # Set the Set1 bit ([7]) to 0 which clears the IRQ register
        self.unset_bits(MFRC522.Register.ComIrqReg, MFRC522.BIT_MASK_MSB)

        # Flush the FIFO buffer (set FlushBuffer ([7]) to 1)
        self.set_bits(MFRC522.Register.FIFOLevelReg, MFRC522.BIT_MASK_MSB)

        self._write_data_to_fifo(data)

        # TODO figure out how to handle rxAlign and validBits (if necessary)
        #      there may need to be a BitFraming command here

        # Run the transceive command
        self.write(MFRC522.Register.CommandReg, MFRC522.PCDCommand.TRANSCEIVE)

        # Set StartSend to 1 to start the transmission of data
        self.set_bits(MFRC522.Register.BitFramingReg, MFRC522.BIT_MASK_MSB)

        # Setup default return values
        results = []
        results_len = 0

        # The countdown is a failsafe against the timer IRQ never firing
        # If we checked in a while True loop and the timer IRQ never fired and neither did the RX or Idle IRQs
        # we'd just wait for ever. For example, this would happen if we lost communication with the card.
        # This limits the wait to a reasonable value
        countdown = MFRC522.TRANSCEIVE_CHECKS
        while countdown > 0:
            countdown -= 1
            # Read the current status of the interrupts from the register
            interrupts = self.read(MFRC522.Register.ComIrqReg)

            if (interrupts & MFRC522.BIT_MASK_COMIRQ_RX_AND_IDLE):
                # Either receiving is complete or the command completed, either way, we're done waiting
                break

            if (interrupts & MFRC522.BIT_MASK_LSB):
                # timer interrupt - nothing received
                return (MFRC522.ErrorCode.TIMEOUT, results, results_len)

        if countdown == 0:
            # None of the interrupts fired before the countdown finished
            # Did we lose connectivity with the MFRC522?
            return (MFRC522.ErrorCode.ERR.COUNTDOWN_TIMEOUT, results, results_len)

        errors = self.read(MFRC522.Register.ErrorReg)
        if  errors & MFRC522.BIT_MASK_TRANCEIVE_ERRORS:
            # We had an error
            return (MFRC522.ErrorCode.ERR, results, results_len)

        # Check how many bytes were written to the FIFO
        bytes_written = self.read(MFRC522.Register.FIFOLevelReg)
        # Check [0][1][2] to see how many valid bits in the last byte (0 (zero) indicates the whole byte is valid)
        bits_in_last_byte = self.read(MFRC522.Register.ControlReg) & 0b00000111
        if bits_in_last_byte != 0:
            results_len = (bytes_written - 1) * MFRC522.BITS_IN_BYTE + bits_in_last_byte
        else:
            results_len = bytes_written * MFRC522.BITS_IN_BYTE

        if bytes_written > MFRC522.FIFO_BUFFER_MAX_SIZE:
            bytes_written = MFRC522.FIFO_BUFFER_MAX_SIZE

        for i in range(bytes_written):
            # Read the data from the FIFO
            results.append(self.read(MFRC522.Register.FIFODataReg))

        if errors & MFRC522.BIT_MASK_COLLISION_ERRORS:
            return (MFRC522.ErrorCode.COLLISION, results, results_len)

        # Normally, you'd want to check if a CRC was requested
        # However, I don't need to actually read MiFare data (which is when this is requested)
        # So I didn't implement that check

        # If we reached this point, all the checks have passed, we can return OK
        return (MFRC522.ErrorCode.OK, results, results_len)

    def anticollision(self, cascade_level = MFRC522.PCDCommand.ANTICOLL_CS1, uid=[], valid_bits=0):
        self._clear_bits_after_collision()

        finished = False
        while not finished:
            if cascade_level == MFRC522.PCDCommand.ANTICOLL_CS1:
                use_cascade_tag = valid_bits and len(uid) > 4
                uid_start_index = 0  # We know nothing yet

            if cascade_level == MFRC522.PCDCommand.ANTICOLL_CS2:
                use_cascade_tag = valid_bits and len(uid) > 7
                uid_start_index = 3  # We know about 4 bytes

            if cascade_level == MFRC522.PCDCommand.ANTICOLL_CS3:
                use_cascade_tag = False  # Never used in cascade level 3
                uid_start_index = 6  # We know about 7 bytes


            # The anticollision process is broken into two parts: anticollision proper and selection
            # Anticollision is executed when we don't know anything about the UID (because we can't select the one we want)
            # We wait until we have a single PICC in the field so we can get the first part of the UID and select after that
            # Once we have the at least 4 bytes of the UID, we can proceed with selects

            # How many bits of the UID do we know about?
            # When calling this method with just the uid, this value is always a multiple of 8
            # which makes this math a little strange. However, when we run an anticollision check
            # and we get a collision (multiple PICCs in the field) then we'll be provided a position from 0-32 where the
            # collision occured (the number of unique bits in the various PICC's uid) and this may mean we have to send bytes
            # and a few bits around
            known_bits = valid_bits - (MFRC522.BITS_IN_BYTE * uid_start_index)
            if known_bits < 0:
                known_bits = 0

            if use_cascade_tag:
                # If we're using the cascade tag we actually know about 8 more bits (the cascade tag)
                known_bits += MFRC522.BITS_IN_BYTE

            # This is needed to understand how to populate the BitFramingReg's lower 4 bits
            bits_to_transfer = 0

            # Determine how many bytes of the uid we need to add to the data list
            bytes_to_copy = int(known_bits / 8) + 1 if (known_bits % 8) else 0
            max_bytes = 3 if use_cascade_tag else 4
            if bytes_to_copy > max_bytes:
                bytes_to_copy = max_bytes

            select_finished = False
            while !select_finished:
                if known_bits >= 32:  # We've got all the bits we're going to get for this cascade level, time to select
                    data.append(cascade_level)
                    data.append(MFRC522.NVB_SEVEN_BYTES)  # command, NVB, 4 bytes of UID (or CT + 3 bytes) and the BCC
                    # We have at least 4 uid bytes at this point
                    # Calculate the Block Check Character (BCC)
                    bcc = uid[0] ^ uid[1] ^ uid[2] ^ uid[3]
                    first_uid_byte = 0
                    if use_cascade_tag:
                        data.append(MFRC522.CASCADE_TAG)
                        # Since the cascade tag is in uid[0] we can only copy over the remaining 3 bytes
                        first_uid_byte = 1
                    for i in range(start=first_uid_byte, stop=4):
                        data.append(uid[i])
                    data.append(bcc)

                    # Calculate CRC_A
                    status, crc_result = self.calculate_crc(data)
                    if status != MFRC522.ErrorCode.OK:
                        return (status, result)
                    for b in result:
                        # Add the CRC values to the buffer
                        data.append(b)
                    bits_to_transfer = 0  # We've validated that all the bits are so there are no stragglers
                else:  # This is anticollision
                    data.append(cascade_level)
                    bits_to_transfer = known_bits % 8
                    bytes_to_transfer = int(known_bits / 8)  # Make sure this is an int (drop the fraction)
                    total_bytes_to_send = 2 + bytes_to_transfer  # Need to add the bytes for command and NVB to the overall bytes
                    data.append((total_bytes_to_send << 4) + bits_to_transfer)  # The total number of bytes needs to be in the high 4 bits,
                                                                                # remaining bits are in low 4 bits
                    for i in range(start=uid_start_index, stop=uid_start_index + bytes_to_copy):
                        data.append(uid[i])

                # Reset the the bit-oriented frame settings
                # The high four bits (actuall only [4], [5], and [6] because the bits will never be more than 7)
                # indicate where the LSB is stored
                # [0], [1], and [2] is the number of bits in the last byte that need to be transferred
                # [3] is reserved and therefore should stay at 0 (zero)
                self.write(MFRC522.Register.BitFramingReg, (bits_to_transfer << 4) + bits_to_transfer)

                status, results, results_len = self.transceive(data)


                if status != MFRC522.ErrorCode.OK:
                    # An error occured
                    return (status, results)
                elif status == MFRC522.ErrorCode.COLLISION:
                    # There was more than PICC in the field!
                    collision_info = self.read(MFRC522.Register.CollReg)
                    if collision_info & MFRC522.BIT_MASK_COLLREG_POSITION_NOT_VALID:
                        # We don't have a valid collision position and can't continue
                        return (status, results)
                    collision_position = collision_info & MFRC522.BIT_MASK_COLLREG_POSITION
                    if (collision_position == 0):
                        collision_position = 32
                    if collision_position <= known_bits:
                        # Wait a second, we already know these bits are valid so
                        # something has gone terribly wrong
                        return (MFRC522.ErrorCode.UNKNOWN_COLLISION_ERROR, result)

                    # Determine how many new bits (or bytes) we now have of the uid and copy them out of the results
                    new_bits_found = collision_position - known_bits
                    new_bytes_found = int((new_bits_found + (known_bits % 8)) / 8)
                    if known_bits % 8 > 0:
                        # we had a partial byte, the results[0] is the full byte, copy it over
                        uid[len(uid) - 1] = results[0]
                        if new_bytes_found > 0:
                            # There are additional bytes in results[1-n], copy them over
                            for i in range(start=1, len(results)):
                                uid.append(results[i])
                    else:
                        # We had only full bytes so everything in results is new, copy it over
                        for r in results:
                            uid.appened(r)
                    # Reset how many bits we know about
                    known_bits = collision_position
                else:
                    if known_bits >= 32:
                        # we just performed a select and it's successful so we're done
                        select_finished = True
                    else:
                        # we just performed an anticollision without error so now we know all 4 bytes and this cascade level is done
                        known_bits = 32
                        # Run the select loop again
            # Select complete, let's review the SAK
            if (results_len != 3):
                # We don't have 1 byte of SAK and 2 bytes of CRC
                return (MFRC522.ErrorCode.ERR, results)

            # Check if the cascade bit is set
            if results[0] & 0b00000100:
                cascade_level++
            else
                finished = True

        return (MFRC522.ErrorCode.OK, uid)

    def calculate_crc(self, data):
        # Idle the card, cancelling any current commands
        self.write(MFRC522.Register.CommandReg, MFRC522.PCDCommand.IDLE)

        # Clear the CRC IRQ bit
        self.unset_bits(MFRC522.Register.DivIrqReg, MFRC522.BIT_MASK_DIVIRQ_CRCIRQ)
        # Flush the FIFO buffer (set FlushBuffer ([7]) to 1)
        self.set_bits(MFRC522.Register.FIFOLevelReg, MFRC522.BIT_MASK_MSB)
        self._write_data_to_fifo(data)

        # Calculate the CRC
        self.write(MFRC522.Register.CommandReg, MFRC522.PCDCommand.CALCCRC)

        countdown = MFRC522.CRC_CHECKS
        while countdown > 0:
            countdown -= 1
            interrupts = self.read(MFRC522.Register.DivIrqReg)
            if interrupts & MFRC522.BIT_MASK_DIVIRQ_CRCIRQ:
                # Stop calculation, we're done
                self.write(MFRC522.Register.CommandReg, MFRC522.PCDCommand.IDLE)
                rv = []
                rv.append(self.read(MFRC522.Register.CRCResultRegL))
                rv.append(self.read(MFRC522.Register.CRCResultRegH))
                return (MFRC522.ErrorCode.OK, rv)
        # Timeout happened
        return (MFRC.ErrorCode.COUNTDOWN_TIMEOUT, [])

    def _clear_bits_after_collision(self):
        # Set ValuesAfterColl to zero to ensure all bits are cleared after a collision
        self.unset_bits(MFRC522.Register.CollReg, MFRC522.BIT_MASK_MSB)

    def _write_data_to_fifo(self, data):
        # Write the data to the FIFO Data register
        for datum in data:
            self.write(MFRC522.Register.FIFODataReg, datum)
