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
    class ReturnCode(IntEnum):
        """ The actual values don't matter. """
        OK = 0  # everything is OK
        NOTAGERR = 1  # There was no tag to read
        ERR = 2  # Something went wrong (e.g. parity error, buffer oveflow, protocol error)
        TIMEOUT = 3  # The timer completed before we were done
        COUNTDOWN_TIMEOUT = 4  # Our countdown completed without the timer going off
        COLLISION = 5  # A collision error happened
        UNKNOWN_COLLISION_ERROR = 6  # A collision happened but the position is within the data we already know which shouldn't be possible
        INVALID_COLLISION_POSITION = 7  # A collision happened by the result did not provide a valid collision position
        INVALID_SAK_RESULT = 8  # We successfully completed a SEL but the SAK wasn't the right size/content
        SAK_CRC_WRONG = 9  # We got a SAK but the CRC value didn't actually check out



    @unique
    class PICCCommand(IntEnum):
        """ proximity inductive coupling card (PICC) commands """
        REQA = 0x26  # Is there a card in the field?
        ANTICOLL_CS1 = 0x93  # Anti-collision cascade level 1
        ANTICOLL_CS2 = 0x95  # Anti-collision cascade level 2
        ANTICOLL_CS3 = 0x97  # Anti-collision cascade level 3

    MAX_SPEED_HZ = 106000  # 106 kBd - see initialize_card()
    FIFO_BUFFER_MAX_SIZE = 64  # The size (in bytes) of the FIFO buffer
    BITS_IN_BYTE = 8  # The number of bits in a byte
    BIT_MASK_ANTENNA_POWER = 0b00000011  # A bit mask for [0] and [1] (Tx1RFEn, Tx2RFEn) which indicate the current power state of the antenna
    BIT_MASK_CASCADE_BIT_SET = 0b000000100  # Bit mask to check the SAK for the cascade bit
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
        if status == MFRC522.ReturnCode.OK and results_len == 16:
            return True
        return False

    def read_uid(self, timeout=None):
        """
        Checks for the presence of a type A PICC and, if found, executes anticollision and returns the uid
        If type A PICC not found or anticollision fails for any reason, returns None

        timeout - The number of seconds to wait for a read, defaults to None which means wait infinitely
        """
        if timeout and timeout > -1:
            timeout = time.time() + timeout

        uid = None
        while not uid:
            if self.card_present():
                status, results = self.anticollision()

                if status == MFRC522.ReturnCode.OK:
                    return self._uid_bytes_to_hex_string(results)
                else:
                    time.sleep(0.001)  # Wait a very short time to avoid hogging the CPU
                    if time.time() >= timeout:
                        return None

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
        self.write(MFRC522.Register.BitFramingReg, MFRC522.BIT_FRAMING_SHORT_FRAME_FORMAT)
        return self.transceive([MFRC522.PICCCommand.REQA])

    def transceive(self, data):
        # Idle the card, cancelling any current commands
        self.write(MFRC522.Register.CommandReg, MFRC522.PCDCommand.IDLE)

        # Set the Set1 bit ([7]) to 0 which clears the IRQ register
        self.unset_bits(MFRC522.Register.ComIrqReg, MFRC522.BIT_MASK_MSB)

        # Flush the FIFO buffer (set FlushBuffer ([7]) to 1)
        self.set_bits(MFRC522.Register.FIFOLevelReg, MFRC522.BIT_MASK_MSB)

        self._write_data_to_fifo(data)

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
                return (MFRC522.ReturnCode.TIMEOUT, results, results_len)

        if countdown == 0:
            # None of the interrupts fired before the countdown finished
            # Did we lose connectivity with the MFRC522?
            return (MFRC522.ReturnCode.ERR.COUNTDOWN_TIMEOUT, results, results_len)

        errors = self.read(MFRC522.Register.ErrorReg)
        if  errors & MFRC522.BIT_MASK_TRANCEIVE_ERRORS:
            # We had an error
            return (MFRC522.ReturnCode.ERR, results, results_len)

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
            return (MFRC522.ReturnCode.COLLISION, results, results_len)

        # Normally, you'd want to check if a CRC was requested
        # However, I don't need to actually read MiFare data (which is when this is requested)
        # So I didn't implement that check

        # If we reached this point, all the checks have passed, we can return OK
        return (MFRC522.ReturnCode.OK, results, results_len)

    def anticollision(self):
        """
        This method is called anticollision but it actually implements the entire select/anticollision process.
        There are three cascade levels to work through (1, 2, 3) (SEL) depending on the size of the uid (4-, 7-, or 10-bytes).
        The process is started with no information about the PICCs and at cascade level 1. An initial request is made and, if
        only a single PICC responds, we can move directly onto a select operation. If this succeeds, we check the select acknowledge (SAK)
        to see if the PICC indicated there were more bytes in the uid. If there are, we move to the next cascade level and start the process
        again. If not, we have the whole uid and it is returned.

        For reference, here are a couple of terms/acronyms:
          - SEL -> Select, the actual value of this command changes based on the current level (1, 2, 3)
          - SAK -> Select acknowledgement, what is returned by the PICC for a full select
          - NVB -> Number of valid bits, the total number of bytes (high 4 bits) and bits (low 4 bits) we will are sending, this is for the complete
                   command and not just the UID
        """
        self._clear_bits_after_collision()

        # Setup the initial values:
        # How many bits within the UID have we verified so far
        known_bits = 0
        # Which cascade level to start with
        cascade_level = MFRC522.PICCCommand.ANTICOLL_CS1
        # A list to hold the uid
        uid = [0] * 10
        # A list to hold the data we need to transceive
        buffer = [0] * 9
        # How many extra bits do we need to transceive?
        transceive_bits = 0
        # How many slots in the buffer do we need to transceive (this will control the size of the slice later)
        # Default to 2 as the first command is always just SEL + NVB
        transceive_buffer_size = 2

        # Perform the steps below until the SAK indicates we have the complete UID
        uid_complete = False
        while not uid_complete:
            # Determine some additional options based on the current cascade level
            if cascade_level == MFRC522.PICCCommand.ANTICOLL_CS1:
                uid_start_index = 0  # We know nothing yet

            if cascade_level == MFRC522.PICCCommand.ANTICOLL_CS2:
                uid_start_index = 3  # We know about 4 bytes

            if cascade_level == MFRC522.PICCCommand.ANTICOLL_CS3:
                uid_start_index = 6  # We know about 7 bytes

            # Set the command we'll be using
            buffer[0] = cascade_level

            # Calculate the known_bits based on the cascade level
            known_bits = uid_start_index * MFRC522.BITS_IN_BYTE

            # Determine which index in the buffer we need to start copying the uid information into
            # Default to 2 ([0] = SEL, [1] = NVB)
            buffer_index = 2

            # Copy the bits that we know about out of the uid list into the buffer so we can send them along with our commands
            # We start by determining the total number of bytes (known_bits / 8) (NOTE, make sure this is inside an int() call to avoid floating point results),
            # then add 1 additionl bit if known_bits is not evenly divisible by 8 (known_bits % 8)
            bytes_to_copy = (int(known_bits / 8)) + (1 if known_bits % 8 else 0)
            if bytes_to_copy:
                # We only have room for 4 bytes
                bytes_to_copy = min(4, bytes_to_copy)

                # Copy the uid bytes (starting and the uid_start_index) into the buffer in the correct location
                for i in range(bytes_to_copy):
                    buffer[buffer_index] = uid[(uid_start_index % 3)+ i]
                    buffer_index += 1

            print('before SEL/ANTICOLL', 'uid_start_index', uid_start_index, 'buffer_index', buffer_index, 'known_bits', known_bits, 'buffer', buffer)
            # Start the SEL/ANTICOLL loop
            select_finished = False
            while not select_finished:
                print('top of SEL/ANTICOLL loop', 'cascade_level', cascade_level)
                if known_bits >= 32:  # We've got all the bits we're going to get for this cascade level, time to select
                    print('we have more than 32 bits, starting select...')
                    buffer[1] = MFRC522.NVB_SEVEN_BYTES  # SEL, NVB, 4 bytes of UID (or CT + 3 bytes) and the BCC
                    # We have at least 4 uid bytes at this point
                    # Calculate the Block Check Character (BCC) (buffer[6])
                    buffer[6] = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5]

                    # Calculate a CRC_A value the first 7 bytes of the buffer
                    status, crc_results = self.calculate_crc(buffer[:7])
                    if status != MFRC522.ReturnCode.OK:
                        return (status, crc_results)

                    # Add CRC to the buffer
                    buffer[7] = crc_results[0]
                    buffer[8] = crc_results[1]

                    # We have all the bytes so no extraneous bits to transceive
                    transceive_bits = 0
                    # We're sending a full select so we'll use all 9 bytes
                    transceive_buffer_size = 9
                else:  # This is anticollision
                    print('starting anticollision...')
                    transceive_bytes = int(known_bits / 8)
                    transceive_bits = known_bits % 8
                    # Calculate the total number of whole bytes we're going to send, we always send SEL and NVB and we're only sending the UID bytes
                    nvb_byte_count = 2 + transceive_bytes
                    # Set the NVB - high 4 is equal to the total bytes, low 4 is equal to the remaining bits
                    buffer[1] = (nvb_byte_count << 4) + transceive_bits
                    transceive_buffer_size = nvb_byte_count + (1 if transceive_bits else 0)

                # Reset the the bit-oriented frame settings
                # This is made up of rxAlign ([4], [5], [6]) and indicates where the LSB is and
                # TxLastBits ([0], [1], [2]) which stores how many bits in the last byte will be transmitted
                # [3] is reserved and will be left at zero
                # [7] is the start send indicator and will be left at zero since we're not yet ready to start
                self.write(MFRC522.Register.BitFramingReg, (transceive_bits << 4) + transceive_bits)

                # Transceive only the part of the buffer indicated by transceive_buffer_size!
                print('about to transceive', 'buffer', buffer, 'transceive_buffer_size', transceive_buffer_size, 'buffer to send', buffer[:transceive_buffer_size])
                status, results, results_len = self.transceive(buffer[:transceive_buffer_size])
                print('transceive results', 'status', status, 'results', results, 'results_len', results_len)

                if status == MFRC522.ReturnCode.COLLISION:
                    print('collision!', 'status', status, 'results', results, 'results_len', results_len)
                    # There was more than PICC in the field! Read the CollReg to get more info
                    collision_info = self.read(MFRC522.Register.CollReg)
                    if collision_info & MFRC522.BIT_MASK_COLLREG_POSITION_NOT_VALID:
                        # We don't have a valid collision position and can't continue
                        return (MFRC522.ReturnCode.INVALID_COLLISION_POSITION, results)
                    collision_position = collision_info & MFRC522.BIT_MASK_COLLREG_POSITION
                    if (collision_position == 0):
                        collision_position = 32
                    if collision_position <= known_bits:
                        # Wait a second, we already know these bits are valid so
                        # something has gone terribly wrong
                        return (MFRC522.ReturnCode.UNKNOWN_COLLISION_ERROR, result)

                    # Determine how many new bytes and bits we've just learned about
                    new_bytes = int((collision_position - known_bits) / 8)
                    new_bits = (collision_position - known_bits) % 8
                    # Copy the new information we just got from the results into the buffer for use next time through
                    # buffer_index is left +1 position from the last byte of the uid copied in
                    # if we had bits in the last byte, we'll need to go back one (because results now has more bits)
                    # otherwise start copying there
                    copy_index = buffer_index
                    if known_bits % 8:
                        copy_index = buffer_index - 1
                    for i in range(new_bytes):
                        buffer[copy_index + i] = results[i]

                    # Based on the results of collision, we now have some additional known bits
                    known_bits = collision_position
                    # The protocol indicates we need to flip the bit prior to the collision position and try again
                    # Need to first know the specific bit that caused the collision
                    collision_bit = known_bits % 8
                    # Need to know the specific bit location within the last byte to flip
                    # Also need to account for any byte boundaries so we can't just subtract 1 from the collision bit
                    bit_to_flip = (known_bits -1) % 8
                    # Determine which index in the buffer contains the bit that needs to be flipped
                    # Start with index 1 ([0] = SEL, [1] = NVB), add the number of whole bytes and then add one if there are still some bits
                    bit_to_flip_index = 1 + (int(known_bits / 8)) + (1 if collision_bit else 0)
                    # Flip the bit by bitwise OR'ing with 1 shifted to the correct bit position
                    buffer[bit_to_flip_index] |= (1 << bit_to_flip)
                elif status != MFRC522.ReturnCode.OK:
                    print('An error has occured', 'status', status, 'results', results)
                    # An error occured
                    return (status, results)
                else:
                    # We were successful, but sucessful at what?
                    if known_bits >= 32:
                        print('selected success!')
                        # we just performed a select and it's successful so we're done
                        select_finished = True
                    else:
                        print('anticollision success!', 'results', results)
                        # we just performed an anticollision without error so results contains all 32 bits of the UID for this level
                        # Copy them over to the buffer for the select call
                        # We can blindly copy starting at index 2 ([0] = SEL, [1] = NVB) as the cascade tag isn't a factor
                        # the response is 40 bits, the first 32 are the uid bytes, the last 8 are the BCC value which we can discard
                        for i in range(len(results) - 1):
                            buffer[2 + i] = results[i]
                        known_bits = 32
                        print('anticollision success copy complete', 'buffer', buffer)
                        # Run the select loop again

            # We've completed the select for this cascade level, copy over the known uid bytes
            # Need to adjust based on whether we're received a cascade tag or not
            print('buffer[2] in hex:', hex(buffer[2]), 'MFRC522.CASCADE_TAG in hex:', hex(MFRC522.CASCADE_TAG))
            if buffer[2] == MFRC522.CASCADE_TAG:
                buffer_index_with_uid = 3
                bytes_to_copy = 3
            else:
                buffer_index_with_uid = 2
                bytes_to_copy = 4            # Loop with zip?
            for i in range(bytes_to_copy):
                uid[uid_start_index + i] = buffer[buffer_index_with_uid]
                buffer_index_with_uid += 1
            print('uid so far', uid)

            # Select complete, let's review the SAK
            if (len(results) != 3):
                # We don't have 1 byte of SAK and 2 bytes of CRC
                return (MFRC522.ReturnCode.INVALID_SAK_RESULT, results)

            print('About to recalculate the CRC_A', 'results', results)
            # Let's double check that CRC_A we got back by calculating our own
            status, crc_result = self.calculate_crc(results[:1])
            if status != MFRC522.ReturnCode.OK:
                return (status, crc_result)
            if (results[1] != crc_result[0]) or (results[2] != crc_result[1]):
                return (MFRC522.ReturnCode.SAK_CRC_WRONG, results)

            # Do we have the whole UID or not?
            if results[0] & MFRC522.BIT_MASK_CASCADE_BIT_SET:
                # Nope, there's still more
                print('Cascade bit set, moving to next level...')
                cascade_level = self._next_cascade_level(cascade_level)
            else:
                uid_complete = True

        return (MFRC522.ReturnCode.OK, uid[:self._uid_size(cascade_level)])

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
                return (MFRC522.ReturnCode.OK, rv)
        # Timeout happened
        return (MFRC522.ReturnCode.COUNTDOWN_TIMEOUT, [])

    def _clear_bits_after_collision(self):
        # Set ValuesAfterColl to zero to ensure all bits are cleared after a collision
        self.unset_bits(MFRC522.Register.CollReg, MFRC522.BIT_MASK_MSB)

    def _write_data_to_fifo(self, data):
        # Write the data to the FIFO Data register
        for datum in data:
            self.write(MFRC522.Register.FIFODataReg, datum)

    def _next_cascade_level(self, cascade_level):
        if cascade_level == MFRC522.PICCCommand.ANTICOLL_CS1:
            return MFRC522.PICCCommand.ANTICOLL_CS2
        if cascade_level == MFRC522.PICCCommand.ANTICOLL_CS2:
            return MFRC522.PICCCommand.ANTICOLL_CS3

    def _uid_size(self, cascade_level):
        if cascade_level == MFRC522.PICCCommand.ANTICOLL_CS1:
            # single
            return 4
        if cascade_level == MFRC522.PICCCommand.ANTICOLL_CS2:
            # double
            return 7
        if cascade_level == MFRC522.PICCCommand.ANTICOLL_CS3:
            # triple
            return 10

    def _uid_bytes_to_hex_string(uid):
        uid = ''
        for digit in uid:
            uid += f'{digit:0>2X}'
        return uid
