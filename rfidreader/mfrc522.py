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
            We use the write method first and then bitwise OR with 0x80 (1000 0000)
            to ensure that the MSB is 1.
            """
            return self.write() | 0x80

        def write(self):
            """
            Registers are represented by a 6-bit value where the LSB is always 0 (zero)
            and the MSB indicates the mode: 0 for write, 1 for read.
            To ensure that the LSB is always zero, we bit shift 1 place to the left.
            To ensure that the MSB is 0, we bitwise AND with 0x7E (0111 1110)
            Example:
              register = 0x03 = 0000 0011
              bit shift << 1 = 0000 0110
              AND with 0x7E

                0000 0110
              & 0111 1110
              -----------
                0000 0110

            If you now drop the MSB and LSB you're left with 000011 which is still 0x03
            """
            return (self << 1) & 0x7E


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

        # Setup the timer
        # The two time sections below will set the timer value to: ~0.015 seconds

        # 0x8D ==  1000 1101
        # 1101 - The high 4 bits of TPrescaler
        # 0    - TAutoRestart - timer automatically restarts its count-down from the 16-bit timer reload value instead of counting down to zero
        # 00   - TGated - non-gated mode
        # 1    - TAuto - timer starts automatically at the end of the transmission in all communication modes at all speeds
        # Setting the 4 high bits of TPrescaler to 0xD (13), leaving other values unset
        self.write(MFRC522.Register.TModeReg, 0x8D)

        # 0x3E == 0011 1110  - these are the lower 8 bits of the TPrescaler value
        # This sets the TPrescale value to 3390
        self.write(MFRC522.Register.TPrescalerReg, 0x3E)

        # Setup the timer reload value
        # Sets the TReload value to 30
        self.write(MFRC522.Register.TReloadRegL, 30)
        self.write(MFRC522.Register.TReloadRegH, 0)

        # Setup transmission modulation
        # 0x40 == 0100 0000
        # 00 0000  - Reserved
        # 1        - Force100ASK forces a 100 % ASK modulation independent of the ModGsPReg register setting
        # 0        - Reserved
        self.write(MFRC522.Register.TxASKReg, 0x40)

        # Setup general transmitting and receiving
        # 0x3D == 0011 1101
        # 01  - CRCPreset - defines the preset value for the CRC coprocessor for the CalcCRC command - 0x6363
        # 1   - Reserved (why not 0?)
        # 1   - PolMFin - defines the polarity of pin MFIN - polarity of pin MFIN is active HIGH
        # 1   - reserved (why not 0?)
        # 1   - TxWaitRF - transmitter can only be started if an RF field is generated
        # 0   - reserved
        # 0   - MSBFirst - CRC coprocessor calculates the CRC with MSB first (False)
        self.write(MFRC522.Register.ModeReg, 0x3D)

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
        # The TxControlReg contains a single byte with the following meaning:
        # [0] = Tx1RFEn = output signal on pin TX1 delivers the 13.56 MHz energy carrier modulated by the transmission data
        # [1] = Tx2RFEn = output signal on pin TX2 delivers the 13.56 MHz energy carrier modulated by the transmission data
        # [2] = reserved
        # [3] = Tx2CW = 1 = output signal on pin TX2 continuously delivers the unmodulated 13.56 MHz energy carrier,
        #               0 = Tx2CW bit is enabled to modulate the 13.56 MHz energy carrie
        # [4] = InvTx1RFOff = output signal on pin TX1 inverted when driver TX1 is disabled
        # [5] = InvTx2RFOff = output signal on pin TX2 inverted when driver TX2 is disabled
        # [6] = InvTx1RFOn = output signal on pin TX1 inverted when driver TX1 is enabled
        # [7] = output signal on pin TX2 inverted when driver TX2 is enabled
        current = self.read(MFRC522.Register.TxControlReg)

        # Determine if the antenna is currently off, if it is, turn it back on
        # Bitwise AND with 0x03 (0000 0011) will ignore all but first two bits (Tx1RFEn, Tx2RFEn)
        # If either of the first two bits are 0 (zero), then the antenna is off
        # By bitwise NOT'ing the value, we can check if the result is zero or not, if not zero, then it wasn't on
        if (~(current & 0x03)):
            # We want to change the value of the first two bits to 1 (one)
            # without changing any of the other values, use 0x03 (0000 0011)
            # as the mask to do that.
            self.set_bits(MFRC522.Register.TxControlReg, 0x03)

    def set_bits(self, register, mask):
        """ Reads the current value and then sets the specified bits to 1 """
        current = self.read(register)
        self.write(register, current | mask)

    def unset_bits(self, register, mask):
        """ Reads the current value and then sets the specified bits to 0 """
        current = self.read(register)
        self.write(register, current & (~mask))

    def card_present(self):
        # BitFramingReg - Adjustments for bit-oriented frames
        # [0][1][2] - TxLastBits - used for transmission of bit oriented frames:
        #                          defines the number of bits of the last byte that will be transmitted
        # [3]       - Reserved
        # [4][5][6] - RxAlign - used for reception of bit-oriented frames:
        #                       defines the bit position for the first bit received to be stored in the FIFO buffer
        # [7]       - StartSend - starts the transmission of data
        #                         only valid in combination with the Transceive command
        # A value of 0x07 (0000 0111) sets the TxLastBits to 7 indicating that only 7 bits of the last byte will be sent
        self.write(MFRC522.Register.BitFramingReg, 0x07)

        (status, results, results_len) = self.transceive([MFRC522.PICCCommand.REQIDL])

        # TODO covert status to boolean, determine if we need the results or len for anything else
        if ((status != MFRC522.ErrorCode.OK) | (results_len != 16)):
            status = MFRC522.ErrorCode.ERR

        return (status, results, results_len)

    def transceive(self, data):
        # Setup interrupt requests
        # [0] - TimerIEn - allows the timer interrupt request (TimerIRq bit) to be propagated to pin IRQ
        # [1] - ErrIEn - allows the error interrupt request (ErrIRq bit) to be propagated to pin IRQ
        # [2] - LoAlertIEn - allows the low alert interrupt request (LoAlertIRq bit) to be propagated to pin IRQ
        # [3] - HiAlertIEn - allows the high alert interrupt request (HiAlertIRq bit) to be propagated to pin IRQ
        # [4] - IdleIEn - allows the idle interrupt request (IdleIRq bit) to be propagated to pin IRQ
        # [5] - RxIEn - allows the receiver interrupt request (RxIRq bit) to be propagated to pin IRQ
        # [6] - TxIEn - allows the transmitter interrupt request (TxIRq bit) to be propagated to pin IRQ
        # [7] - IRqInv - 1: signal on pin IRQ is inverted with respect to the Status1Reg register’s IRq bit
        #                0: signal on pin IRQ is equal to the IRq bit;
        #                In combination with the DivIEnReg register’s IRqPushPull bit,
        #                the default value of logic 1 ensures that the output level on pin IRQ is 3-state
        # We're going to write 1111 0111
        # This will turn on all the signals except for HiAlertIEn
        irq_bitmask = 0x77
        self.write(MFRC522.Register.ComIEnReg, irq_bitmask | 0x80)

        # Set the Set1 bit ([7]) to 0 which clears the register
        self.unset_bits(MFRC522.Register.ComIrqReg, 0x80)

        # Flush the FIFO buffer (set FlushBuffer ([7]) to 1)
        self.set_bits(MFRC522.Register.FIFOLevelReg, 0x80)

        # Idle the card, cancelling any current commands
        self.write(MFRC522.Register.CommandReg, MFRC522.PCDCommand.IDLE)

        # Write the data to the FIFO Data register
        for i in range(len(data)):
            self.write(MFRC522.Register.FIFODataReg, data[i])

        # Run the transceive command
        self.write(MFRC522.Register.CommandReg, MFRC522.PCDCommand.TRANSCEIVE)

        # BitFramingReg - Adjustments for bit-oriented frames
        # [0][1][2] - TxLastBits - used for transmission of bit oriented frames:
        #                          defines the number of bits of the last byte that will be transmitted
        # [3]       - Reserved
        # [4][5][6] - RxAlign - used for reception of bit-oriented frames:
        #                       defines the bit position for the first bit received to be stored in the FIFO buffer
        # [7]       - StartSend - starts the transmission of data
        #                         only valid in combination with the Transceive command
        # A value of 0x80 (1000 0000) sets the StartSend bit to 1 to start the transmission of data
        self.set_bits(MFRC522.Register.BitFramingReg, 0x80)

        # wait for input for 30 seconds (2000 * .015 seconds)
        countdown = 2000
        tx_rx_bitmask = 0x30

        while True:
            interrupts = self.read(MFRC522.Register.ComIrqReg)
            countdown -= 1
            #print(countdown != 0, interrupts & 0x01, interrupts & tx_rx_bitmask)
            #print(~((countdown != 0) and (interrupts & 0x01) and (interrupts & tx_rx_bitmask)))
            if ~((countdown != 0) and ~(interrupts & 0x01) and ~(interrupts & tx_rx_bitmask)):
                break

        # Stop sending
        self.unset_bits(MFRC522.Register.BitFramingReg, 0x80)

        status = MFRC522.ErrorCode.ERR
        results = []
        results_len = 0
        # Check if we timed out (countdown == 0)
        if countdown != 0:
            # ErrorReg:
            # [0] - ProtocolErr
            # [1] - ParityErr
            # [2] - CRCErr
            # [3] - CollErr
            # [4] - BufferOvfl
            # [5] - reserved
            # [6] - TempErr
            # [7] - WrErr
            # Check to see if there are any protocol, partity, collision or buffer overflow errors
            if (self.read(MFRC522.Register.ErrorReg) & 0x1B) == 0x00:
                status = MFRC522.ErrorCode.OK

                # Check to see if the timer counted down
                if interrupts & irq_bitmask & 0x01:
                    status = MFRC522.ErrorCode.NOTAGERR

                # Check how many bytes were written to the FIFO
                bytes_written = self.read(MFRC522.Register.FIFOLevelReg)
                # Check how many valid bits in the last byte (0 (zero) indicates all were valid)
                bits_in_last_byte = self.read(MFRC522.Register.ControlReg) & 0x07
                if bits_in_last_byte != 0:
                    results_len = (bytes_written - 1) * 8 + bits_in_last_byte
                else:
                    results_len = bytes_written * 8

                if bytes_written == 0:
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
        # Make sure we aren't sending data
        self.write(MFRC522.Register.BitFramingReg, 0x00)

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
