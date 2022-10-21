

"""
Code to manage a single MWA BFIF board, and the beamformer connected to it.

Written by Andrew Williams (Andrew.Williams@curtin.edu.au)

"""

import logging
import threading
import time

# noinspection PyUnresolvedReferences
import RPi.GPIO as GPIO
# noinspection PyUnresolvedReferences
import smbus

logging.basicConfig()

# IO pin allocations on the Raspberry Pi GPIO header
DOCPOWER = 18  # DoCPwrEnable - 5V and 48V DoC power Enable - 'high' to turn ON power.
BFPOWER = 19  # BFPowerEnable - Beamformer Power Enable - 'high' to turn ON power. Only have this on when DOCPOWER is on.

TXDATA = 13  # DoCTxData - DoC Data Output
TXCLOCK = 12  # DoCTxClock - DoC Transmit Clock Output
RXDATA = 15  # DoCRxData - DoC Data Input

OPMODE = 16  # OperationMode - General purpose hardware link select.
RFOFOFF = 21  # RFoFPowerOff - 'high' to turn OFF power to both Astron RFoF modules (if present)
AUXOFF = 22  # VauxPowerOff - 'high' to turn OFF power supply running media converter (if present)
SERIALMODE = 23  # RS485/RS232_L - 'high' for RS485, 'low' for RS232
SERENABLE = 24  # SerialCommsOn - 'high' to ENABLE power to the serial comms IC

# I2C device addresses for the i2c sensors on the BFIF board. Note that these are _seven_ bit addresses,
# so 0x68 in seven bits corresponds to D0 when the r/w bit is appended as bit 0 of the address, and
# 0x48 corresponds to 0x90.
ADDRESS7_LTC4151 = 0x68
ADDRESS7_DS75 = 0x48


##################################################################################
#
# Functions to set up the GPIO ports on the Raspberry Pi, and clean them up
# on shutdown.
#
##################################################################################

def setup_gpio():
    """
        Setup GPIO pin configuration as required.

        :return: None
    """
    GPIO.setmode(GPIO.BOARD)  # Use board connector pin numbers to specify I/O pins
    GPIO.setwarnings(False)
    GPIO.setup(DOCPOWER, GPIO.OUT)
    GPIO.setup(BFPOWER, GPIO.OUT)
    GPIO.setup(TXDATA, GPIO.OUT)
    GPIO.setup(TXCLOCK, GPIO.OUT)
    GPIO.setup(RXDATA, GPIO.IN)
    GPIO.setup(OPMODE, GPIO.IN)
    GPIO.setup(RFOFOFF, GPIO.OUT)
    GPIO.setup(AUXOFF, GPIO.OUT)
    GPIO.setup(SERIALMODE, GPIO.OUT)
    GPIO.setup(SERENABLE, GPIO.OUT)


def cleanup_gpio():
    """
        Release all GPIO pins, ready for program exit.

        Note - BEFORE calling this cleanup_gpio() function, any code using the BFIFHandler
        class should call the BFIFHandler.cleanup() function, to safely turn disable and
        turn off the RxDoC card, and power to the tile.

        :return: None
    """
    GPIO.cleanup()

##################################################################################
#
# Class to handle the BFIF board (power, DoC card, RFoF, etc)
#
##################################################################################


class BFIFHandler(object):
    """
      Manages everything on the BFIF card, using GPIO pins from the Raspberry Pi header and
      the Raspberry Pi's I2C bus. This includes switching 48V to the Receiver DoC card (RxDoC),
      switching 48V to the beamformer, and switching power to the RF-on-Fibre (RFoF) modules (if present).
    """

    def __init__(self, logger=logging):
        self.logger = logger
        self.lock = threading.RLock()  # Used to control access to the hardware resources (i2C, GPIO) on the BFIF board
        with self.lock:
            self.logger.debug('BFIFHandler - Initialising BFIFHandler()')
            self.bus = smbus.SMBus(1)  # Initialise the i2c bus on the Raspberry Pi.
            self.current = 0.0
            self.voltage = 0.0
            self.temp = 0.0
            self.serialmode = ''  # Either 'RS232' or 'RS485'

            # Booleans for hardware state. True for 'ON', False for 'OFF', None if there was an error on the last state change:
            self.docpower = bool(GPIO.input(DOCPOWER))  # True if the RxDoC card is powered up.
            self.bfpower = bool(GPIO.input(BFPOWER))  # True if the enable bit is set on the RxDoC, powering up the beamformer.
            self.opmode = bool(GPIO.input(OPMODE))  # Input, hardware link status, defaults to False. Move jumper to 'True' to shut down Pi cleanly.
            self.rfof = not bool(GPIO.input(RFOFOFF))  # True if the power is on to the RFoF modules.
            self.auxpower = not bool(GPIO.input(AUXOFF))  # True if the 9V auxillary power supply is turned on.
            self.serialpower = bool(GPIO.input(SERENABLE))  # True if the MAX232 serial chip is powered up.

            try:
                self.bus.write_i2c_block_data(ADDRESS7_DS75, 1, [96])  # Set 12 bit temperature resolution
            except:
                logger.warning('BFIFHandler - could not set 12-bit resolution temperature measurement mode')
            time.sleep(0.1)

            # Set up initial hardware state on boot:
            self.turnoff_bf()  # Leave the beamformer turned off.
            self.turnoff_doc()  # Leave the RxDoC turned off.
            self.turnoff_rfof()  # Leave the RFoF modules turned off.
            self.check()  # Read initial voltages, currents and temperatures.

    def get_status(self):
        """
          Return a dict containing the current status data.

          This dict can be parsed by whatever code calls this function (eg, a Nagios plugin) and
          isn't meant to be human-readable. For a human readable status message, call the __str__()
          method.

          :return: dictionary containing status data.
        """
        with self.lock:
            status = {'current':self.current,
                      'voltage':self.voltage,
                      'temp':self.temp,
                      'docpower':self.docpower,
                      'bfpower':self.bfpower,
                      'opmode':self.opmode,
                      'rfof':self.rfof,
                      'auxpower':self.auxpower,
                      'serialpower':self.serialpower,
                      'serialmode':self.serialmode}
        return status

    def turnon_bf(self):
        """
          Turn on the output power FET on the RxDoC card. This must be called _after_ powering up the DoC card.

          Return True if there were no errors, False if there was an error, None if the DoC card was off when called.

         :return: boolean, True for success, False for failure
        """
        if not self.docpower:
            self.logger.error("BFIFHandler - must turn on power to the doc before enabling it")
            return None
        with self.lock:
            self.logger.debug('BFIFHandler - Turning ON 48V to beamformer')
            GPIO.output(BFPOWER, 1)
            self.bfpower = bool(GPIO.input(BFPOWER))
        return self.bfpower is True

    def turnoff_bf(self):
        """
          Turn off the output power FET on the RxDoC card.

          Return True if there were no errors, False if there was an error.

          :return: boolean, True for success, False for failure
        """
        with self.lock:
            self.logger.debug('BFIFHandler - Turning OFF 48V to beamformer')
            GPIO.output(BFPOWER, 0)
            self.bfpower = bool(GPIO.input(BFPOWER))
        return self.bfpower is False

    def turnon_doc(self):
        """
          Switch the power on to the RxDoC card, using the FET on the BFIF board. If the beamformer is enabled
          when this method is called, it will be automatically disabled before power is turned on.

          Return True if there were no errors, False if there was an error.

          :return: boolean, True for success, False for failure
        """
        with self.lock:
            if self.bfpower:
                self.turnoff_bf()  # If the DoC card is enabled, disable it BEFORE turning the power on.
                time.sleep(0.1)
            self.logger.debug('BFIFHandler - Turning ON 48V to DoC card')
            GPIO.output(DOCPOWER, 1)
            self.docpower = bool(GPIO.input(DOCPOWER))
        return self.docpower is True

    def turnoff_doc(self):
        """
          Switch the power off to the RxDoC card, using the FET on the BFIF board. If the beamformer is enabled
          when this method is called, it will be automatically disabled before power is turned off.

          Return True if there were no errors, False if there was an error.

          :return: boolean, True for success, False for failure
        """
        with self.lock:
            if self.bfpower:
                self.turnoff_bf()
                time.sleep(0.1)
            self.logger.debug('BFIFHandler - Turning OFF 48V to DoC card')
            GPIO.output(DOCPOWER, 0)
            self.docpower = bool(GPIO.input(DOCPOWER))
        return self.docpower is False

    def turnon_aux(self):
        """
          Turn on the Auxillary power supply, which provides power to the network media converter (if present).
          If this power supply is off, no network activity is possible unless directly connected to the Raspberry
          Pi's RJ45 port.

          Return True if there were no errors, False if there was an error.

          :return: boolean, True for success, False for failure
        """
        with self.lock:
            self.logger.debug('BFIFHandler - Turning ON power to fibre/copper media converter')
            GPIO.output(AUXOFF, 0)
            self.auxpower = not bool(GPIO.input(AUXOFF))
        return self.auxpower is True

    def turnoff_aux(self):
        """
          Turn off the Auxillary power suppply, which provides power to the network media converter (if present).
          If this power supply is off, no network activity is possible.

          Return True if there were no errors, False if there was an error.

          :return: boolean, True for success, False for failure
        """
        with self.lock:
            self.logger.debug('BFIFHandler - Turning OFF power to fibre/copper media converter')
            GPIO.output(AUXOFF, 1)
            self.auxpower = not bool(GPIO.input(AUXOFF))
        return self.auxpower is False

    def turnon_rfof(self):
        """
          Turn on power to the 'RF over Fibre' modules, if present.

          Return True if there were no errors, False if there was an error.

          :return: boolean, True for success, False for failure
        """
        with self.lock:
            self.logger.debug('BFIFHandler - Turning ON power to RFoF modules')
            GPIO.output(RFOFOFF, 0)
            self.rfof = not bool(GPIO.input(RFOFOFF))
        return self.rfof is True

    def turnoff_rfof(self):
        """
          Turn off power to the 'RF over Fibre' modules, if present.

          Return True if there were no errors, False if there was an error.

          :return: boolean, True for success, False for failure
        """
        with self.lock:
            self.logger.debug('BFIFHandler - Turning OFF power to RFoF modules')
            GPIO.output(RFOFOFF, 1)
            self.rfof = not bool(GPIO.input(RFOFOFF))
        return self.rfof is False

    def turnon_serial(self):
        """
          Turn on power to the serial comms chip that allows communication with external devices.
          If this power is turned off, no serial communication is possible.

          Return True if there were no errors, False if there was an error.

          :return: boolean, True for success, False for failure
        """
        with self.lock:
            self.logger.debug('BFIFHandler - Turning ON power to serial chip')
            GPIO.output(SERENABLE, 1)
            self.serialpower = bool(GPIO.input(SERENABLE))
        return self.serialpower is True

    def turnoff_serial(self):
        """
          Turn off power to the serial comms chip that allows communication with external devices.
          If this power is turned off, no serial communication is possible.

          Return True if there were no errors, False if there was an error.

          :return: boolean, True for success, False for failure
        """
        with self.lock:
            self.logger.debug('BFIFHandler - Turning OFF power to serial chip')
            GPIO.output(SERENABLE, 0)
            self.serialpower = bool(GPIO.input(SERENABLE))
        return self.serialpower is False

    def setrs232(self):
        """
          Configure the serial comms link to the SPIU as RS232.

          Return True if there were no errors, False if there was an error.

          :return: boolean, True for success, False for failure
        """
        with self.lock:
            self.logger.debug('BFIFHandler - Set serial mode to RS232')
            GPIO.output(SERIALMODE, 0)
            self.serialmode = 'RS232'
        return not bool(GPIO.input(SERIALMODE))

    def setrs485(self):
        """
          Configure the serial comms link to the SPIU as RS485

          Return True if there were no errors, False if there was an error.

          :return: boolean, True for success, False for failure
        """
        with self.lock:
            self.logger.debug('BFIFHandler - Set serial mode to RS485')
            GPIO.output(SERIALMODE, 1)
            self.serialmode = 'RS485'
        return bool(GPIO.input(SERIALMODE))

    def check(self):
        """
          Read local (BFIF) temperature, and voltage and current to the RxDoC card, from the I2C sensors on the,
          and update the local sensor values.

          Also read the state of the local 'OPMODE' jumper setting.

          Return True if there were no errors, False if there was an error.

          :return: boolean, True for success, False for failure
        """
        self.opmode = bool(GPIO.input(OPMODE))  # Input, hardware link status, defaults to False. Move jumper to 'True' to shut down Pi cleanly.
        ok = True
        with self.lock:
            # Read voltage and current from the LTC4151:
            try:
                data = self.bus.read_i2c_block_data(ADDRESS7_LTC4151, 0, 4)
                self.current = ((data[0] * 16) + (
                            data[1] / 16)) * 20e-6 / 0.02  # 20uV per ADU, through a 0.02 Ohm shunt
                self.voltage = ((data[2] * 16) + (data[3] / 16)) * 0.025  # 25mV per ADU
            except IOError:
                self.logger.error("BFIFHandler - Can't read LTC4151 sensor on BFIF board")
                ok = False
                self.current = -999.0
                self.voltage = -999.0

            # Read temperature from the DS75
            try:
                data = self.bus.read_i2c_block_data(0x48, 0, 2)
                self.temp = data[0] + data[1] / 256.0
            except IOError:
                self.logger.error("BFIFHandler - Can't read DS75 sensor on BFIF board")
                ok = False
                self.temp = -999.0
        return ok

    def testboard(self):
        """
          Low-level board test, used to do lab testing of a bare BFIF board before assembly.

          NEVER exits, runs tests continuously. Does not need a Beamformer connected.

        """
        while True:
            if not self.turnon_aux():  # Power to the fibre media converter for network access.
                self.logger.critical('AUX power on FAILED')
            else:
                self.logger.info('AUX power on PASSED')

            time.sleep(1)

            if not self.turnon_serial():  # Power to the serial comms to the BL233 chip for the SPIUHandler().
                self.logger.critical('Serial power on FAILED')
            else:
                self.logger.info('Serial power on PASSED')

            time.sleep(1)

            if not self.turnon_doc():  # Power to the RxDoC.
                self.logger.critical('RxDoC power on FAILED')
            else:
                self.logger.info('RxDoC power on PASSED')

            time.sleep(1)

            if not self.turnon_bf():  # Enable Beamformer signal to the RxDoc.
                self.logger.critical('RxDoc enable beamformer signal FAILED')
            else:
                self.logger.info('RxDoc enable beamformer signal PASSED')

            time.sleep(1)

            if not self.turnon_rfof():  # Power to the RFoF modules.
                self.logger.critical('RFoF power on FAILED')
            else:
                self.logger.info('RFoF power on PASSED')

            time.sleep(1)

            time.sleep(1)

            ok = self.check()
            if not ok:
                self.logger.critical('I2C test FAILED, bad comms to devices')
            else:
                self.logger.info('I2C test PASSED: voltage=%5.2f V, current=%4.0f mA, temp=%4.1f C' % (self.voltage, self.current * 1000, self.temp))

            self.logger.info('State of OPMODE jumper is %s' % self.opmode)  # Pin is tested by check() method.

            time.sleep(1)

            if not self.turnoff_rfof():  # Power to the RFoF modules.
                self.logger.critical('RFoF power off FAILED')
            else:
                self.logger.info('RFoF power off PASSED')

            time.sleep(1)

            if not self.turnoff_bf():  # Disable Beamformer signal to the RxDoc.
                self.logger.critical('RxDoc disable beamformer signal FAILED')
            else:
                self.logger.info('RxDoc disable beamformer signal PASSED')

            time.sleep(1)

            if not self.turnoff_doc():  # Power to the RxDoC.
                self.logger.critical('RxDoC power off FAILED')
            else:
                self.logger.info('RxDoC power off PASSED')

            time.sleep(1)

            if not self.turnoff_serial():  # Power to the serial comms to the BL233 chip for the SPIUHandler().
                self.logger.critical('Serial power off FAILED')
            else:
                self.logger.info('Serial power off PASSED')

            time.sleep(1)

            if not self.turnoff_aux():  # Power to the fibre media converter for network access.
                self.logger.critical('AUX power off FAILED')
            else:
                self.logger.info('AUX power off PASSED')

            time.sleep(1)

    def cleanup(self):
        """
            Safely disable the RxDoC card output, and turn off power to the tile, before exiting the program.

            :return: None
        """
        with self.lock:
            self.logger.info('BFIFHandler - turning off Beamformer, DoC')
            ok1 = self.turnoff_bf()
            if not ok1:
                self.logger.error('BFIFHandler - Error turning off beamformer')
            ok2 = self.turnoff_doc()
            if not ok2:
                self.logger.error('BFIFHandler - Error turning off DoC card')
            ok3 = self.turnoff_rfof()
            if not ok3:
                self.logger.error('BFIFHandler - Error turning off RFoF power')

    def __repr__(self):
        """
          Human readable status string, describing the complete state of the BFIF card.

          :return: string describing the object status.
        """
        self.check()
        BDICT = {False:'OFF', True:'ON', None:'ERROR!'}

        params = (BDICT[self.docpower],
                  BDICT[self.bfpower],
                  BDICT[self.rfof],
                  BDICT[self.auxpower],
                  BDICT[self.serialpower],
                  self.serialmode,
                  self.opmode)
        outs = "BFIFHandler: Flags: docpower=%3s, bfpower=%3s, RFoF=%3s, Aux=%3s, SerialPower=%3s, SerialMode=%s, OpMode=%s\n" % params

        params = (self.voltage,
                  self.current * 1000,
                  self.temp)
        outs += "BFIFHandler: Values: voltage=%5.2f V, current=%4.0f mA, temp=%4.1f C\n" % params

        return outs

    def __str__(self):
        """
          :return: string describing the object status.
        """
        return self.__repr__()


##################################################################################
#
# Class to handle pointing an MWA beamformer
#
##################################################################################

class BFHandler(object):
    """
      Manages the MWA beamformer, using GPIO pins from the Raspberry Pi header for TXDATA, TXCLOCK
      and RXDATA. Generates the 253-bit command sequences for different pointings, sends them to the
      beamformer, and receives temperature and flag data in response.

      Note that powering the beamformer up and down is handled by the BFIF class.
    """

    def __init__(self, logger=logging):
        self.logger = logger
        logger.debug('BFHandler - Initialising BFHandler()')
        self.lock = threading.RLock()
        self.temp = 0.0  # Beamformer temperature, read each time a pointing command is sent.
        self.flags = 999  # Flag value returned by the last pointing command. Should contain 128 if there were no comms errors.
        self.last_pointing = 0, None, None  # Contains a tuple of  (time.time(),xdelays,ydelays) for the last pointing command.
        self.standby_mode = False  # This will be True in Standby mode, False in normal mode.

    def get_status(self):
        """
          Return a dict containing the current status data.
          This dict can be parsed by whatever code calls this function (eg, a Nagios plugin) and
          isn't meant to be human-readable. For a human readable status message, call the __str__()
          method.

          :return: dictionary containing the current object status
        """
        status = {'temp':self.temp,
                  'flags':self.flags,
                  'last_pointing':self.last_pointing,
                  'standby_mode':self.standby_mode}
        return status

    def __repr__(self):
        """
          Human readable status string describing the complete state of the beamformer.

          :return human-readable string describing the current state
        """
        lastp, xdelays, ydelays = self.last_pointing
        return "BFHandler: Last pointing at '%s', temp=%4.1f, flags=%d to xdelays=%s\n" % (time.ctime(lastp),
                                                                                           self.temp,
                                                                                           self.flags,
                                                                                           xdelays)

    def __str__(self):
        return self.__repr__()

    def _gen_bitstring(self, xdelays, ydelays):
        """
          Given two arrays of 16 integers (each 0-63), representing the xdelays and ydelays, return
          a string containing 253 characters, each '1' or '0', representing the bit stream
          to be sent to the beamformer.

          Format is:
            8 zeroes
            4 ones
            20 zeroes
            6 blocks of 17 bits, each containing a 16-bit number containing the packed 6-bit
                x delay values, followed by a '1' bit
            6 blocks of 17 bits, each containing a 16-bit number containing the packed 6-bit
                y delay values, followed by a '1' bit
            16 bits of checksum (the twelve 16-bit packed delay words XORed together)
            A '1' bit to mark the end of the 13th 16-bit word

            These 253 bits are returned as a string with 253 '1' and '0' characters.

            After those bits are clocked out, a further 24 clock pulses should be sent, and
            24 bits of data (16 bits containing a 12-bit signed temperature, and 8 bits of
            flags) will be received.

            Returns None if there was an error creating the bitstring (due to invalid delay values),
            or a string containing 253 '1's and '0's to send to the beamformer.

            :param xdelays: List of 16 integer delays for individual dipoles in X, each 0-32
            :param ydelays: List of 16 integer delays for individual dipoles in Y, each 0-32
            :return: None if there was an error in the passed delays, or a string containing the bitstring
                     to send to the receiver.
        """
        if ((type(xdelays) != list) or (type(ydelays) != list) or
                (len(xdelays) != 16) or (len(ydelays) != 16)):
            self.logger.error('BFHandler - must pass a list of 16 integers in xdelays and ydelays')
            return None
        outlist = ['0' * 8, '1' * 4, '0' * 20]  # Header bits in packet, before delay values.
        dwords = []
        for val in (xdelays + ydelays):
            if (type(val) != int) or (val < 0) or (val > 63):
                self.logger.error('BFHandler - delay values must be integers.')
                return None  # Each delay value must be an integer, and must fit in 6 bits.
            else:
                dwords.append('{0:06b}'.format(val))
        dstring = ''.join(dwords)
        checksum = 0
        for i in range(0, 12 * 16, 16):
            checksum = checksum ^ int(dstring[i:i + 16], 2)
            outlist.append(dstring[i:i + 16] + '1')
        outlist.append('{0:016b}'.format(checksum))  # Append checksum bits to the end of the packet.
        return ''.join(outlist)  # 253 bits of output data, as ASCII '1' and '0' characters.

    def _send_bitstring(self, outstring, bittime=0.00002):
        """
          Given a string of 253 '1' or '0' characters, clock them out using the TXDATA and TXCLOCK
          pins, then clock in 24 bits of temp and flag data from the RXDATA pin.

          bittime is the total time to take to send one bit, in seconds.

          Returns a tuple of (temp, flags) where 'temp' is the beamformer temperature in deg C, and
          'flags' is the flag value (128 if there were no comms errors).

          :param outstring: String containing 253 '1's and '0's to send to the beamformer
          :param bittime: Total time to send one bit of data, in seconds.
          :return: Tuple of (temp, flags), where temp is a float containing beamformer temperature, and flags is an int
                   which should be equal to 128 if the communications were successful.
        """
        with self.lock:
            for bit in outstring:
                GPIO.output(TXDATA, {'1':1, '0':0}[bit])
                time.sleep(bittime / 4)  # wait for data bit to settle
                GPIO.output(TXCLOCK, 1)  # Send clock high
                time.sleep(bittime / 2)  # Leave clock high for half the total bit transmit time
                GPIO.output(TXCLOCK, 0)  # Send clock low,so data is valid on both rising and falling edge
                time.sleep(bittime / 4)  # Leave data valid until the end of the bit transmit time

            # While the temperature is 16 bits and the checksum is 8 bits, giving 24
            # bits in total, we appear to have to clock an extra bit-time to complete the
            # read-back operation. Once that's done, the checksum is the final (right-
            # most) 8 bits, and the temperature is 13 bits (signed plus 12-bits). Both
            # values are most-significant-bit first (chronologically).

            GPIO.output(TXDATA, 0)
            inbits = []
            for i in range(25):  # Read in temp data
                time.sleep(bittime / 4)
                GPIO.output(TXCLOCK, 1)
                time.sleep(bittime / 4)
                inbits.append({True:'1', False:'0'}[GPIO.input(RXDATA)])
                time.sleep(bittime / 4)
                GPIO.output(TXCLOCK, 0)
                time.sleep(bittime / 4)

        rawtemp = int(''.join(inbits[:17]), 2)  # Convert the first 16 bits to a temperature
        self.temp = 0.0625 * (rawtemp & 0xfff)  # 12 bits of value
        if (rawtemp & 0x1000):
            self.temp -= 256.0
        self.flags = int(''.join(inbits[17:]), 2)
        return self.temp, self.flags

    def point(self, xdelays=None, ydelays=None):
        """
          Send a new pointing to the beamformer using the given xdelay and ydelay values (each a list of
          16 integers between 0 and 63).

          Returns a tuple of (temp, flags) where 'temp' is the beamformer temperature in deg C, and
          'flags' is the flag value (128 if there were no comms errors).

          If the beamformer is in standby mode, it will not be pointed, and this function will return (-999, 999)

          :param xdelays: List of 16 integer delays for individual dipoles in X, each 0-32
          :param ydelays: List of 16 integer delays for individual dipoles in Y, each 0-32
          :return: Tuple of (temp, flags), where temp is a float containing beamformer temperature, and flags is an int
                   which should be equal to 128 if the communications were successful.
        """
        if not self.standby_mode:
            outstring = self._gen_bitstring(xdelays=xdelays, ydelays=ydelays)
            self.last_pointing = (time.time(), xdelays, ydelays)
            result = self._send_bitstring(outstring=outstring)
            temp, flags = result
            self.logger.debug('BFHandler - Beamformer sent new pointing. Flags=%d, temp=%4.1f' % (flags, temp))
            return result
        else:
            self.logger.warning('BFHandler - Beamformer in low-power mode, cannot send new pointing')
            return -999, 999

    def test_bf(self):
        """
          Pointing test mode - to be used in an RFI testing chamber. Runs forever, and point the beamformer every
          8 seconds.

          Does not return.
        """
        self.logger.info("Entering test mode")
        self.logger.info("Powered up RxDOC and beamformer")
        self.logger.info("Powered up fibre-copper media converter")
        time.sleep(3)

        while True:  # Loop forever in RFI test mode
            self.logger.info("sending new pointing...")
            self.point(xdelays=range(16), ydelays=range(16))

            self.logger.info("Status:")
            self.logger.info(self.__repr__())
            time.sleep(8)
