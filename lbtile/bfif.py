

"""
Code to manage a single MWA BFIF board.

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
BFENABLE = 19  # BFPowerEnable - Beamformer Power Enable - 'high' to turn ON power. Only have this on when DOCPOWER is on.

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
    GPIO.setup(BFENABLE, GPIO.OUT)
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
        """
        Create a BFIFHandler instance.

        :param logger: Optional logging.Logger() instance.
        """
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
            self.bfenabled = bool(GPIO.input(BFENABLE))  # True if the enable bit is set on the RxDoC, powering up the beamformer.
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
            self.disable_bf()  # Leave the beamformer turned off.
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
                      'bfpower':self.bfenabled,
                      'opmode':self.opmode,
                      'rfof':self.rfof,
                      'auxpower':self.auxpower,
                      'serialpower':self.serialpower,
                      'serialmode':self.serialmode}
        return status

    def enable_bf(self):
        """
        Turn on the output power enable FET on the RxDoC card. This must be called _after_ powering up the DoC card.

        Return True if there were no errors, False if there was an error, None if the DoC card was off when called.

        :return: boolean, True for success, False for failure
        """
        if not self.docpower:
            self.logger.error("BFIFHandler - must turn on power to the doc before enabling it")
            return None
        with self.lock:
            self.logger.debug('BFIFHandler - Turning ON 48V to beamformer')
            GPIO.output(BFENABLE, 1)
            self.bfenabled = bool(GPIO.input(BFENABLE))
        return self.bfenabled is True

    def disable_bf(self):
        """
        Turn off the output power enable FET on the RxDoC card.

        Return True if there were no errors, False if there was an error.

        :return: boolean, True for success, False for failure
        """
        with self.lock:
            self.logger.debug('BFIFHandler - Turning OFF 48V to beamformer')
            GPIO.output(BFENABLE, 0)
            self.bfenabled = bool(GPIO.input(BFENABLE))
        return self.bfenabled is False

    def turnon_doc(self):
        """
        Switch the power on to the RxDoC card, using the FET on the BFIF board. If the beamformer is enabled
        when this method is called, it will be automatically disabled before power is turned on.

        Return True if there were no errors, False if there was an error.

        :return: boolean, True for success, False for failure
        """
        with self.lock:
            if self.bfenabled:
                self.disable_bf()  # If the DoC card is enabled, disable it BEFORE turning the power on.
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
            if self.bfenabled:
                self.disable_bf()
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

            if not self.enable_bf():  # Enable Beamformer signal to the RxDoc.
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

            if not self.disable_bf():  # Disable Beamformer signal to the RxDoc.
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
            ok1 = self.disable_bf()
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
                  BDICT[self.bfenabled],
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
