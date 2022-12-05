#!/usr/bin/env python

"""
Runs on one Raspberry Pi inside a beamformer control box, to do power control and voltage/current
monitoring for the eight beamformers connected to that box.

On startup, it:
    -Turns on the 48V DC supply.
    -Turns on, and enables, each of the eight beamformer DoC cards in that box.
    -Loops forever, writing voltage and current information to stdout, and to /tmp/edamc.log.

On exit (eg, with a control-C or a 'kill' command), it:
    -Disables and turns off all the beamformer DoC cards in that box.
    -Turns off the 48V DC supply.
    -Exits.

Make sure that this process is not running before connecting or disconnecting beamformers to the DoC cards, so that the
sockets aren't 'live' with 48V DC.
"""

import atexit
import datetime
import logging
import signal
import sys
import threading
import time

# noinspection PyUnresolvedReferences
import RPi.GPIO as GPIO
# noinspection PyUnresolvedReferences
import smbus

# set up the logging

LOGLEVEL_CONSOLE = logging.INFO  # Logging level for console messages (INFO, DEBUG, ERROR, CRITICAL, etc)
LOGLEVEL_LOGFILE = logging.DEBUG  # Logging level for logfile
LOGLEVEL_REMOTE = logging.INFO
LOGFILE = "/tmp/bfpower.log"


class MWALogFormatter(logging.Formatter):
    def format(self, record):
        return "%s:%s - %s" % (datetime.datetime.now().isoformat(sep=' ')[:-7],
                               record.levelname,
                               record.getMessage())


mwalf = MWALogFormatter()

LOGGER = logging.getLogger()
LOGGER.setLevel(logging.DEBUG)
LOGGER.handlers = []
LOGGER.propagate = False

fh = logging.FileHandler(LOGFILE)
fh.setLevel(LOGLEVEL_LOGFILE)
fh.setFormatter(mwalf)

ch = logging.StreamHandler()
ch.setLevel(LOGLEVEL_CONSOLE)
ch.setFormatter(mwalf)

# rh = handlers.SysLogHandler(address=('mw-gw'))
# rh.setLevel(LOGLEVEL_REMOTE)
# rh.setFormatter(mwalf)

# add the handlers to the logger
LOGGER.addHandler(fh)
LOGGER.addHandler(ch)
# logger.addHandler(rh)

STATUS = None

# IO pin allocations as (enable, power) for each of the 8 RxDOC cards in this box, numbered 1-8
BFIOPINS = {1:(29, 16), 2:(26, 15), 3:(24, 13), 4:(23, 12), 5:(22, 11), 6:(21, 10), 7:(19, 8), 8:(18, 7)}

# I2C addresses for the eight LTC4151's, one on each RxDoC card. Note that these are _seven_ bit addresses,
# corresponding to D0,D2,D4,...DE when the r/w bit is appended as bit 0 of the address.
ADDRESSES = {1:0x68, 2:0x69, 3:0x6a, 4:0x6b, 5:0x6c, 6:0x6d, 7:0x6e, 8:0x6f}

POWER48 = 32
ALARMPOWER = 36
ALARM48 = 38
DIGOUT1 = 33
DIGOUT2 = 35
DIGIN1 = 40

BDICT = {False:'OFF', True:'ON'}

SIGNAL_HANDLERS = {}
CLEANUP_FUNCTION = None


class RxDoC(object):
    """
    Class to monitor and control a single RxDoC card
    """
    def __init__(self, bfnum, enable_pin, power_pin, i2c_address, bus, logger=LOGGER):
        """
        Create an instance of an RxDoC controller.

        :param bfnum: Slot number (1-8)
        :param enable_pin: GPIO pin number for the beamformer enable pin on the RxDoC card
        :param power_pin: GPIO pin number for 48V power control to the RxDoC card
        :param i2c_address: Address of the LTC4151 on this RxDoC card
        :param bus: An smbus.SMBus() instance for I2C communications
        :param logger: Optional logging.Logger() instance
        """
        self.bfnum = bfnum
        self.enable_pin = enable_pin
        self.power_pin = power_pin
        self.i2c_address = i2c_address
        self.bus = bus
        self.logger = logger
        self.current = 0.0
        self.voltage = 0.0
        self.enabled = bool(GPIO.input(self.enable_pin))
        self.power = bool(GPIO.input(self.power_pin))
        self.lock = threading.RLock()  # Used to control access to the hardware resources (i2C, GPIO) on the board

    def check(self):
        """
        Communicate with the hardware to update current pin states, voltage, and current values.

        :return: None
        """
        with self.lock:
            self.enabled = bool(GPIO.input(self.enable_pin))
            self.power = bool(GPIO.input(self.power_pin))
            try:
                data = self.bus.read_i2c_block_data(self.i2c_address, 0, 4)
                self.current = ((data[0] * 16) + (data[1] / 16)) * 20e-6 / 0.02  # 20uV per ADU, through a 0.02 Ohm shunt
                self.voltage = ((data[2] * 16) + (data[3] / 16)) * 0.025  # 25mV per ADU
            except IOError:
                self.current = 0.0
                self.voltage = 0.0

    def __repr__(self):
        return "BF# %d: power=%3s, enable=%3s, voltage=%5.2f V, current=%4.0f mA" % (self.bfnum,
                                                                                     BDICT[self.power],
                                                                                     BDICT[self.enabled],
                                                                                     self.voltage,
                                                                                     self.current * 1000)

    def enable_bf(self):
        """
        Turn on the output power enable FET on the RxDoC card. This must be called _after_ powering up the DoC card.

        Return True if there were no errors, False if there was an error, None if the DoC card was off when called.

        :return: boolean, True for success, False for failure
        """
        if not self.power:
            self.logger.error("RxDoC - must turn on power to the doc before enabling it")
            return None
        self.logger.debug('RxDoC - Turning ON 48V to beamformer')
        with self.lock:
            GPIO.output(self.enable_pin, 1)
            self.enabled = bool(GPIO.input(self.enable_pin))
        return self.enabled is True

    def disable_bf(self):
        """
        Turn off the output power enable FET on the RxDoC card.

        Return True if there were no errors, False if there was an error.

        :return: boolean, True for success, False for failure
        """
        self.logger.debug('RxDoC - Turning OFF 48V to beamformer')
        with self.lock:
            GPIO.output(self.enable_pin, 0)
            self.enabled = bool(GPIO.input(self.enable_pin))
        return self.enabled is False

    def turnon_doc(self):
        """
        Switch the power on to the RxDoC card, using the FET on the BF Controller board. If the beamformer is enabled
        when this method is called, it will be automatically disabled before power is turned on.

        Return True if there were no errors, False if there was an error.

        :return: boolean, True for success, False for failure
        """
        if self.enabled:
            self.disable_bf()  # If the DoC card is enabled, disable it BEFORE turning the power on.
            time.sleep(0.1)
        self.logger.debug('RxDoC - Turning ON 48V to DoC card')
        with self.lock:
            GPIO.output(self.power_pin, 1)
            self.power = bool(GPIO.input(self.power_pin))
        return self.power is True

    def turnoff_doc(self):
        """
        Switch the power off to the RxDoC card, using the FET on the BFIF board. If the beamformer is enabled
        when this method is called, it will be automatically disabled before power is turned off.

        Return True if there were no errors, False if there was an error.

        :return: boolean, True for success, False for failure
        """
        if self.enabled:
            self.disable_bf()
            time.sleep(0.1)
        self.logger.debug('RxDoC - Turning OFF 48V to DoC card')
        with self.lock:
            GPIO.output(self.power_pin, 0)
            self.power = bool(GPIO.input(self.power_pin))
        return self.power is False


class BFController(object):
    """
    Class to monitor the status of all eight RxDoC cards, as well as the 48V power supply.
    """
    def __init__(self):
        """
        Create an global status instance, and all eight DOCstatus instances, one for each RxDoC card.
        """
        self.bfs = {}
        self.power48 = bool(GPIO.input(POWER48))
        self.alarmpower = None
        self.alarm48 = None
        bus = smbus.SMBus(1)  # Initialise the I2C bus and save the connection object
        for bfnum in range(1, 9):
            ep, pp = BFIOPINS[bfnum]
            self.bfs[bfnum] = RxDoC(bfnum=bfnum,
                                    enable_pin=ep,
                                    power_pin=pp,
                                    i2c_address=ADDRESSES[bfnum],
                                    bus=bus)
        self.check()

    def check(self):
        """
        Communicate with the hardware to update current pin states, voltage, and current values.

        :return: None
        """
        self.alarmpower = bool(GPIO.input(ALARMPOWER))
        self.alarm48 = bool(GPIO.input(ALARM48))

        pled = 1  # Box RxDoC power LED, on if all RxDoC's are on
        eled = 1  # Box RxDoC enable LED, on if all RxDoC's are enabled
        for bfnum in range(1, 9):
            self.bfs[bfnum].check()
            if not self.bfs[bfnum].power:
                pled = 0
            if not self.bfs[bfnum].enabled:
                eled = 0

        # Turn the box RxDoC power and enable LEDs on or off
        GPIO.output(DIGOUT1, pled)
        GPIO.output(DIGOUT2, eled)

    def __repr__(self):
        rets = "EDA Status: 48V=%3s (Alarm=%3s)\n" % (BDICT[self.power48], BDICT[self.alarm48])
        rets += "  Beamformers:\n"
        for bfnum in range(1, 9):
            rets += '    ' + repr(self.bfs[bfnum]) + '\n'
        return rets

    def turn_on_48(self):
        """
        Turn on the 48V power supply. If any beamformers are enabled and/or turned on, turn them off first to
        avoid damage.

        :return: True for success, False on error
        """
        if True in [x.enabled for x in self.bfs.values()]:
            for bf in self.bfs.values():
                bf.disable_bf()
        if True in [x.power for x in self.bfs.values()]:
            for bf in self.bfs.values():
                bf.turnoff_doc()
        time.sleep(0.2)
        GPIO.output(POWER48, 1)
        time.sleep(0.1)
        self.power48 = bool(GPIO.input(POWER48))
        return (self.power48 is True)

    def turn_off_48(self):
        """
        Turn off the 48V power supply. If any beamformers are enabled and/or turned on, turn them off first to
        avoid damage.

        :return: True for success, False on error
        """
        if True in [x.enabled for x in self.bfs.values()]:
            for bf in self.bfs.values():
                bf.disable_bf()
        if True in [x.power for x in self.bfs.values()]:
            for bf in self.bfs.values():
                bf.turnoff_doc()
        GPIO.output(POWER48, 0)
        time.sleep(0.1)
        self.power48 = bool(GPIO.input(POWER48))
        return (self.power48 is False)

    def turnon_all(self):
        """
        Turn on the 48V power supply, then turn on and enable all eight RxDoC cards.

        :return: None
        """
        self.turn_on_48()
        time.sleep(0.5)
        powerok = True
        enabled_ok = True
        for bf in self.bfs.values():
            powerok &= bool(bf.turnon_doc())
            time.sleep(0.1)
            enabled_ok &= bool(bf.enable_bf())
        GPIO.output(DIGOUT1, powerok)
        GPIO.output(DIGOUT2, enabled_ok)

    def turnoff_all(self):
        """
        Turn off and disable all eight RxDoC cards, then turn off the 48V power supply.

        :return: None
        """
        powerok = True
        enabled_ok = True
        for bf in self.bfs.values():
            enabled_ok &= bool(bf.disable_bf())
            time.sleep(0.1)
            powerok &= bool(bf.turnoff_doc())
        self.turn_off_48()
        GPIO.output(DIGOUT1, not powerok)
        GPIO.output(DIGOUT2, not enabled_ok)


def init():
    """
    Initialise IO pins for power/enable control with all 8 beamformers,
    and create the global STATUS object.

    :return: None
    """
    global STATUS
    GPIO.setmode(GPIO.BOARD)  # Use board connector pin numbers to specify I/O pins
    GPIO.setwarnings(False)
    GPIO.setup(POWER48, GPIO.OUT)
    GPIO.setup(ALARMPOWER, GPIO.IN)
    GPIO.setup(ALARM48, GPIO.IN)
    GPIO.setup(DIGOUT1, GPIO.OUT)
    GPIO.setup(DIGOUT2, GPIO.OUT)
    GPIO.setup(DIGIN1, GPIO.IN)
    for i in range(1, 9):
        enable, power = BFIOPINS[i]
        GPIO.setup(enable, GPIO.OUT)
        GPIO.setup(power, GPIO.OUT)


def cleanup():
    """
    Called on exit - turns everything off, releases GPIO pins.

    :return: None
    """
    LOGGER.info("Turning off all eight beamformers, and 48V supplies")
    BFCON.turnoff_all()
    LOGGER.info("Cleaning up GPIO library")
    GPIO.cleanup()


# noinspection PyUnusedLocal
def SignalHandler(signum=None, frame=None):
    """
    Called when a signal is received thay would result in the programme exit, if the
    RegisterCleanup() function has been previously called to set the signal handlers and
    define an exit function using the 'atexit' module.

    Note that exit functions registered by atexit are NOT called when the programme exits due
    to a received signal, so we must trap signals where possible. The cleanup function will NOT
    be called when signal 9 (SIGKILL) is received, as this signal cannot be trapped.

    :param signum: Signal number trapped
    :param frame: Stack frame (unused)
    :return: None
    """
    print("Signal %d received." % signum)
    sys.exit(-signum)  # Called by signal handler, so exit with a return code indicating the signal received


def RegisterCleanup(func):
    """
    Traps a number of signals that would result in the program exit, to make sure that the
    function 'func' is called before exit. The calling process must define its own cleanup
    function - typically this would delete any facility controller objects, so that any
    processes they have started will be stopped.

    We don't need to trap signal 2 (SIGINT), because this is internally handled by the python
    interpreter, generating a KeyboardInterrupt exception - if this causes the process to exit,
    the function registered by atexit.register() will be called automatically.

    :param func: Function to be called on exit
    :return: None
    """
    global SIGNAL_HANDLERS, CLEANUP_FUNCTION
    CLEANUP_FUNCTION = func
    for sig in [3, 15]:
        SIGNAL_HANDLERS[sig] = signal.signal(sig, SignalHandler)  # Register a signal handler
    SIGNAL_HANDLERS[1] = signal.signal(1, signal.SIG_IGN)
    # Register the passed CLEANUP_FUNCTION to be called on
    # on normal programme exit, with no arguments.
    atexit.register(CLEANUP_FUNCTION)


if __name__ == '__main__':
    init()  # Set up GPIO pins

    BFCON = BFController()
    RegisterCleanup(cleanup)

    LOGGER.info("Turning on 48V supplies and all eight beamformers")
    BFCON.turnon_all()

    while True:
        BFCON.check()
        LOGGER.info(str(BFCON))
        time.sleep(10)
