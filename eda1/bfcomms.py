#!/usr/bin/env python

"""
Runs on one Raspberry Pi inside each of the beamformer control boxes, to send pointing commands to the eight
beamformers connected to that box.

On startup, it loops forever, pointing all 8 beamformers at the zenith, every 10 seconds.

On exit (eg, with a control-C or a 'kill' command), it:
    -Cleans up the GPIO pins.
    -Exits.
"""

import atexit
import datetime
import logging
from logging import handlers
import signal
import sys
import time

# noinspection PyUnresolvedReferences
import RPi.GPIO as GPIO

from mwabf import beamformer

# set up the logging
LOGLEVEL_CONSOLE = logging.INFO  # Logging level for console messages (INFO, DEBUG, ERROR, CRITICAL, etc)
LOGLEVEL_LOGFILE = logging.DEBUG  # Logging level for logfile
LOGFILE = "/tmp/bfcomms.log"


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

fh = handlers.RotatingFileHandler(LOGFILE,
                                  maxBytes=1000000000,
                                  backupCount=5)  # 1 Gb per file, max of five old log files
fh.setLevel(LOGLEVEL_LOGFILE)
fh.setFormatter(mwalf)

ch = logging.StreamHandler()
ch.setLevel(LOGLEVEL_CONSOLE)
ch.setFormatter(mwalf)

# add the handlers to the logger
LOGGER.addHandler(fh)
LOGGER.addHandler(ch)


# IO pin allocations as (txdata, txclock, rxdata) for each of the 8 RxDOC cards in this box, numbered 1-8
IOPINS = {1:(29, 16, 40), 2:(26, 15, 38), 3:(24, 13, 37), 4:(23, 12, 36), 5:(22, 11, 35), 6:(21, 10, 33), 7:(19, 8, 32),
          8:(18, 7, 31)}

BFS = {}   # Dict with bfnum (1-8) as key, and mwabf.beamformer.BFHandler as values

SIGNAL_HANDLERS = {}
CLEANUP_FUNCTION = None


def init():
    """
    Initialise IO pins for pointing comms with all 8 beamformers.

    :return: None
    """
    GPIO.setmode(GPIO.BOARD)  # Use board connector pin numbers to specify I/O pins
    GPIO.setwarnings(False)
    for i in range(1, 9):
        txdata, txclock, rxdata = IOPINS[i]
        GPIO.setup(rxdata, GPIO.IN)
        GPIO.setup(txdata, GPIO.OUT)
        GPIO.setup(txclock, GPIO.OUT)


def cleanup():
    """
    Called on program exit, to clean up GPIO pins gracefully.

    :return: None
    """
    LOGGER.info("Cleaning up GPIO library")
    GPIO.cleanup()


# noinspection PyUnusedLocal
def SignalHandler(signum=None, frame=None):
    """
    Called when a signal is received that would result in the programme exit, if the
    RegisterCleanup() function has been previously called to set the signal handlers and
    define an exit function using the 'atexit' module.

    Note that exit functions registered by atexit are NOT called when the programme exits due
    to a received signal, so we must trap signals where possible. The cleanup function will NOT
    be called when signal 9 (SIGKILL) is received, as this signal cannot be trapped.

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

    init()
    RegisterCleanup(cleanup)

    for slot in range(1, 9):
        txd, txc, rxd = IOPINS[slot]
        BFS[slot] = beamformer.BFHandler(txdata=txd, txclock=txc, rxdata=rxd, logger=LOGGER)

    # All zero delays - point at the zenith
    xdelays = [0] * 16
    ydelays = [0] * 16
    while True:
        for bfnum in range(1, 9):
            temp, flags = BFS[bfnum].point(xdelays=xdelays, ydelays=ydelays)
            LOGGER.info("bf %d bitstring sent, return flags=%d, temp=%4.1f." % (bfnum, flags, temp))
        time.sleep(10)
