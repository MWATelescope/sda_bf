#!/usr/bin/env python

"""
Script to point a single tile at a given azimuth and elevation.

Written by Andrew Williams (Andrew.Williams@curtin.edu.au)

"""

import argparse
import atexit
import logging
import signal
import sys
import time

logging.basicConfig(level=logging.INFO)
LOGGER = logging.getLogger('bftest')

import bfif_lib
import tile_geometry

SIGNAL_HANDLERS = {}
CLEANUP_FUNCTION = None

DESCRIPTION = """Points the beamformer.
"""


def shutdown():
    print('Shutting down, turning off RxDoC card and beamformer.')
    if BFIF is not None:
        BFIF.cleanup()
    bfif_lib.cleanup_gpio()
    print('Shut down.')


# noinspection PyUnusedLocal
def SignalHandler(signum=None, frame=None):
    """Called when a signal is received thay would result in the programme exit, if the
       RegisterCleanup() function has been previously called to set the signal handlers and
       define an exit function using the 'atexit' module.

       Note that exit functions registered by atexit are NOT called when the programme exits due
       to a received signal, so we must trap signals where possible. The cleanup function will NOT
       be called when signal 9 (SIGKILL) is received, as this signal cannot be trapped.
    """
    sys.exit(-signum)  # Called by signal handler, so exit with a return code indicating the signal received, AFTER
    # calling the cleanup function registered by the atexit.register() call in RegisterCleanup()


def RegisterCleanup(func):
    """Traps a number of signals that would result in the program exit, to make sure that the
       function 'func' is called before exit. The calling process must define its own cleanup
       function - typically this would delete any facility controller objects, so that any
       processes they have started will be stopped.

       We don't need to trap signal 2 (SIGINT), because this is internally handled by the python
       interpreter, generating a KeyboardInterrupt exception - if this causes the process to exit,
       the function registered by atexit.register() will be called automatically.
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
    parser = argparse.ArgumentParser(description=DESCRIPTION,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--alt", "--el", type=float, default=90.0, help="Elevation angle in degrees (default 90.0)")
    parser.add_argument("--az", type=float, default=0.0, help="Azimuth angle in degrees (default 0.0)")
    args = parser.parse_args()

    bfif_lib.setup_gpio()  # Need to call this before using the library

    BFIF = bfif_lib.BFIFHandler(logger=LOGGER)
    # Turn on the beamformer
    BFIF.turnon_doc()
    time.sleep(0.5)
    BFIF.turnon_bf()
    time.sleep(2)

    BFIF.check()
    print("RxDoC card status: Voltage=%5.2f V, Current=%5.3f A, Temp=%4.1f degC" % (BFIF.voltage, BFIF.current, BFIF.temp))

    BF = bfif_lib.BFHandler(logger=LOGGER)

    delays = tile_geometry.calc_delays(az=args.az, el=args.alt)
    BF.point(xdelays=delays, ydelays=delays)
    print("Tile pointed to Azimuth=%5.1f, Elevation=%4.1f" % (args.az, args.alt))

    if sys.version_info[0] == 3:
        input('Press ENTER to turn off beamformer, and exit: ')
    else:
        # noinspection PyUnresolvedReferences
        raw_input('Press ENTER to turn off beamformer, and exit: ')

    RegisterCleanup(shutdown)  # Trap signals and register the cleanup() function to be run on exit.
