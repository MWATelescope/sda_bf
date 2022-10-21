#!/usr/bin/env python

"""
Script to exercise and test all dipoles and/or delay lines in an MWA beamformer.
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


ALLDELAYS = {'Z':('Zen', [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]),
             'A':('DipA', [0, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32]),
             'B':('DipB', [32, 0, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32]),
             'C':('DipC', [32, 32, 0, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32]),
             'D':('DipD', [32, 32, 32, 0, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32]),
             'E':('DipE', [32, 32, 32, 32, 0, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32]),
             'F':('DipF', [32, 32, 32, 32, 32, 0, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32]),
             'G':('DipG', [32, 32, 32, 32, 32, 32, 0, 32, 32, 32, 32, 32, 32, 32, 32, 32]),
             'H':('DipH', [32, 32, 32, 32, 32, 32, 32, 0, 32, 32, 32, 32, 32, 32, 32, 32]),
             'I':('DipI', [32, 32, 32, 32, 32, 32, 32, 32, 0, 32, 32, 32, 32, 32, 32, 32]),
             'J':('DipJ', [32, 32, 32, 32, 32, 32, 32, 32, 32, 0, 32, 32, 32, 32, 32, 32]),
             'K':('DipK', [32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 0, 32, 32, 32, 32, 32]),
             'L':('DipL', [32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 0, 32, 32, 32, 32]),
             'M':('DipM', [32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 0, 32, 32, 32]),
             'N':('DipN', [32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 0, 32, 32]),
             'O':('DipO', [32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 0, 32]),
             'P':('DipP', [32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 0]),
             None:('', [0] * 16)}

DELDELAYS = {'0':('None', [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]),
             '1':('Del1', [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]),
             '2':('Del2', [2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2]),
             '3':('Del3', [4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4]),
             '4':('Del4', [8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8]),
             '5':('Del5', [16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16]),
             '6':('All ', [31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31, 31]),
             '7':('Off ', [32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32]),
             None:('', [0] * 16)}

NTESTS, NFAILED = 0, 0  # Number of total pointings in the current mode, and number of failed pointings.

SIGNAL_HANDLERS = {}
CLEANUP_FUNCTION = None

DESCRIPTION = """"Tests one dipole (A-P) at a time with zero delay, or one delay line 
setting (0-6) at a time with all dipoles selected. By default each test is run once, 
waiting for a keypress each time, after which the program exits and the beamformer is
turned off.

Tests are specified as one or more mode strings on the command line. Each mode string is
either the word 'dipole' (optionally followed, with no spaces, by one or more dipole letters,
in any order), or the word 'delay' (optionally followed, with no spaces, by one or more
delay line specifiers, in any order). If no mode string is given, the default ('dipole')
will be used.

For example, 'dipole' will test all dipoles, 'dipoleBCD' will just test dipoles B, C, and D,
'delay' will test all delays, 'delay024' will test only delays 0, 2 and 4.
"""


def mainloop(modes=None, maxloops=1, delaytime=0.0):
    global NTESTS, NFAILED
    NTESTS, NFAILED = 0, 0
    nloops = 0

    if modes is None:
        modes = ['DIPOLE']

    while (maxloops == 0) or (nloops < maxloops):
        for mode in modes:
            if 'DIPOLE' in mode:
                if len(mode) == len('DIPOLE'):
                    dlist = 'ABCDEFGHIJKLMNOP'
                else:
                    dlist = mode[len('DIPOLE'):]
            elif 'DELAY' in mode:
                if len(mode) == len('DELAY'):
                    dlist = '0123456'
                else:
                    dlist = mode[len('DELAY'):]
            else:
                dlist = '01234567'

            for delayindex in dlist:
                if 'DIPOLE' in mode:
                    delayname, currdelays = ALLDELAYS[delayindex]
                elif 'DELAY' in mode:
                    delayname, currdelays = DELDELAYS[delayindex]
                else:
                    delayname, currdelays = ALLDELAYS[None]

                temp, flags = BF.point(xdelays=currdelays, ydelays=currdelays)  # Do the tests and increment the counter/s
                NTESTS += 1
                if (flags != 0x80):  # If there's a comms failure, and it's NOT the first test, increment the fail count
                    NFAILED += 1
                    badstr = '** BAD **'
                else:
                    badstr = '         '

                if 'DIPOLE' in mode:
                    print('Dipole %s: Temp=%4.1f Flags=%02x  %s' % (delayname, temp, flags, badstr))
                else:
                    print('Delay %s: Temp=%4.1f Flags=%02x  %s' % (delayname, temp, flags, badstr))

                if (divmod(NTESTS, 100)[1] == 0):   # Every 100th test:
                    print('Test %04d, of which %04d were bad' % (NTESTS, NFAILED))

                if delaytime == 0.0:
                    if sys.version_info[0] == 3:
                        input('Press ENTER to continue: ')
                    else:
                        # noinspection PyUnresolvedReferences
                        raw_input('Press ENTER to continue: ')
                else:
                    time.sleep(delaytime)

        nloops += 1


def shutdown():
    print('Total of %d tests, of which %04d were bad' % (NTESTS, NFAILED))
    print('Shutting down')
    time.sleep(0.5)
    if BFIF is not None:
        BFIF.cleanup()
    bfif_lib.cleanup_gpio()
    print('Shut down')


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
    parser = argparse.ArgumentParser(description=DESCRIPTION)
    parser.add_argument("--maxloops", "-m",
                        dest='maxloops',
                        default=1,
                        type=int,
                        help="Number of times to loop over all delays/dipoles for the mode/s specified, before exiting. To loop forever, specify 0.")
    parser.add_argument("--delaytime", "-d",
                        dest='delaytime',
                        default=0.0,
                        type=float,
                        help="Delay time between modes, 0 to wait for a keypress")
    parser.add_argument('modes', nargs='+', help="One or more test modes. Either 'dipole[[A-P][A-P]...' or 'delay[[0-6][0-6]...'")
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

    if args.modes:
        modes = [x.upper().strip() for x in args.modes]
    else:
        modes = ['DIPOLE']

    RegisterCleanup(shutdown)  # Trap signals and register the cleanup() function to be run on exit.

    mainloop(modes=modes, maxloops=args.maxloops, delaytime=args.delaytime)
