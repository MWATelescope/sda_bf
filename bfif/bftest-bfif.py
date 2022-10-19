#!/usr/bin/env python

import atexit
import optparse
import signal
import sys
import time

import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)  # Use board connector pin numbers to specify I/O pins
# IO pin allocations on the Raspberry Pi GPIO header
DOCPOWER = 18  # DoCPwrEnable - 5V and 48V DoC power Enable - equivilant to receiver_power. 'high' to turn ON power.
BFPOWER = 19  # BFPowerEnable - Beamformer Power Enable - equivilant to beamer_power. Only have this on when DOCPOWER is on. 'high' to turn ON power.
AUXOFF = 22  # VauxPowerOff - 'high' to turn OFF power supply running media converter on a BFIF, or the LNA supply on the BF tester

TXDATA = 13  # DoCTxData - DoC Data Output
TXCLOCK = 12  # DoCTxClock - DoC Transmit Clock Output
RXDATA = 15  # DoCRxData - DoC Data Input

BUTTONUP = 16  # Push button, high when pushed, low when released
BUTTONDN = 29  # Push button, low when pushed, high when released

GPIO.setup(RXDATA, GPIO.IN)
GPIO.setup(TXDATA, GPIO.OUT)
GPIO.setup(TXCLOCK, GPIO.OUT)
GPIO.setup(DOCPOWER, GPIO.OUT)
GPIO.setup(BFPOWER, GPIO.OUT)
GPIO.setup(AUXOFF, GPIO.OUT)
GPIO.setup(BUTTONUP, GPIO.IN)
GPIO.setup(BUTTONDN, GPIO.IN)

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


def gen_bitstring(xdelays=None, ydelays=None):
    """Given two arrays of 16 integers, representing the xdelays and ydelays, return
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
    """
    outlist = ['0' * 8, '1' * 4, '0' * 20]
    dwords = []
    for val in (xdelays + ydelays):
        if (val < 0) or (val > 63):
            return  # Each delay value must fit in 6 bits
        else:
            dwords.append('{0:06b}'.format(val))
    dstring = ''.join(dwords)
    checksum = 0
    for i in range(0, 12 * 16, 16):
        checksum = checksum ^ int(dstring[i:i + 16], 2)
        outlist.append(dstring[i:i + 16] + '1')
    outlist.append('{0:016b}'.format(checksum))
    return ''.join(outlist)  # 253 bits of output data


def send_bitstring(outstring, bittime=0.00002):
    """Given a string of 253 '1' or '0' characters, clock them out using the TXDATA and TXCLOCK
       pins, then clock in 24 bits of temp and flag data from the RXDATA pin.

       bittime is the total time to take to send one but, in seconds.
    """
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
    temp = 0.0625 * (rawtemp & 0xfff)  # 12 bits of value
    if (rawtemp & 0x1000):
        temp -= 256.0
    flags = int(''.join(inbits[17:]), 2)
    return temp, flags  # Temp should be a value in degC, flags should be 0x80


def do_pointing(xdelays=None, ydelays=None):
    temp, flags = send_bitstring(gen_bitstring(xdelays=xdelays, ydelays=ydelays))
    return temp, flags  # Temp should be a value in degC, flags should be 0x80


def check_power():
    """Checks that the 5V and 48V rail to Rx-DOC, and bf-enable output, are all on. Returns
       a tuple of booleans for 5V, 48V, BFenable, where each is True if a given rail is up,
       False if it's down.

       Result will be (True, True) if the power supplies and BF enable bits are all
       on and ready for a pointing.
    """
    return (GPIO.input(DOCPOWER), GPIO.input(BFPOWER))


# When turning the supply rails on and off, it's important to do things in the right order.
# 5V first, then 48V, then enable the beam former. The reverse is also true when turning things off:
# disable the beamformer, drop the 48V rail, then the 5V rail. The following "power" functions
# enforce this discipline.

def rxdoc_poweron():
    """Turn ON the 5V supply to the RxDoC board. This must be done before applying 48V power
       or turning on the BFenable pin.

       Returns True if the 5V supply is turned on after this call, False otherwise.
    """
    if (GPIO.input(BFPOWER)):
        bf_disable()
        time.sleep(0.1)

    GPIO.output(DOCPOWER, 1)

    if GPIO.input(DOCPOWER):
        print('5V/48V power is ON!!!')
    else:
        print('5V/48V power is off.')

    return bool(GPIO.input(DOCPOWER))


def rxdoc_poweroff():
    """Turn OFF the 5V supply to the RxDoC board. If the BFenable and/or 48V supplies are on
       when called, turn them off first, then turn off the 5V supply.

       Returns True if the 5V supply is turned off after this call, False otherwise.
    """
    if (GPIO.input(BFPOWER)):
        bf_disable()
        time.sleep(0.1)

    GPIO.output(DOCPOWER, 0)

    if GPIO.input(DOCPOWER):
        print('5V/48V power is ON!!!')
    else:
        print('5V/48V power is off.')

    return not bool(GPIO.input(DOCPOWER))


def bf_enable():
    """If the 5V and 48V supplies supply to the RxDoC are already on, then turn on the BFenable,
       otherwise do nothing.

       Returns True if the BFenable is turned on after this call, False otherwise.
    """
    if GPIO.input(DOCPOWER):
        GPIO.output(BFPOWER, 1)

    if GPIO.input(BFPOWER):
        print('BF power is ON!!!')
    else:
        print('BF power is off.')

    return bool(GPIO.input(BFPOWER))


def bf_disable():
    """Turn off the BFenable.

       Returns True if the BFenable is turned off after this call, False otherwise.
    """
    GPIO.output(BFPOWER, 0)

    if GPIO.input(BFPOWER):
        print('BF power is ON!!!')
    else:
        print('BF power is off.')

    return not bool(GPIO.input(BFPOWER))


def lna_poweron():
    """Turn ON the 5V supply to the LNA output.

       Returns True if the 5V supply is turned on after this call, False otherwise.
    """
    GPIO.output(AUXOFF, 0)

    if not GPIO.input(AUXOFF):
        print('LNA power is ON!!!')
    else:
        print('LNA power is off.')

    return not bool(GPIO.input(AUXOFF))


def lna_poweroff():
    """Turn OFF the 5V supply to the LNA output.

       Returns True if the 5V supply is turned off after this call, False otherwise.
    """
    GPIO.output(AUXOFF, 1)

    if not GPIO.input(AUXOFF):
        print('LNA power is ON!!!     ')
    else:
        print('LNA power is off.      ')

    return bool(GPIO.input(AUXOFF))


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

                temp, flags = do_pointing(xdelays=currdelays,
                                          ydelays=currdelays)  # Do the tests and increment the counter/s
                NTESTS += 1
                if (flags != 0x80):  # If there's a comms failure, and it's NOT the first test, increment the fail count
                    NFAILED += 1
                    badstr = '** BAD **'
                else:
                    badstr = '         '

                if GPIO.input(BUTTONUP):
                    butt_up = 1
                else:
                    butt_up = 0
                if GPIO.input(BUTTONDN):
                    butt_dn = 1
                else:
                    butt_dn = 0

                if 'DIPOLE' in mode:
                    print('Dipole %s: Temp=%4.1f Flags=%02x UP=%d, DN=%d   %s' % (delayname, temp, flags, butt_up, butt_dn, badstr))
                else:
                    print('Delay %s: Temp=%4.1f Flags=%02x UP=%d, DN=%d    %s' % (delayname, temp, flags, butt_up, butt_dn, badstr))

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
    bf_disable()
    rxdoc_poweroff()
    lna_poweroff()
    time.sleep(0.5)
    print('Shut down')

    GPIO.cleanup()


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
    parser = optparse.OptionParser(usage="usage: %prog [options]")
    parser.add_option("--maxloops", "-m",
                      dest='maxloops',
                      default=1,
                      type="int",
                      help="Number of times to loop over all delays/dipoles for the mode/s specified, before exiting. To loop forever, specify 0.")
    parser.add_option("--delaytime", "-d",
                      dest='delaytime',
                      default=0.0,
                      type="float",
                      help="Delay time between modes, 0 to wait for a keypress")
    (options, args) = parser.parse_args()

    # Turn on the beamformer
    rxdoc_poweron()
    lna_poweron()
    time.sleep(0.5)
    bf_enable()
    time.sleep(2)

    if args:
        modes = [x.upper().strip() for x in args]
    else:
        modes = ['DIPOLE']

    RegisterCleanup(shutdown)  # Trap signals and register the cleanup() function to be run on exit.

    mainloop(modes=modes, maxloops=options.maxloops, delaytime=options.delaytime)
