
import logging
import threading
import time

# noinspection PyUnresolvedReferences
import RPi.GPIO as GPIO
# noinspection PyUnresolvedReferences
import smbus

logging.basicConfig()


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

    Note that powering the beamformer up and down is handled elsewhere, and depends on the board that
    the RxDoC card is plugged into.
    """

    def __init__(self, txdata, rxdata, txclock, logger=logging):
        """
        Create a new beamformer instance, given the IO pin numbers needed to control it.

        :param txdata: GPIO pin number for transmitted data output
        :param rxdata: GPIO pin number for received data input
        :param txclock: GPIO pin number for transmitted clock output
        :param logger: Optional logging.Logger() instance
        """
        self.txdata = txdata
        self.rxdata = rxdata
        self.txclock = txclock
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

        Returns a tuple of (temp, flags) where 'temp' is the beamformer temperature in deg C, and
        'flags' is the flag value (128 if there were no comms errors).

        :param outstring: String containing 253 '1's and '0's to send to the beamformer
        :param bittime: Total time to send one bit of data, in seconds.
        :return: Tuple of (temp, flags), where temp is a float containing beamformer temperature, and flags is an int
                 which should be equal to 128 if the communications were successful.
        """
        with self.lock:
            for bit in outstring:
                GPIO.output(self.txdata, {'1':1, '0':0}[bit])
                time.sleep(bittime / 4)  # wait for data bit to settle
                GPIO.output(self.txclock, 1)  # Send clock high
                time.sleep(bittime / 2)  # Leave clock high for half the total bit transmit time
                GPIO.output(self.txclock, 0)  # Send clock low,so data is valid on both rising and falling edge
                time.sleep(bittime / 4)  # Leave data valid until the end of the bit transmit time

            # While the temperature is 16 bits and the checksum is 8 bits, giving 24
            # bits in total, we appear to have to clock an extra bit-time to complete the
            # read-back operation. Once that's done, the checksum is the final (right-
            # most) 8 bits, and the temperature is 13 bits (signed plus 12-bits). Both
            # values are most-significant-bit first (chronologically).

            GPIO.output(self.txdata, 0)
            inbits = []
            for i in range(25):  # Read in temp data
                time.sleep(bittime / 4)
                GPIO.output(self.txclock, 1)
                time.sleep(bittime / 4)
                inbits.append({True:'1', False:'0'}[GPIO.input(self.rxdata)])
                time.sleep(bittime / 4)
                GPIO.output(self.txclock, 0)
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
