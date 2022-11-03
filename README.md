# sda_bf
Space Domain Awareness beamformer control/test code

Communicates with MWA beamformers via Raspberry Pi's installed on 
a custom PCB, connected to one or more RxDoC cards.

Files in the 'mwabf' directory are for controlling MWA beamformers 
connected to a Raspberry Pi's GPIO pins, whether it's on a BFIF
board (the lbtile package) or an 8-port beamfomer controller
(the eda1 package).

  - tile_geometry.py - functions to calculate beamformer delays from alt/az values.
  - beamformer.py - a class to monitor and point an MWA beamformer connected
    to a Raspberry Pi. The user passes in the GPIO pin numbers to use.

Files in the 'lbtile' directory are for PCB's based on the MWA long-baseline
tile hardware, where a single beamformer is connected via a 'Beam
Former Interface' box (BFIF).

  - bfif.py - classes to control a BFIF board and MWA beamformer.
  - bftest.py - script to exercise individual dipoles or delay lines.
  - point.py - script to turn on the BF and point it at a given alt/az.

Files in the 'eda1' directory are for SDA boxes based on the 
beamformer controllers used in the original 'Engineering Development
Array' (EDA). These boxes have two Raspberry Pi's, and control eight
MWA beamformers, using eight RxDoC cards. One Raspberru Pi controls 
power to all eight RxDoC cards, and monitors currents and voltages. The
other Raspberry Pi sends pointing commands as needed, to all eight
tiles via the RxDoC cards.

  - bfpower.py - on startup, turns on all eight RxDoC cards and beamformers, 
    and loops forever, printing out voltage and current readings.
  - bfcomms.py - on startup, loops forever, pointing all eight tiles at the
    zenith, every 10 seconds.

The beamformer has two F connectors for coax cable. Both deliver 48V DC to 
power the beamformer and dipole LNA's. The beamformer transmits its summed 
RF out over the X and Y, coax cables, and also uses the centre conductors on 
the X and Y coax to send delay values to the beamformer, and to get flags and 
an 12-bit temperature value back. 

This communication uses three GPIO pins on a Raspberry Pi - TXDATA, RXDATA, 
and TXCLOCK. To send a new delay configuration, the clock line (on one coax) 
is pulsed 253 times, and on each rising edge, the value of the TXDATA pin (on 
the other coax) is latched. After those 253 bits are transmitted, the clock
is pulsed another 24 times, and each time, the RXDATA pin is read, to get
8 bits of flags, and a 16-bit number containing a 12-but temperature value.

Two more GPIO pins are used to control the RxDoC card - DOCPOWER is used to control
FETs on the main PCB, to turn on and off the 5V and 48V power to the RxDoC card,
and BFPOWER is sent to the RxDoC card to turn on and off 48V power over the coax
out to the beamformer.

The delay values are in units of 435 picoseconds, and legal delay values range
from 0-63. Values from 0-31 are used for pointing the beamformer, and a value
of 32 turns off the dipole at the summing junction (useful if the LNA is bad).
That means that any delay greater than 31 has the dipole turned off at the 
summing junction in the beamformer, so the lower bit values are irrelevant.

The conversion from azimuth and elevation to delay values (0-31) is done in
mwafb/tile_geometry/calc_delays(), and the conversion from delay values to 
the 253-bit value sent to the beamformer is in 
mwabf/beamformer/BFHandler._gen_bitstring().

For more information, contact Andrew Williams 
(Andrew.Williams@curtin.edu.au)