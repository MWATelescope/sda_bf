# sda_bf
Space Domain Awareness beamformer control/test code

Communicates with MWA beamformers via Raspberry Pi's installed on 
a custom PCB, connected to one or more RxDoC cards.

Files in the 'bfif' directory are for PCB's based on the MWA long-baseline
tile hardware, where a single beamformer is connected via a 'Beam
Former Interface' box (BFIF).

  - bfif_lib.py - classes to control a BFIF board and MWA beamformer.
  - bftest.py - script to exercise individual dipoles or delay lines.
  - point.py - script to turn on the BF and point it at a given alt/az.
  - tile_geometry.py - functions to calculate beamformer delays from alt/az values.

Files in the 'eda1' directory are for SDA boxes based on the 
beamformer controllers used in the original 'Engineering Development
Array' (EDA). These boxes have two Raspberry Pi's, and control eight
MWA beamformers, using eight RxDoC cards. One Raspberru Pi controls 
power to all eight RxDoC cards, and monitors currents and voltages. The
other Raspberry Pi sends pointing commands as needed, to all eight
tiles via the RxDoC cards.

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

For more information, contact Andrew Williams 
(Andrew.Williams@curtin.edu.au)