# sda_bf
Space Domain Awareness beamformer control/test code

Communicates with MWA beamformers via a Raspberry Pi installed on 
a custom PCB, connected to one or more RxDoC cards.

Files in the 'bfif' directory are for PCB's based on the MWA long-baseline
tile hardware, where a single beamformer is connected via a 'Beam
Former Interface' box (BFIF).

  - bfif_lib.py - classes to control a BFIF board and MWA beamformer
  - bftest.py - script to exercise individual dipoles or delay lines
  - point.py - script to turn on the BF and point it at a given alt/az.

Files in the 'eda1' directory are for SDA boxes based on the 
beamformer controllers used in the original 'Engineering Development
Array' (EDA). These boxes have two Raspberry Pi's, and control eight
MWA beamformers, using eight RxDoC cards. One Raspberru Pi controls 
power to all eight RxDoC cards, and monitors currents and voltages. The
other Raspberry Pi sends pointing commands as needed, to all eight
tiles via the RxDoC cards.