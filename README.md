# Snake robot local stiffness controller (LSC)

A centralised controller for the STM32 snake robot. A Python script (running on a capable machine such as a PC or Raspberry Pi) acts as a centralised controller, providing commands to the snake robot's tail segment (master node) over USB. 

The master node distributes commands to the respective slave nodes over CAN bus. 

Likewise, feedback (to perform the local stiffness control) from each node is sent firstly to the master node over CAN, and then to the central controller over USB. 

$ Branches

master: working, centralised LSC

centralised: centralised LSC working branch (may not be clean)

pi: working branch for incorporating Raspberry Pi for wireless centralised control
