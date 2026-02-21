# robotics-task-2
A repository to consolidate code for the "The Mark 4" group for the SEMTM0042-43 (Robotics) unit for Bristol University

## Receive and Transmit
To use the contents of this repo, first of all clone it, and open each of the transmit and receive .ino files in separate arduino windows.
Then remove the chassis-s from two Pololu 3pi+ robots, and upload the code respectively to one and the other. You'll need batteries in the transmitter, and your receiver communicating over Serial.
You should see the test bit of 0xFC transmitted and received, and you can see debug info by changing the DEBUG_STATES, DEBUG_OUTPUTS (Serial monitor) and DEBUG_INPUTS (Serial plotter).

Happy RX/TXing!
