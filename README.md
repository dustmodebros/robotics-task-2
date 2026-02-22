# robotics-task-2
A repository to consolidate code for the "The Mark 4" group for the SEMTM0042-43 (Robotics) unit for Bristol University

## Contributing (read this first)
I don't want to fork over $4 per person per month to have branch protection, so rules are self enforced. For new features, *PLEASE PLEASE PLEASE* make a new branch from `dev`, and work on that branch, then PR into `dev` so any changes can be reviewed. Also please make sure that **any code that is pushed compiles**. If I find a handy arduino test library, I might write some automated tests, and they'll get a workflow. Broken code in `dev` will result in sadness, and broken code in `main` will result in floggings.

I cba to enforce any silly commit message requirements, so go nuts as long as they mean *something* relevant to your code.

## Receive and Transmit
To use the contents of this repo, first of all clone it, and open each of the transmit and receive .ino files in separate arduino windows.
Then remove the chassis-s from two Pololu 3pi+ robots, and upload the code respectively to one and the other. You'll need batteries in the transmitter, and your receiver communicating over Serial.
You should see the test bit of 0xFC transmitted and received, and you can see debug info by changing the DEBUG_STATES, DEBUG_OUTPUTS (Serial monitor) and DEBUG_INPUTS (Serial plotter).

Happy RX/TXing!

## How it works
The transmission and reception works based on the principle of pulse-duration decoding. A pulse of a long duration encodes a 1, and a short duration encodes a 0. There's also a longer sync pulse so that the recipient can tell when a byte begins and ends.

Right now the timings are as follows:
SYNC_PULSE_MS   100 (signals a byte is about to be sent)
SHORT_PULSE_MS   30 (0)
LONG_PULSE_MS    60 (1)

## Notes
Going for a smaller timescale to up the bitrate seems to run into problems where we struggle to read the pulses due to the latency on reading the IR sensor, as well as delays caused by execution on the microcontroller.
Unfortunately this limits us to a somewhat piddly ~1.4ish Bytes/sec. Additionally, going slower seems to make the connection less fiddly and precise.
Experimentation is encouraged! It's probably possible to make the sync pulse like 20ms shorter without compromising accuracy... give it a try!
