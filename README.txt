The libraries provide MTP interface to onboard microSD card in a Teensy 3.5/3.6 board via a connected host PC.

To test it, please install Teensyduino version 1.39.  Do the following steps:

1.  Clone SdFat libraries from https://github.com/greiman/SdFat into your sketchbook libraries folder
2.  Clone thie library https://github.com/greiman/MTP into your sketchbook libraries folder
3.  Copy the files boards.txt into <Arduino's installation directory>/hardware/teensy/avr/
4.  Copy the files usb_desc.h into <Arduino's installation directory>/hardware/teensy/avr/cores/teensy3/
5.  Insert a FAT/FAT32 formatted microSD card into the on-board microSD card slot
6.  Open the examples/MTP_blinky/MTP_blinky.ino program
7.  Select USB Type "MTP Disk (Experimental)"
8.  Compile and download the binary into Teensy 3.5/3.6 boards
9.  Open serial monitor to see the trace

See the history in https://forum.pjrc.com/threads/43050-MTP-Responder-Contribution?p=138772
