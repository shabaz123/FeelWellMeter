# FeelWellMeter
Room Air Quality Monitor

This code implements an room air quality monitor using an Arduino Micro, a Sensirion Environmental Sensor Shield, a 100uA moving-coil meter and a few ancillary parts
(4 x LEDs, a push-button, a 47 kohm resistor and 4 x 220 ohm resistors).

Usage: 
Open the code using the Arduino development environment, and select Arduino Micro (click Tools->Board->Board Manager if it is not already available)

Go to Sketch->Include Library->Manage Libraries and type Sensirion in the search box, and select "arduino-ess"

Plug in the USB cable into the PC and the Arduino Micro, and build and transfer the code to it.

Once the circuit is assembled (see the schematic file), the project can be powered up via the USB connection. If the button is held down before powering up, then a calibration routine is run.
Otherwise, the code will automatically cycle through displaying temperature, humidity, VOC and CO2 measurements. Press the button at any time to manually select any of these to be monitored indefinitely until the button is pressed again.
Keep pressing the button until an LED flickers, to go back to the automatic cycling mode.


