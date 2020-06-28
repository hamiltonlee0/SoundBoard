# SoundBoard

### Summary:
 Teensy 3.6 based Soundboard with 2 recordable sound effects, 11 programmable sound effects, and 1 programmable FIR filter. The final code for the project is located in the file sound_board. All other files are code to test different hardware components to this projects. Most of the coding is done on Arduino IDE as to utilize the nicely developed sound libraries from Teensy (especially the I2S drivers). Filter design was preliminarily done on jupyter notebook (python) using a rememz exchange algorithm, but the final version was done using a filter design website using TFilter (http://t-filter.engineerjs.com/) as to generate integer taps instead of float taps. Only FIR filters up to 200 taps are supported.

### Components:

* Teensy 3.6 development board: https://www.sparkfun.com/products/14057
* I2S MEMS Microphone (SPH0645LM4H): https://www.adafruit.com/product/3421
* I2S Audio Amplifier (Class D - 3W): https://www.adafruit.com/product/3006
* 3W 4 Ohm Speaker: https://www.adafruit.com/product/1314
* RGB Matrix-Scan Buttonpad (4x4): https://learn.sparkfun.com/tutorials/button-pad-hookup-guide (Multiple components for the buttonpad listed in this link)
* Various resistors, LEDs, potentiometers, diodes, etc...

### System Diagram/Schematic

