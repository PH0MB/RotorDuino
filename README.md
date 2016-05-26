# RotorDuino
Arduino XY rotor automation using Yaesu GS-232B command set and an old CCTV Dennard camera rotor.

This project is a combination of an Arduino and some hardware interfacing for controlling your rotor position from your PC.
Software like Ham Radio Deluxe directly supports this kind of rotor control trough the Yaesu protocol.

I used an old Dennard CCTV Camera PAN/TILT rotor using synchonous motors. These motors and the calculated gearing in there gives 
an exact relative position  by counting the pulses of the AC supply on the motors.
The arduino measures the X and Y displacement just counting the pulses and calculating the azimuth and elevation degrees.
It implements the serial command language for the Yaesu rotor interface and displays the data on a regular 16x2 LCD display.
A separate arduino shield or intface was designed for running the 24 AC motors and counting the pulses using a opto coupler and opto solid state relays.

Future enhancements:
- Static encoder position feedback, no rotor initialisation needed any more
- stepper motor option
- servo option

Regards and 73's


Marcel PH0MB


