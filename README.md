Selfbalancing-robot
===================

Selfbalancing robot in Arduino. Implemented with PID controllers, FIR filters, complementary filter.
More info at http://sebastiannilsson.com/k/projekt/selfbalancing-robot/

Folders:
selfbalancingrobot is the main code for the balancing robot. You can use this without the console if you want. 
Just comment everything with I2C communication to the console.

selfbalancingrobot_console is the code for the console. 
Uses a nokia display and some tatile buttons for on-the-fly configuration of the robot. 
Much faster than uploading new code with different parameters.


Libraries:
FIR filter library at https://github.com/sebnil/FIR-filter-Arduino-Library

FreeIMU library at http://www.varesano.net/projects/hardware/FreeIMU

PID library at https://github.com/br3ttb/Arduino-PID-Library

Button library at https://github.com/JChristensen/Button

Adafruit_GFX library (for the console) at https://github.com/adafruit/Adafruit-GFX-Library

Adafruit_PCD8544 library (for the console) at https://github.com/adafruit/Adafruit-PCD8544-Nokia-5110-LCD-library