# Selfbalancing-robot

Selfbalancing robot in Arduino. Implemented with PID controllers, FIR filters, complementary filter.
More info at http://sebastiannilsson.com/k/projekt/selfbalancing-robot/

## Video
<a href="http://www.youtube.com/watch?feature=player_embedded&v=DmoShkJclLI
" target="_blank"><img src="http://img.youtube.com/vi/DmoShkJclLI/0.jpg" 
alt="Selfbalancing robot video 1" width="240" height="180" border="10" /></a>

<a href="http://www.youtube.com/watch?feature=player_embedded&v=QH83A746gS0
" target="_blank"><img src="http://img.youtube.com/vi/QH83A746gS0/0.jpg" 
alt="Selfbalancing robot video 2" width="240" height="180" border="10" /></a>

## Getting started
  * selfbalancingrobot is the main code for the balancing robot. 
    * You can use this without the console if you want. Just comment everything with I2C communication to the console.
  * selfbalancingrobot_console is the code for the console. 
  * Uses a nokia display and some tatile buttons for on-the-fly configuration of the robot. 
    * Much faster than uploading new code with different parameters.


# Libraries:
  * FIR filter library at https://github.com/sebnil/FIR-filter-Arduino-Library
  * FreeIMU library at http://www.varesano.net/projects/hardware/FreeIMU
  * PID library at https://github.com/br3ttb/Arduino-PID-Library
  * Button library at https://github.com/JChristensen/Button
  * Adafruit_GFX library (for the console) at https://github.com/adafruit/Adafruit-GFX-Library
  * Adafruit_PCD8544 library (for the console) at https://github.com/adafruit/Adafruit-PCD8544-Nokia-5110-LCD-library


   
## Support my creation of open source software:
[![Flattr this git repo](http://api.flattr.com/button/flattr-badge-large.png)](https://flattr.com/submit/auto?user_id=sebnil&url=https://github.com/sebnil/Selfbalancing-robot)

<a href='https://ko-fi.com/A0A2HYRH' target='_blank'><img height='36' style='border:0px;height:36px;' src='https://az743702.vo.msecnd.net/cdn/kofi2.png?v=0' border='0' alt='Buy Me a Coffee at ko-fi.com' /></a>