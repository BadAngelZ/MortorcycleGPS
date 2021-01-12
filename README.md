# MotorcycleGPS

GPS Speed indicator, based on SSD1283 display and ATGM336H GPS(AT3440, supposed to work as a Neo-m8n) for ESP8266 (Node MCU)
Jerome Chapelle <nospam@teranova.org>

Previous work:
v06 : works with ESP8266
v07 : GPS configuration. Allows: enhanced refresh rate enhanced, GPS+BDS+GLONASS, vehicule mode
v08 : use of checksum function
v09 : display a 130x130 logo at startup, print debug messages on screen during boot

Current release (v10):
v10 :
- clear speed when searching satellites
- resize speed font when speed over 199km/h
- first push on github

Notes:
- SoftwareSerial must be the EspSoftwareSerial (re-install it after arduino IDE updates)
- Serial is really stable at 19200, at 38400 some GPS frames are lost leading to "0 tracked satelites"
- always init LCD FIRST, before init software serial
- use of ESP8266 gives better refresh speeds (with arduino uno, display refresh rate is sloooow
  ~1 frame every 2 seconds). ESP8266 is 80Mhz, Uno is 8Mhz, and display communication depends on CPU clock.
- NEO-6M GPS is configured
- debug messages, heavy computes, leads to problem with software serial
- ATGM336H-5N-31 allows GPS+BDS positionning, 11:GPS,21:BDS,31:GPS+BDS(PCAS04,3),51:GPS+GLONASS,71:GPS+BDS+GLONASS(PCAS04,7)

Todo:
- fix white screen on cold boot (due to serial init ?)
- dont display time when time is unknown
- Automatic GPS baud rate from 9600 to NEWSPEED
- Display GPS position when 0 km/h
- functions for GPS configuration
- display init messages on screen
- deal with GPS problems on screen (failed to get fix, gps init ...)
- disable debug (serial) messages (#define)
- adapt to ESP32
- deal with multi CPU for IO extensions: SD writing, leds, switches, ...
- comments in english
- documentation
- PCB
- 3D printed case
- instructables / hackaday

