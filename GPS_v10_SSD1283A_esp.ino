// GPS Speed indicator, based on SSD1283 display and ATGM336H GPS(AT3440, supposed to work as a Neo-m8n) for ESP8266 (Node MCU)
// Jerome Chapelle <nospam@teranova.org>
// v06 : works with ESP8266
// v07 : GPS configuration. Allows: enhanced refresh rate enhanced, GPS+BDS+GLONASS, vehicule mode
// v08 : use of checksum function
// v09 : display a 130x130 logo at startup, print debug messages on screen during boot
// v10 : clear speed when searching satellites, resize speed font when speed over 199km/h
// Notes:
// - SoftwareSerial must be the EspSoftwareSerial (re-install it after arduino IDE updates)
// - Serial is really stable at 19200, at 38400 some GPS frames are lost leading to "0 tracked satelites"
// - always init LCD FIRST, before init software serial
// - use of ESP8266 gives better refresh speeds (with arduino uno, display refresh rate is sloooow
//   ~1 frame every 2 seconds). ESP8266 is 80Mhz, Uno is 8Mhz, and display communication depends on CPU clock.
// - NEO-6M GPS is configured
// - debug messages, heavy computes, leads to problem with software serial
// - ATGM336H-5N-31 allows GPS+BDS positionning, 11:GPS,21:BDS,31:GPS+BDS(PCAS04,3),51:GPS+GLONASS,71:GPS+BDS+GLONASS(PCAS04,7)
//
// Todo:
// - fix white screen on cold boot (due to serial init ?)
// - dont display time when time is unknown
// - Automatic GPS baud rate from 9600 to NEWSPEED
// - Display GPS position when 0 km/h
// - functions for GPS configuration
// - display init messages on screen
// - deal with GPS problems on screen (failed to get fix, gps init ...)
// - disable debug (serial) messages (#define)
// - adapt to ESP32
// - deal with multi CPU for IO extensions: SD writing, leds, switches, ...
// - comments in english
// - documentation
// - PCB
// - 3D printed case
// - git / instructables / hackaday

// https://randomnerdtutorials.com/esp8266-pinout-reference-gpios/
// - use unused pins:
//   - A0 : Analog, no output
//   - D8 : IO, Boot fails if pulled HIGH


// Plusieurs bibliothèque trouvées pour "SoftwareSerial.h"
// Utilisé : /home/badangel/.arduino15/packages/esp8266/hardware/esp8266/2.7.4/libraries/SoftwareSerial
// Non utilisé : /home/badangel/Arduino/libraries/EspSoftwareSerial

// Doc AT3340 GPS:
// http://www.icofchina.com/d/file/xiazai/2016-12-05/95deb933f6db900c6a7a6c768ded8248.pdf

#include <Wire.h>
#include <NMEAGPS.h>
#include <SPI.h>

//-------------------------------------------------------------------------
//  The GPSport.h include file tries to choose a default serial port
//  for the GPS device.  If you know which serial port you want to use,
//  edit the GPSport.h file.
//#include <GPSport.h>
// Pour ESP8266: BIEN VERIFIER QUE ON UTILISE EspSoftwareSerial (à réinstaller au besoin)
#include <SoftwareSerial.h>
SoftwareSerial serialGPS(D6, D0); // RX | TX

#define OLDSPEED 9600
#define NEWSPEED 38400

#define gpsPort serialGPS
#define GPS_PORT_NAME "SoftwareSerial"
#define DEBUG_PORT Serial


//------------------------------------------------------------
// This object parses received characters
//   into the gps.fix() data structureg
static NMEAGPS  gps;


//------------------------------------------------------------
//  Define a set of GPS fix information.  It will
//  hold on to the various pieces as they are received from
//  an RMC sentence.  It can be used anywhere in your sketch.
static gps_fix  fix;

#include <LCDWIKI_GUI.h> //Core graphics library
#include <SSD1283A.h> //Hardware-specific library

// adapt the constructor parameters to your wiring for the appropriate processor conditional,
// or add a new one or adapt the catch all other default

#if (defined(TEENSYDUINO) && (TEENSYDUINO == 147))
// for Mike's Artificial Horizon
SSD1283A_GUI mylcd(/*CS=*/ 10, /*DC=*/ 15, /*RST=*/ 14, /*LED=*/ -1); //hardware spi,cs,cd,reset,led

// for my wirings used for e-paper displays:
#elif defined (ESP8266)
SSD1283A_GUI mylcd(/*CS=D8*/ SS, /*DC=D3*/ 0, /*RST=D4*/ 2, /*LED=D2*/ 4); //hardware spi,cs,cd,reset,led
#elif defined(ESP32)
SSD1283A_GUI mylcd(/*CS=5*/ SS, /*DC=*/ 17, /*RST=*/ 16, /*LED=*/ 4); //hardware spi,cs,cd,reset,led
#elif defined(_BOARD_GENERIC_STM32F103C_H_)
SSD1283A_GUI mylcd(/*CS=4*/ SS, /*DC=*/ 3, /*RST=*/ 2, /*LED=*/ 1); //hardware spi,cs,cd,reset,led
#elif defined(__AVR)
SSD1283A_GUI mylcd(/*CS=10*/ SS, /*DC=*/ 8, /*RST=*/ 9, /*LED=*/ 7); //hardware spi,cs,cd,reset,led
#else
// catch all other default
SSD1283A_GUI mylcd(/*CS=10*/ SS, /*DC=*/ 8, /*RST=*/ 9, /*LED=*/ 7); //hardware spi,cs,cd,reset,led
#endif

#define  BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

#define COLOR_BACK BLACK
#define COLOR_FORE YELLOW

// https://javl.github.io/image2cpp/
#include "suzuki_black_logo.h"

uint32_t timer;
bool screencleared = false;

int speedT = 0;

int previousSpeed = -1;
int currentSpeed = 0;
bool doDrawStatus = true;


void fill_screen_test()
{
  mylcd.Fill_Screen(BLACK);
  mylcd.Fill_Screen(RED);
  mylcd.Fill_Screen(GREEN);
  mylcd.Fill_Screen(BLUE);
  mylcd.Fill_Screen(BLACK);
}


// Adapted from: https://forum.arduino.cc/index.php?topic=582436.0
// Checksum online: https://nmeachecksum.eqth.net/
int nmea0183_checksum(char *nmea_data)
{
  int crc = 0;
  int i;

  // ignore the first $ sign,  no checksum in sentence
  for (i = 0; i < strlen(nmea_data); i ++) { // removed the - 3 because no cksum is present
    crc ^= nmea_data[i];
  }

  return crc;
}

// Give a string like: "$PCAS02,100", will return "1E"
// Checksum is computed for the whole String
String checksum(String inputString) {
  char copy[50];
  inputString.toCharArray(copy, 50);
  int value = nmea0183_checksum(copy);
  String res = String(value, HEX); //.toUpperCase();
  res.toUpperCase();
  return res;
}

// Give aCommand like: "PCAS02,100", will compute checksum
// and send "$PCAS02,100*1E" to gps connected at aGpsPort
void sendGPSCommand(String aCommand, Stream &aGpsPort) {
  String cs = checksum(aCommand);
  String c3 = "$" + aCommand + "*" + cs;
  //aGpsPort.println(F(c3));
  aGpsPort.println(c3);
  //Serial.println(c3);
}

void initScreen() {

#if defined(SSD1283A_WIKI)
  mylcd.Init_LCD();
  mylcd.Fill_Screen(COLOR_BACK);
  mylcd.Set_Rotation(1);
#else // #elif defined(SSD1283A_OPT)
  mylcd.init();
  mylcd.Fill_Screen(COLOR_BACK);
  mylcd.setRotation(1);
#endif
  //fill_screen_test();

  mylcd.Set_Text_Mode(0);
  delay(500);
  /*mylcd.Fill_Screen(RED);
    delay(500);
    mylcd.Fill_Screen(GREEN);
    delay(500);
    mylcd.Fill_Screen(BLUE);
    delay(500);
  */

  mylcd.Set_Text_Back_colour(COLOR_BACK);
  mylcd.Set_Text_colour(COLOR_FORE);

  if (false) {
    Serial.println("Display \"GPS\"");
    mylcd.Fill_Screen(COLOR_BACK);
    //mylcd.Set_Text_Back_colour(COLOR_BACK);
    mylcd.Set_Text_Size(5);
    mylcd.Set_Text_colour(RED);
    mylcd.Print_String("G", 30, 40);
    mylcd.Set_Text_colour(GREEN);
    mylcd.Print_String("P", 60, 40);
    mylcd.Set_Text_colour(BLUE);
    mylcd.Print_String("S", 90, 40);
    Serial.println("Wait 500ms ...");
    delay(500);
  } else {
    Serial.println("Display \"LOGO\"");

    mylcd.drawRGBBitmap(0, 0, mylogo, 130, 130);
    Serial.println("Wait 2000ms ...");
    delay(2000);
  }
  //mylcd.Fill_Screen(COLOR_BACK);
}

void setGpsSpeed1() {


  printDebug("Set old speed..", "");
  Serial.println(OLDSPEED);
  gpsPort.begin( OLDSPEED );

  printDebug("", "delay 1000ms ...");
  Serial.println("delay 1000ms ...");
  delay(1000);

  //Serial.println("Set refresh rate ...");
  // 500ms refresh rate:
  //gpsPort.println( F("$PCAS02,500*1A") );

  gpsPort.flush();
  //Serial.println("Wait 200ms ...");
  //delay(200);

  // 19200 vs 9600
  if (true) {
    Serial.print("Set new speed bps ");

    printDebug("Set new speed bps", "");
    if (NEWSPEED == 57600) {
      //57600bps: (marche pas ESP8266 ? ou mauvais CRC ?)
      Serial.println("57600 bps");
      printDebug("", "57600 bps");
      gpsPort.println( F("$PCAS01,4*18") );
    }

    if (NEWSPEED == 38400) {
      //38400bps:
      Serial.println("38400 bps");
      printDebug("", "38400 bps");
      gpsPort.println( F("$PCAS01,3*1F") );
    }

    if (NEWSPEED == 19200) {
      //19200bps:
      Serial.println("19200 bps");
      printDebug("", "19200 bps");
      //gpsPort.println( F("$PCAS01,2*1E") );
      sendGPSCommand(F("PCAS01,2"), gpsPort);
    }

    if (NEWSPEED == 9600) {
      //9600bps:
      Serial.println("9600 bps");
      printDebug("", "9600 bps");
      //gpsPort.println( F("$PCAS01,1*1D") );
      sendGPSCommand(F("PCAS01,1"), gpsPort);
    }

    delay(200);

    gpsPort.flush();
    gpsPort.end();
    printDebug("Wait 1000ms ...", "");
    Serial.println("Wait 1000ms ...");
    delay(1000);
    printDebug("Begin gpsPort with newspeed", "");
    Serial.print("Begin gpsPort with ");
    Serial.println(NEWSPEED);
    gpsPort.begin( NEWSPEED );
    printDebug("", "Wait 1000ms ...");
    Serial.println("Wait 1000ms ...");
    delay(1000);
  } else {
    Serial.println("Set 9600 bps ...");
    gpsPort.println( F("$PCAS01,1*1D") );
    delay(200);

    gpsPort.flush();
    gpsPort.end();
    Serial.println("Wait 1000ms ...");
    delay(1000);
    Serial.println("Begin gpsPort with 9600 ...");
    gpsPort.begin( 9600 );
    Serial.println("Wait 1000ms ...");
    delay(1000);
  }
}

int setGpsSpeed(SoftwareSerial &aGpsPort) {
  int newSpeed = NEWSPEED;

  Serial.print("Start at : ");
  Serial.println(OLDSPEED);
  gpsPort.begin( OLDSPEED );

  //gpsPort.begin(9600);
  Serial.print("Set new speed at: ");
  switch (newSpeed) {
    case 57600:
      // (dont work with ESP8266 : too fast)
      Serial.println("57600 bauds");
      sendGPSCommand(F("PCAS01,4"), gpsPort);
      break;
    case 38400:
      Serial.println("38400 bauds");
      sendGPSCommand(F("PCAS01,3"), gpsPort);
      break;
    case 19200:
      Serial.println("19200 bauds");
      sendGPSCommand(F("PCAS01,2"), gpsPort);
      break;
    case 9600:
      Serial.println("9600 bauds");
      sendGPSCommand(F("PCAS01,1"), gpsPort);
      break;
    default:
      // Default: 9600 bauds
      Serial.println("9600 bauds");
      sendGPSCommand(F("PCAS01,1"), gpsPort);
      break;

  }
  if (false) {
    Serial.print("Set new speed bauds ");

    if (NEWSPEED == 57600) {
      //57600bps: (dont work with ESP8266 : too fast)
      Serial.println("57600 bps");
      //gpsPort.println( F("$PCAS01,4*18") );
      sendGPSCommand(F("PCAS01,4"), gpsPort);
    }

    if (NEWSPEED == 38400) {
      //38400bps:
      Serial.println("38400 bps");
      //gpsPort.println( F("$PCAS01,3*1F") );
      sendGPSCommand(F("PCAS01,3"), gpsPort);
    }

    if (NEWSPEED == 19200) {
      //19200bps:
      Serial.println("19200 bps");
      //gpsPort.println( F("$PCAS01,2*1E") );
      sendGPSCommand(F("PCAS01,2"), gpsPort);
    }

    if (NEWSPEED == 9600) {
      //9600bps:
      Serial.println("9600 bps");
      //gpsPort.println( F("$PCAS01,1*1D") );
      sendGPSCommand(F("PCAS01,1"), gpsPort);
    }
  }
  delay(200);

  gpsPort.flush();
  gpsPort.end();

  Serial.println("Wait 1000ms ...");
  delay(1000);
  Serial.print("Begin gpsPort with ");
  Serial.println(NEWSPEED);
  gpsPort.begin( NEWSPEED );
  Serial.println("Wait 1000ms ...");
  delay(1000);
}



void setup() {

  Serial.begin(115200);

  Serial.println("Init screen ...");
  initScreen();

  Serial.print("Begin GPS, set Speed: ");
  //setGpsSpeed(gpsPort);
  setGpsSpeed1();

  printDebug("delay 1000ms ...", "");

  Serial.println("delay 1000ms ...");
  delay(1000);

  //Serial.println("Set refresh rate ...");
  // 500ms refresh rate:
  //gpsPort.println( F("$PCAS02,500*1A") );

  gpsPort.flush();
  //Serial.println("Wait 200ms ...");
  //delay(200);

  // 19200 vs 9600


  printDebug("Set refresh rate ...", "");
  Serial.println("Set refresh rate ...");
  // min 100ms
  //gpsPort.println( F("$PCAS02,100*1E") );
  sendGPSCommand(F("PCAS02,500"), gpsPort);
  //sendGPSCommand(F("PCAS02,100"), gpsPort);
  // 250ms refresh rate:
  //gpsPort.println( F("$PCAS02,250*18") );
  // 500ms refresh rate:
  //gpsPort.println( F("$PCAS02,500*1A") );
  //gpsPort.println( F("$PCAS02,1000*2E") );


  printDebug("Set GPS+BDS ...", "");
  Serial.println("Set GPS+BDS ...");
  //gpsPort.println( F("$PCAS04,3*1A") );
  sendGPSCommand(F("PCAS04,3"), gpsPort);

  //Serial.println("Set GPS+BDS ...");
  //gpsPort.println( F("$PCAS04,3*1A") );


  printDebug("Set Vehicule mode (3) ...", "");
  Serial.println("Set Vehicule mode (3) ...");
  sendGPSCommand(F("PCAS11,3"), gpsPort);
  //printDebug("Set Default mode (0) ...", "");
  //Serial.println("Set Default mode (0) ...");
  //sendGPSCommand(F("PCAS11,0"), gpsPort);

  gpsPort.flush();

  printDebug("", "Wait 200ms ...");
  Serial.println("Wait 200ms ...");
  delay(200);

  timer = millis();

  printDebug("End of setup.", "");
  Serial.println("End of setup.");
  mylcd.Fill_Screen(COLOR_BACK);
  //Serial.println(checksum("PCAS02,100"));

}


//----------------------------------------------------------------
//  This function gets called about once per second, during the GPS
//  quiet time.

static void doSomeWork()
{
  timer = millis(); // reset the timer

  currentSpeed = fix.speed_kph();
  //currentSpeed = 200;
  /*speedT=speedT+5;
    if (speedT>220) {
    speedT=0;
    }
    currentSpeed=speedT;
  */
  if (currentSpeed != previousSpeed) {
    //mylcd.Led_control(true);
    previousSpeed = currentSpeed;
    char speedchar[10];
    //dtostrf(fix.speed_kph(), 3, 0, speedchar);
    dtostrf(currentSpeed, 3, 0, speedchar);
    int posx=0;
    if (currentSpeed > 199) {
      posx=10;
      mylcd.Set_Text_Size(7);
    } else {
      posx=-5;
      mylcd.Set_Text_Size(8);
    }
    mylcd.Set_Text_colour(COLOR_FORE);
    mylcd.Print_String(speedchar, posx, 20);

    mylcd.Set_Text_Size(1);
    mylcd.Set_Text_colour(COLOR_FORE);

    /*
      mylcd.Print_String("N:", 0, 80);
      char latchar[10]; // Buffer big enough for 9-character float
      dtostrf(fix.latitude(), 3, 7, latchar); // Leave room for large numbers
      mylcd.Print_String(latchar,12,80);

      mylcd.Print_String("E:", 0, 90);
      char longchar[10];
      dtostrf(fix.longitude(), 3, 7, longchar);
      mylcd.Print_String(longchar,12,90);


      mylcd.Print_String("A:", 0, 100);
      char altchar[10];
      dtostrf(fix.altitude(), 3, 7, altchar);
      mylcd.Print_String(altchar,12,90);
    */
    //fix.heading()

  } else {
    //mylcd.Led_control(false);
  }

  /*
    char precchar[10];
    //sprintf(precchar, "%.1f/%.1f/%.1f", fix.lat_err(),fix.lon_err(),fix.alt_err());
    //float prec=(fix.lat_err()+fix.lon_err()+fix.alt_err())/3.0;
    //int prec=fix.lat_err_cm+fix.lon_err_cm+fix.alt_err_cm;
    int prec=fix.pdop;
    dtostrf(prec, 3, 1, precchar);
    u8x8.drawString(4, 5, precchar);
    u8x8.drawString(14, 5 , "m.");
  */

}

static void showStatus()
{
  timer = millis(); // reset the timer

  //----------------------------------------------------------------
  //  This section is run before a fix is made to show sat info (Available, Tracked, Time)

  // Count how satellites are being received for each GNSS
  int totalSatellites, trackedSatellites;
  totalSatellites = gps.sat_count;
  for (uint8_t i = 0; i < totalSatellites; i++) {
    if (gps.satellites[i].tracked) {
      trackedSatellites++;
    }
  }


  enum {BufSizeTracked = 3}; //Space for 2 characters + NULL
  char trackedchar[BufSizeTracked];
  snprintf (trackedchar, BufSizeTracked, "%d", trackedSatellites);

  enum {BufSizeTotal = 3};
  char availchar[BufSizeTotal];
  snprintf (availchar, BufSizeTotal, "%d", totalSatellites);

  enum {BufSizeTime = 3};
  char hourchar[BufSizeTime];
  char minutechar[BufSizeTime];


  if (fix.valid.time) {
    // See: EU_DST in NMEAGPS.h for timezone
    int hour = fix.dateTime.hours + 1;
    int minute = fix.dateTime.minutes;

    snprintf (hourchar, BufSizeTime, "%d", hour);
    snprintf (minutechar, BufSizeTime, "%d", minute);
    if ( hour < 10 )
    {
      snprintf (hourchar, BufSizeTime, "%02d", hour);
    }
    if ( minute < 10 )
    {
      snprintf (minutechar, BufSizeTime, "%02d", minute);
    }

  }

  if (doDrawStatus) {
    mylcd.Set_Text_Size(1);
    mylcd.Set_Text_colour(COLOR_FORE);
    mylcd.Print_String("TRACKED", 0, 110);
    mylcd.Print_String("AVAIL", 50, 110);
    mylcd.Print_String("TIME", 90, 110);

    mylcd.Print_String("  ", 0, 120);
    mylcd.Print_String(trackedchar, 0, 120);

    mylcd.Print_String("  ", 60, 120);
    mylcd.Print_String(availchar, 60, 120);
    mylcd.Print_String(hourchar, 90, 120);
    mylcd.Print_String(":", 102, 120);
    mylcd.Print_String(minutechar, 107, 120);

    Serial.print("Tracked: ");
    Serial.print(trackedchar);
    Serial.print(" / Avail: ");
    Serial.print(availchar);
    Serial.println();
  }

}

void printDebug(String line1, String line2) {
  mylcd.Print_String(line1, 0, 110);
  mylcd.Print_String(line2, 0, 120);
}

int searchState = 0;

//  This is the main GPS parsing loop.
static void GPSloop()
{
  while (gps.available( gpsPort )) {
    fix = gps.read();
    Serial.print("p:");
    Serial.print(gps.sat_count);
    showStatus();
    if (fix.valid.location) {
      Serial.print( fix.latitude(), 6 );
      Serial.print( ',' );
      Serial.print( fix.longitude(), 6 );
      Serial.println();
      doSomeWork();
    } else {
      currentSpeed = -1;
      if (currentSpeed != previousSpeed) {
        //mylcd.Led_control(true);
        previousSpeed = currentSpeed;

        mylcd.Set_Text_Size(8);
        mylcd.Set_Text_colour(COLOR_FORE);
        mylcd.Print_String("   ", -5, 20);
      }
      mylcd.Set_Text_Size(8);
      mylcd.Set_Text_colour(COLOR_FORE);
      switch (searchState) {
        case 0:
          mylcd.Print_String("!", 45, 20);
          break;
        case 1:
          mylcd.Print_String("/", 45, 20);
          break;
        case 2:
          mylcd.Print_String("-", 45, 20);
          break;
        case 3:
          mylcd.Print_String("\\", 45, 20);
          break;
        default:
          mylcd.Print_String("!", 45, 20);
          break;
      }

      searchState = searchState + 1;
      if (searchState >= 4) {
        searchState = 0;
      }
    }
    //DEBUG Serial.println("Do some work");

  }
}

void loop()
{

  if (gpsPort.available() > 0) {
    //Serial.println("a");
    GPSloop();
    //delay(10);

    //Serial.println("no fix");
  }
  /*else {
    //Serial.println("No data");

    mylcd.Set_Text_Size(7);
    mylcd.Set_Text_colour(COLOR_FORE);
    mylcd.Print_String("?", 60, 40);
    }*/
  while (false) {
    if (true) {
      int incomingByte = 0;
      if (gpsPort.available() > 0) {
        if (gps.available( gpsPort )) {
          Serial.println();
          Serial.print("=>");
          fix = gps.read();
          if (fix.valid.location) {
            Serial.print( fix.latitude(), 6 );
            Serial.print( ',' );
            Serial.print( fix.longitude(), 6 );
            Serial.println("-------");
          }
        }

        // read the incoming byte:
        incomingByte = gpsPort.read();

        // say what you got:
        //Serial.print("I received: ");
        Serial.print((char)incomingByte);
        //Serial.print(incomingByte,DEC);
        //Serial.println("-");
        delay(10);

        //if (incomingByte==10) { Serial.println("-------"); }

      }

    }
  }


  //gpsPort.end();
  //gpsPort.begin(19200);

  //GPSloop();
  //Serial.print(".");
  //delay(500);


  // until we get a fix, print a dot every second
  /*if (millis() - timer > 2000 && !screencleared) {
    timer = millis(); // reset the timer
    // TODO
    //u8x8.print(".");
    //Serial.print(".");
    }*/
}
