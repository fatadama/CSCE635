/** @file
 * Try reading from GPS using servoTimer2 library instead of servo
 * Connections: Arduino pin 8 (RX) to Logic Level Shifter to GPS TX
 *              Arduino pin 7 (TX) to Logic Level Shifter to GPS RX
 *              Logic Level Shifter HV to Arduino VCC
 *              Logic Level Shifter LV to 3.3V Regulated supply for GPS
 *              Common ground on Logic Level, Arduino, GPS
 */
//#include <Arduino.h>
#include <SoftwareSerial.h>
#include "emilyStatus.h"
#include "emilyGPS.h"

/** serial object for reading GPS */
SoftwareSerial gpsSerial(8, 7); // RX, TX pins
#define GPS_BAUD_RATE 38400
#define GPS_DEFAULT_BAUD_RATE 9600
/** The GPS sample rate in Hz. Valid options: [1,2,4,5,10,20]. We seem to be having problems at 10 */
#define GPS_UPDATE_RATE 2
/** period in microseconds at which to check the serial port */
#define SERIAL_PERIOD_MICROS 10000
/** set to true to enable debug output on serial port */
#define DEBUGGING true
/** XBee baud rate */
#define XBEE_SERIAL_BAUD 9600
/** Debugging serial baud rate */
#define DEBUG_SERIAL_BAUD 9600

/** global status object */
emilyStatus stat;
/** GPS object */
emilyGPS GPS;

uint32_t millis_next = 0;
uint32_t millis_now = 0;
uint32_t serial_millis_next = 0;
// Variable for parsing XBee bytes
uint8_t serialByte;
/** GPS parsing char */
char gpsChar;

// DEBUGGING VARIABLES
float x,y;

/** Configure the GPS: Send binary commands for a faster baud rate and sample rate
 *
 *  Note: if the baud rate is ever changed from the default, you have to power down the GPS to apply a new baud rate, as the program always tries to connect at the default (9600) rate before changing the baud rate.
 *  @param[in] baud: Target baud rate. 0 = 4800 baud, 1 = 9600, 2 = 19200, 3 = 38400, 4 = 57600, 5 = 115200
 *  @param[in] sample: Target sample rate in Hz. Allowed values are [1,2,4,5,10,20]. 4,5,10 require a baud rate of 38400 or higher. 20 requires a baud rate of 57600?
 */
void configure_gps(int baud, int sample){
  uint8_t buffer[256];
  int len;
  gpsSerial.begin(GPS_DEFAULT_BAUD_RATE);
  // set GPS baud rate to 38400 baud, see docs of send_command_configure_serial_port
  len = GPS.send_command_configure_serial_port(buffer,baud);
  buffer[len] = 0;
  for(int j = 0;j<len;j++){
    gpsSerial.write(buffer[j]);
    delay(0.05);
  }
  delay(1000);
  //reconnect to GPS at the faster baud rate
  gpsSerial.begin(GPS_BAUD_RATE);
  delay(5000);
  //pack GPS message to send only RMC GPS messages
  len = GPS.send_command_configure_nmea_message(buffer,0,0,0,0,1,0,0);//should the value be zero??
  buffer[len] = 0;
  // write one byte at a time
  for(int j = 0;j<len;j++){
    gpsSerial.write(buffer[j]);
    delay(0.05);
  }
  delay(1000);
  // pack GPS message for faster sampling - 10 Hz target  
  len = GPS.send_command_configure_position_rate(buffer,sample);
  buffer[len] = 0;
  // write one byte at a time
  for(int j = 0;j<len;j++){
    gpsSerial.write(buffer[j]);
    delay(0.05);
  }
  delay(1000);
}

void setup()
{
  if(DEBUGGING){
    Serial.begin(DEBUG_SERIAL_BAUD);
    Serial.print("Initialized emilyGPS_test\n");
  }
  // set the GPS baud rate and sample rate
  switch (GPS_BAUD_RATE){
    case 38400:
      configure_gps(3,GPS_UPDATE_RATE);
      break;
    case 57600:
      configure_gps(4,GPS_UPDATE_RATE);
      break;
    case 115200:
      configure_gps(5,GPS_UPDATE_RATE);
      break;
  }
  while (gpsSerial.available()){
    uint8_t ch = gpsSerial.read();
    if(DEBUGGING){
      if ( ch == 0xA0 )
        Serial.print("**************************************\n");
      Serial.print(ch,HEX);
      Serial.print(",");
    }
  }
  if(DEBUGGING){
    Serial.print("\n");
  }
  // set target time for reading serial
  serial_millis_next = millis() + (SERIAL_PERIOD_MICROS/1000);
}

void loop()
{
  millis_now = millis();
  /** See if new GPS available */
  if (gpsSerial.available())
  {
    gpsChar = gpsSerial.read();
    if(DEBUGGING)
      Serial.print(gpsChar);
    GPS.parseBytes(gpsChar,millis_now);
  }
  // call periodic functions
  GPS.misc_tasks(millis_now);
  GPS.sync(&stat);

  // test GPS print
  if(DEBUGGING) {
    if (stat.gpsNow.is_new()){
      stat.gpsNow.get(&x,&y);
      Serial.print("GPS Time:");
      Serial.print(stat.gpsNow.t);
      Serial.print(" Lat:");
      Serial.print(stat.gpsNow.lat);
      Serial.print(" Lon:");
      Serial.print(stat.gpsNow.lon);
      Serial.print(" V:");
      Serial.print(stat.gpsNow.v);
      Serial.print(" H:");
      Serial.print(stat.gpsNow.hdg);
      Serial.print(" X:");
      Serial.print(x);
      Serial.print(" Y:");
      Serial.print(y);
      Serial.print(" NEWCMD:");
      Serial.print(stat.gpsCmd.is_new());
      Serial.print("\n");
    }
  }
}
