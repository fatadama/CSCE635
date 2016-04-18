/** @file
 * Send commands to the GPS to set the sample rate and baud rate
 * Connections: Arduino pin 8 (RX) to Logic Level Shifter to GPS TX
 *              Arduino pin 9 (TX) to Logic Level Shifter to GPS RX
 *              Logic Level Shifter HV to Arduino VCC
 *              Logic Level Shifter LV to 3.3V Regulated supply for GPS
 *              Common ground on Logic Level, Arduino, GPS
 */
#include <Arduino.h>
#include <SoftwareSerial.h>
#include "emilyStatus.h"
#include "emilyGPS.h"
#include "commParser.h"
#include "emilyControl.h"

/** serial object for reading GPS */
SoftwareSerial gpsSerial(8, 9); // RX, TX (TX not used)
#define GPS_BAUD_RATE 38400
#define GPS_DEFAULT_BAUD_RATE 9600
#define GPS_UPDATE_RATE 4
/** period in microseconds at which to check the serial port */
#define SERIAL_PERIOD_MICROS 10000
/** set to true to enable debug output on serial port */
#define DEBUGGING true
/** XBee baud rate */
#define XBEE_SERIAL_BAUD 9600
/** Debugging serial baud rate */
#define DEBUG_SERIAL_BAUD 9600

/** serial port name to use: Serial is USB, Serial1 is the RX and TX pins */
#define COMM_SERIAL Serial1
/** rudder signal pin output */
#define RUDDER_PIN 5
/** throttle signal pin output */
#define THROTTLE_PIN 6

/** global status object */
emilyStatus stat;
/** GPS object */
emilyGPS GPS;
/** communications parser object */
commParser comm;
/** Control object */
emilyControl control;

uint32_t millis_next = 0;
uint32_t millis_now = 0;
uint32_t serial_millis_next = 0;
// Variable for parsing XBee bytes
uint8_t serialByte;
/** Rudder signal variable */
uint8_t pwm_rudder;
/** Throttle signal variable */
uint8_t pwm_throttle;
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
  // open XBee serial port
  COMM_SERIAL.begin(XBEE_SERIAL_BAUD);
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
  // initialize servo pins out
  pinMode(RUDDER_PIN,OUTPUT);
  pinMode(THROTTLE_PIN,OUTPUT);  
}

void loop()
{
  millis_now = millis();
  // update the comm status
  // see if it's time to READ XBee serial
  if( millis_now >= serial_millis_next ){
    serial_millis_next += (SERIAL_PERIOD_MICROS/1000);
    // check if serial is available
    if (COMM_SERIAL.peek() > -1){
      while(COMM_SERIAL.available()){ // not sure if we should run as fast as available or not
        // read while available
        serialByte = COMM_SERIAL.read();
        // parse the message
        comm.newBytes(&serialByte,1,millis_now);
      }
    }
  }
  // copy comm status to main status
  comm.sync_after_receive(&stat);
  /** See if new GPS available */
  if (gpsSerial.available())
  {
    gpsChar = gpsSerial.read();
    GPS.parseBytes(gpsChar);
  }
  // call periodic functions
  GPS.misc_tasks();
  GPS.sync(&stat);
  // update comm status
  //comm.update_status(&stat);
  comm.misc_tasks(millis_now,stat);
  comm.sync(&stat);
  // copy comm status to main status
  control.misc_tasks(millis_now,stat);

  // read the control values and write them
  // TODO: analogWrite seems to be causing problems. Replace with a servo() object?
  if(control.new_control() > 0){
    // read from control
    control.get_pwm(&pwm_rudder,&pwm_throttle);
    if(DEBUGGING){
      Serial.print(" R: ");
      Serial.print(pwm_rudder);
      Serial.print(" T: ");
      Serial.print(pwm_throttle);
      Serial.print(" M: ");
      Serial.print(stat.control_mode);
      Serial.print(" S: ");
      Serial.print(stat.comm_status);
      Serial.print("\n");
    }
    /*
    // write out rudder
    analogWrite(RUDDER_PIN,pwm_rudder);
    // write out throttle
    analogWrite(THROTTLE_PIN,pwm_throttle);
    */
  }

  // send any bytes in the transmit buffer
  while(comm.bytes_to_send() > 0){
    COMM_SERIAL.write( comm.get_next_byte() );
  }
  
  // test GPS print
  if(DEBUGGING) {
    if (stat.gpsNow.is_new()){
      stat.gpsNow.get(&x,&y);
      Serial.print("Time: ");
      Serial.print(stat.gpsNow.t);
      Serial.print("Lat: ");
      Serial.print(stat.gpsNow.lat);
      Serial.print(" Lon: ");
      Serial.print(stat.gpsNow.lon);
      Serial.print(" V: ");
      Serial.print(stat.gpsNow.v);
      Serial.print(" H: ");
      Serial.print(stat.gpsNow.hdg);
      Serial.print(" X: ");
      Serial.print(x);
      Serial.print(" Y: ");
      Serial.print(y);
      Serial.print(" NEWCMD: ");
      Serial.print(stat.gpsCmd.is_new());
      Serial.print("\n");
    }
    if (stat.gpsCmd.is_new()){
      stat.gpsCmd.get(&x,&y);
      Serial.print("GPSCMD ");
      Serial.print("Lat: ");
      Serial.print(stat.gpsCmd.lat);
      Serial.print(" Lon: ");
      Serial.print(stat.gpsCmd.lon);
      Serial.print(" X: ");
      Serial.print(x);
      Serial.print(" Y: ");
      Serial.print(y);
      Serial.print("\n");
    }
  }
}
