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
/** period in microseconds at which to check the serial port */
#define SERIAL_PERIOD_MICROS 10000
/** set to true to enable debug output on serial port */
#define DEBUGGING true

/** serial port name to use: Serial is USB, Serial1 is the RX and TX pins */
#define COMM_SERIAL Serial1
/** rudder signal pin output */
#define RUDDER_PIN 5
/** throttle signal pin output */
#define THROTTLE_PIN 6

emilyStatus stat;
emilyStatus commstatus;
/** GPS object */
emilyGPS GPS;
/** communications parser object */
//commParser comm(&stat);
commParser comm(&commstatus);
/** Control object */
emilyControl control;
//emilyControl control(&controlstatus);

uint32_t millis_next = 0;
uint32_t millis_now = 0;
uint32_t serial_millis_next = 0;
// Variable for parsing XBee bytes
uint8_t serialByte;
/** Rudder signal variable */
uint8_t pwm_rudder;
/** Throttle signal variable */
uint8_t pwm_throttle;

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
    Serial.begin(9600);
    Serial.print("Initialized emilyGPS_test\n");
  }
  // open XBee serial port
  COMM_SERIAL.begin(9600);
  // set the GPS baud rate and sample rate
  configure_gps(3,10);
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
  /** See if new GPS available */
  if (gpsSerial.available())
  {
    char ch = gpsSerial.read();
    GPS.parseBytes(ch);
  }
  // call periodic functions
  GPS.misc_tasks();
  GPS.sync(&stat);
  comm.misc_tasks(millis_now);
  control.misc_tasks(millis_now,stat);

  // read the control values and write them
  // TODO: analogWrite seems to be causing problems. Replace with a servo() object
  if(control.new_control() > 0){
    // read from control
    control.get_pwm(&pwm_rudder,&pwm_throttle);
    if(DEBUGGING){
      Serial.print("*********************\n");
      Serial.print("RUDDER: ");
      Serial.print(pwm_rudder);
      Serial.print("THROTTLE: ");
      Serial.print(pwm_throttle);
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
      Serial.print(" Long: ");
      Serial.print(stat.gpsNow.lon);
      Serial.print(" X: ");
      Serial.print(x);
      Serial.print(" Y: ");
      Serial.print(y);
      Serial.print("\n");
    }
  }
}
