/** @file
 *  Send and receive servo commands only. No GPS.
 *  Connections: Arduino RX to XBee Dout
 *               Arduino TX to XBee Din
 *               Arduino pin 9 to rudder signal
 *               Arduino pin 10 to throttle signal
 *               Arduino VCC and GND to XBee explorer VIN and GND
 */

 #include <Arduino.h>
#include <SoftwareSerial.h>
#include <Servo.h>
#include "emilyStatus.h"
#include "commParser.h"
#include "emilyControl.h"

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
#define RUDDER_PIN 9
/** throttle signal pin output */
#define THROTTLE_PIN 10
/** Rudder servo object */
Servo rudderServo;
/** Throttle servo object */
Servo throttleServo;

/** global status object */
emilyStatus stat;
/** communications parser object */
commParser comm;
/** Control object */
emilyControl control;

enum servoMode{
  SERVO_LIBRARY,
  ANALOG_WRITE,
  SERVO2
};

servoMode SERVO_MODE = SERVO_LIBRARY;

uint32_t millis_next = 0;
uint32_t millis_now = 0;
uint32_t serial_millis_next = 0;
// Variable for parsing XBee bytes
uint8_t serialByte;
/** Rudder signal variable */
uint16_t pwm_rudder;
/** Throttle signal variable */
uint16_t pwm_throttle;

// DEBUGGING VARIABLES
float x,y;

void setup_servos(){
  if(SERVO_MODE == SERVO_LIBRARY){
    // initialize servo pins out
    rudderServo.attach(RUDDER_PIN);
    throttleServo.attach(THROTTLE_PIN);  
  }
  if (SERVO_MODE == ANALOG_WRITE){
    pinMode(RUDDER_PIN,OUTPUT);
    pinMode(THROTTLE_PIN,OUTPUT);
  }
}

void write_servos(uint16_t pwm_rudder, uint16_t pwm_throttle){
  switch(SERVO_MODE){
    case SERVO_LIBRARY:
      // write out rudder
      rudderServo.writeMicroseconds(pwm_rudder);
      // write out throttle
      throttleServo.writeMicroseconds(pwm_throttle);
      break;
    case ANALOG_WRITE:
      analogWrite(RUDDER_PIN, uint8_t((pwm_rudder-1000)*256/1000) );
      analogWrite(THROTTLE_PIN, uint8_t((pwm_throttle-1000)*256/1000) );
      break;
  }
}

void setup()
{
  if(DEBUGGING){
    Serial.begin(DEBUG_SERIAL_BAUD);
    Serial.print("Initialized emilyGPS_test\n");
  }
  // open XBee serial port
  COMM_SERIAL.begin(XBEE_SERIAL_BAUD);
  // set target time for reading serial
  serial_millis_next = millis() + (SERIAL_PERIOD_MICROS/1000);
  // call servo setup function
  setup_servos();
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
  // update comm status
  comm.misc_tasks(millis_now,stat);
  comm.sync(&stat);
  // copy comm status to main status
  control.misc_tasks(millis_now,stat);

  // read the control values and write them
  if(control.new_control() > 0){
    // read from control
    control.get_pwm(&pwm_rudder,&pwm_throttle);
    if(DEBUGGING){
      Serial.print("CTRL R:");
      Serial.print(pwm_rudder);
      Serial.print(" T:");
      Serial.print(pwm_throttle);
      Serial.print(" M:");
      Serial.print(stat.control_mode);
      Serial.print(" S:");
      Serial.print(stat.comm_status);
      Serial.print("\n");
    }
    // write out rudder
    write_servos(pwm_rudder,pwm_throttle);
  }

  // send any bytes in the transmit buffer
  while(comm.bytes_to_send() > 0){
    COMM_SERIAL.write( comm.get_next_byte() );
  }
  
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
    if (stat.gpsCmd.is_new()){
      stat.gpsCmd.get(&x,&y);
      Serial.print("GPSCMD ");
      Serial.print("Lat:");
      Serial.print(stat.gpsCmd.lat);
      Serial.print(" Lon:");
      Serial.print(stat.gpsCmd.lon);
      Serial.print(" X:");
      Serial.print(x);
      Serial.print(" Y:");
      Serial.print(y);
      Serial.print("\n");
    }
  }
}
