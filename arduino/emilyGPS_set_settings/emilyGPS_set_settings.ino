/** @file
 * Send commands to the GPS to set the sample rate and baud rate
 * Connections: Arduino pin 8 (RX) to Logic Level Shifter to GPS TX
 *              Arduino pin 9 (TX) to Logic Level Shifter to GPS RX
 *              Logic Level Shifter HV to Arduino VCC
 *              Logic Level Shifter LV to 3.3V Regulated supply for GPS
 *              Common ground on Logic Level, Arduino, GPS
 */
#include <SoftwareSerial.h>
#include "emilyStatus.h"
#include "emilyGPS.h"

SoftwareSerial gpsSerial(8, 9); // RX, TX (TX not used)
#define READ_RATE_MILLIS 100
#define PRINT_RAW false
#define GPS_BAUD_RATE 38400
#define GPS_DEFAULT_BAUD_RATE 9600

emilyStatus stat;
emilyGPS GPS(&stat);

uint32_t millis_next = 0,
millis_now = 0;
float x,y;

void configure_gps(){
  uint8_t buffer[256];
  int len;
  gpsSerial.begin(GPS_DEFAULT_BAUD_RATE);
  // set GPS baud rate to 38400 baud, see docs of send_command_configure_serial_port
  len = GPS.send_command_configure_serial_port(buffer,3);
  buffer[len] = 0;
  for(int j = 0;j<len;j++){
    gpsSerial.write(buffer[j]);
    delay(0.05);
  }
  delay(1000);
  //gpsSerial.end();
  //delay(1000);
  //reconnect to GPS at the faster baud rate
  gpsSerial.begin(GPS_BAUD_RATE);
  delay(5000);
  // pack GPS message for faster sampling - 10 Hz target  
  len = GPS.send_command_configure_position_rate(buffer,10);
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
  uint8_t buffer[256];
  
  Serial.begin(9600);
  Serial.print("Initialized emilyGPS_test\n");
  configure_gps();
  while (gpsSerial.available()){
    uint8_t ch = gpsSerial.read();
    if ( ch == 0xA0 )
      Serial.print("**************************************\n");
    Serial.print(ch,HEX);
    Serial.print(",");
  }
  Serial.print("\n");
}

void loop()
{
  millis_now = millis();
  if (gpsSerial.available())
  {
    char ch = gpsSerial.read();
    GPS.parseBytes(ch);
    if(PRINT_RAW)
      Serial.print(ch);
  }
  
  if (stat.gpsNow.is_new()){
    stat.gpsNow.get(&x,&y);
    Serial.print("*********************\n");
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
