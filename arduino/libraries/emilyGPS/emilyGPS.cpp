#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "emilyGPS.h"

emilyGPS::emilyGPS(){
  parseCounter = 0;
  time_last_millis = 0;
}

int16_t emilyGPS::parseBytes(char ch,uint32_t millis){
  if (ch != '\n' && parseCounter < sentenceSize){
    sentence[parseCounter] = ch;
    parseCounter++;
  }
  else
  {
    sentence[parseCounter] = '\0';
    parseCounter = 0;
    // run parser function
    int16_t outVal = parseSentence();
    if (outVal > 0)
      time_last_millis = millis;// set the time
    return outVal;
  }
  return 0;
}

int16_t emilyGPS::parseSentence(){
  char field[sentenceSize];
  getField(field, 0);
  if (strcmp(field, "$GPRMC") == 0)
  {
    // check if this is a valid message
    getField(field,2);
    if (strcmp(field,"A") != 0){
      return -2;
    }
    // read time
    getField(field,1);
    float timei = convertTimef(field); // time in seconds
    // read lat
    getField(field, 3);  // number in 0.01 * degrees
    int32_t lat = convertGPS(field);
    getField(field, 4); // N/S
    if (strcmp(field,"S")==0){
      lat = -lat;
    }
    // read lon
    getField(field, 5);  // number in 0.01 * degrees
    int32_t lon = convertGPS(field);
    getField(field, 6);  // E/W
    if (strcmp(field,"W")==0){
      lon = -lon;
    }
    //gpsNow.set(lat,lon,timei);
    // read the speed in knots
    getField(field,7);
    float v = atof(field);
    // read the heading in "course made good"
    getField(field,8);
    float hdg = atof(field);
    // if gpsNow is initialized, set gpsLast to gpsNow
    if (gpsNow.init){
      gpsLast.lon = gpsNow.lon;
      gpsLast.lat = gpsNow.lat;
      gpsLast.t = gpsNow.t;
      gpsLast.x = gpsNow.x;
      gpsLast.y = gpsNow.y;
      gpsLast.v = gpsNow.v;
      gpsLast.hdg = gpsNow.hdg;
    }
    // do outlier rejection
    if ( acceptGps(&gpsNow,&gpsLast)){}
      // set the status object including speed and heading
      gpsNow.set(lat,lon,timei,v,hdg);
    }
    // set the time
    return 1;
  }
  else{
    return -1;
  }
}

void emilyGPS::getField(char* buffer, int index)
{
  int sentencePos = 0;
  int fieldPos = 0;
  int commaCount = 0;
  while (sentencePos < sentenceSize)
  {
    if (sentence[sentencePos] == ',')
    {
      commaCount ++;
      sentencePos ++;
    }
    if (commaCount == index)
    {
      buffer[fieldPos] = sentence[sentencePos];
      fieldPos ++;
    }
    sentencePos ++;
  }
  buffer[fieldPos] = '\0';
}

void emilyGPS::misc_tasks(uint32_t millis){
  if (millis - time_last_millis > GPS_TIMEOUT_PERIOD_MS){
    // set a status member of the gpsNow class object
    gpsNow.health = GPS_STATUS_LOST;
  }
  else{
    gpsNow.health = GPS_STATUS_HEALTHY;
  }
}

uint8_t emilyGPS::compute_checksum(uint8_t*buffer,uint8_t len){
  uint8_t cs = 0;
  for (int j = 0;j<len;j++){
    cs = cs ^ buffer[j];
  }
  return cs;
}

uint8_t emilyGPS::send_command_configure_serial_port(uint8_t*buffer,uint8_t baud){
  //header bytes
  buffer[0] = 0xA0;buffer[1] = 0xA1;
  buffer[2] = 0x00;// payload length byte 1 (MSB)
  buffer[3] = 0x04;// payload length byte 2 (LSB)
  buffer[4] = 0x05;// message ID
  buffer[5] = 0x00;// COMM PORT ID: 0 is Com1
  buffer[6] = baud;
  buffer[7] = 0;// 0 = update to SRAM, 1 = update to SRAM and FLASH
  buffer[8] = compute_checksum(&buffer[4],4);//checksum
  // end of message bytes
  buffer[9] = 0x0D; buffer[10] = 0x0A;
  return 11;
}

uint8_t emilyGPS::send_command_configure_position_rate(uint8_t*buffer,uint8_t rate){
  //header bytes
  buffer[0] = 0xA0;buffer[1] = 0xA1;
  buffer[2] = 0x00;// payload length byte 1 (MSB)
  buffer[3] = 0x03;// payload length byte 2 (LSB)
  buffer[4] = 0x0E;// message ID
  buffer[5] = rate;// rate in Hz
  buffer[6] = 0;// 0 = update to SRAM, 1 = update to SRAM and FLASH
  buffer[7] = compute_checksum(&buffer[4],3);//checksum
  // end of message bytes
  buffer[8] = 0x0D; buffer[9] = 0x0A;
  return 10;
}

uint8_t emilyGPS::send_command_configure_nmea_message(uint8_t*buffer,uint8_t gga,uint8_t gsa, uint8_t gsv, uint8_t gll, uint8_t rmc, uint8_t vtg, uint8_t zda){
  //header bytes
  buffer[0] = 0xA0;buffer[1] = 0xA1;
  buffer[2] = 0x00;// payload length byte 1 (MSB)
  buffer[3] = 0x09;// payload length byte 2 (LSB)
  buffer[4] = 0x08;// message ID
  buffer[5] = gga;
  buffer[6] = gsa;
  buffer[7] = gsv;
  buffer[8] = gll;
  buffer[9] = rmc;
  buffer[10] = vtg;
  buffer[11] = zda;
  buffer[12] = 0;// 0 = update to SRAM, 1 = update to SRAM and FLASH
  buffer[13] = compute_checksum(&buffer[4],9);
  buffer[14] = 0x0D; buffer[15] = 0x0A;
  return 16;
}

uint8_t emilyGPS::send_command_restart_cold(uint8_t*buffer){
  //header bytes
  buffer[0] = 0xA0;buffer[1] = 0xA1;
  buffer[2] = 0x00;// payload length byte 1 (MSB)
  buffer[3] = 0x0F;// payload length byte 2 (LSB)
  //message body
  buffer[4] = 0x01;// message ID
  buffer[5] = 3;// COLD restart
  buffer[6] = 0x07;//year HIGH byte
  buffer[7] = 0xD8;//year LOW byte
  buffer[8] = 0x0B;//month UTC
  buffer[9] = 0x0E;//day UTC
  buffer[10] = 0x08;//hour UTC
  buffer[11] = 0x2E;//min UTC
  buffer[12] = 0x03;//sec UTC
  buffer[13] = 0x09;//latitutde high byte
  buffer[14] = 0xC4;//latitude low byte
  buffer[15] = 0x30;//longitude high byte
  buffer[16] = 0x70;//longitude low byte
  buffer[17] = 0x00;//altitude high byte
  buffer[18] = 0x64;//altitude low byte

  buffer[19] = compute_checksum(&buffer[4],15);//checksum
  // end of message bytes
  buffer[20] = 0x0D; buffer[21] = 0x0A;
  return 22;
}

void emilyGPS::sync(emilyStatus*status){
  if (gpsNow.is_new()){
    // copy the lat/lon and other data to the global structure
    status->gpsNow.set(gpsNow.lat,gpsNow.lon,gpsNow.t,gpsNow.v,gpsNow.hdg);
    // HACK: call get() to make the gps data no longer new
    float x,y;
    gpsNow.get(&x,&y);
  }
  // set the status of the global structure
  status->gpsNow.health = gpsNow.health;
  // set the control mode to PASSIVE if GPS is bad
  if (gpsNow.health == GPS_STATUS_LOST){
    status->control_mode = CONTROL_MODE_PASSIVE;
  }
}

int32_t convertGPS(char* buffer){
  double deg = (atof(buffer)*0.01);
  double rem = deg - (int32_t(deg));
  int32_t ret = int32_t(deg)*1e7;
  ret += int32_t(1e7*(5.0*rem)/3.0);//convert 0.01*minutes to degrees -> multiply by 100, then divide by 60 minutes/deg
  return ret;
}

float convertTimef(char*buffer){
  float time = atof(buffer);
  int32_t hr = int32_t(1.0e-4*time);
  time = time - (1e4*hr);
  int32_t min = int32_t(1.0e-2*time);
  float sec = time-(1e2*min);
  return (sec + min*60 + hr*3600);
}


int32_t convertTime(char*buffer){
  int32_t time = atol(buffer);
  int32_t hr = int32_t(1.0e-4*time);
  time = time - (1e4*hr);
  int32_t min = int32_t(1.0e-2*time);
  int32_t sec = time-(1e2*min);
  return (sec + min*60 + hr*3600);
}

bool acceptGps(gpsData*g,gpsData*glast){
  // if time decreases, reject
  if(g->t < glast->t)
    return false;
  if(g->t-glast->t > 1000.0)
    return false;
  if ( (fabs(g->x-glast->x) > 40.0) || (fabs(g->y-glast->y) > 40.0) )
    return false;
  /*
  if (fabs(g->v-glast->v) > 10.0)
    return false;
  if (fabs(g->hdg-glast->hdg) > 3.14159)
    return false;
  if (fabs(g->hdg) > 6.28318)
    return false;
  if (fabs(g->v) > 100.0)
    return false;
  */
  return true;
}
