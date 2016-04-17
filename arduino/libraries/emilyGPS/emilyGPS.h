#ifndef __EMILYGPS_H_DEFINED__
#define __EMILYGPS_H_DEFINED__

#include <stdint.h>
#include "emilyStatus.h"

/** MAximum NMEA string sentence length */
const int sentenceSize = 80;
/** @file */

class emilyGPS{
public:
  /** Constructor
   *
   *
   */
  emilyGPS();
  int16_t parseBytes(char ch);/** Parse a sequence of raw bytes from serial port. Return values: -1 == got a non-PMC message, 0 == no message, 1 == not a PMC message */
  int16_t parseSentence();/** Parse a sentence from the GPS */
  void misc_tasks();/** Placeholder for miscellaneous tasks */
  /** Send the command to set the baud rate. Returns a string
   *
   * @param[out] buffer variable in which to put the command
   * @param[in] baud_var set the baud rate according to the binary command set. 0 = 4800 baud, 1 = 9600, 2 = 19200, 3 = 38400, 4 = 57600, 5 = 115200
   * @param[out] the number of bytes written
   */
  uint8_t send_command_configure_serial_port(uint8_t*buffer,uint8_t baud_var);
  /** Send the command to set the target update rate.
   *
   * @param[out] buffer variable in which to put the command
   * @param[in] rate The rate in Hz. [1,2,4,5,8,10,20] are supported
   * @param[out] the number of bytes written
   */
  uint8_t send_command_configure_position_rate(uint8_t*buffer,uint8_t rate);
  /** Send command to perform a cold restart */
  uint8_t send_command_restart_cold(uint8_t*buffer);
  /** Return new GPS data */
  void sync(emilyStatus*st);
private:
  char sentence[sentenceSize];/** Buffer that holds incoming strings */
  emilyStatus* status; /** Pointer to the global status object */
  int16_t parseCounter; /** Counts the position within the buffer */
  void getField(char* buffer, int index);/** Read a field from sentence */
  uint8_t compute_checksum(uint8_t*buffer,uint8_t len);/** Compute the checksum for Venus638FLPx binary commands */
  gpsData gpsNow;/*!< GPS object to store new values in */
};

/** Convert a GPS character field for longitude or latitude in standard format to a int32_t format in (10^-7 degrees)
 * Standard format is: xxxyy.zz, where xxx is whole degrees, and yy.zz is minutes. Maximum percent error is about 0.0001 ... in degrees
 */
int32_t convertGPS(char* buffer);
/** Convert GPS time field to a UTC time in seconds (INT) since midnight. Rolls over at midnight UTC! */
int32_t convertTime(char*buffer);
/** Convert GPS time field to a UTC time in seconds (FLOAT) since midnight. Rolls over at midnight UTC! */
float convertTimef(char*buffer);

#endif
