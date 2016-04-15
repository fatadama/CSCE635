#ifndef __EMILYSTATUS_H_DEFINED__
#define __EMILYSTATUS_H_DEFINED__

#include <stdint.h>

/** Constant for converting GPS (degrees) to position in meters.
 * @brief equals degrees2radians factor (x) earth radius (meters) 6378100*pi/180*1e-7
 */
#define GPSDATA_D2R_RE 0.0111318845
/** Direct control mode - we are sending offboard commands for the rudder and throttle */
#define CONTROL_MODE_DIRECT 0
/** Direct control mode - we are sending heading and speed commands and do PID control locally */
#define CONTROL_MODE_INDIRECT 1
/** passive control mode - initialized, no commands received */
#define CONTROL_MODE_PASSIVE 2
/** conversion factor knots to meters/s */
#define KNOTS2MS 0.514444
/** conversion factor degrees to radians */
#define DEG2RAD 0.0174533

#define GPS_HOME_LAT_DEFAULT 306200000
#define GPS_HOME_LON_DEFAULT -963400000

double deg2m(int32_t);/*!< Convert a double in (10^-7 degrees) to arc length in meters */

/** Enumated type for the communication health.
 * Meanings: HEALTHY less than 1 second since the last message from the master
 *           WARNING more than 1 second but less than 10 seconds since the last message
 *           LOST more than 10 seconds since the last message
 */
enum commStatus{
  COMM_STATUS_HEALTHY,
  COMM_STATUS_WARNING,
  COMM_STATUS_LOST
};

/*! Class for holding GPS data */
class gpsData{
public:
  gpsData();
  int32_t lon; /*!< Longitude (10^-7 degrees east) */
  int32_t lat; /*!< Latitutde (10^-7 degrees north) */
  double t; /*!< Time (sec) */
  double x; /*!< X = north (meters) data in north-east earth frame. Defined relative to a fixed point near USA for better precision. */
  double y; /*!< Y = east (meters) data in north-east earth frame. Defined relative to a fixed point near USA for better precision. */
  double v;/*!< velocity in meters/sec */
  double hdg;/*!< heading in radians */
  bool init; /*!< Status indicator so we can tell if the value of a GPS object has been set */
  /** Set the value of GPS data object and compute x-y
   *
   * @param[in] lati latitutde reading in DEGREES
   * @param[in] loni longitude reading in DEGREES
   * @param[in] ti time from GPS in seconds
   */
  void set(double lati,double longi,double ti);
  /** Set the value of GPS data object, compute X-Y position, and set velocity and heading
   *
   * @param[in] lati latitutde reading in DEGREES
   * @param[in] loni longitude reading in DEGREES
   * @param[in] ti time from GPS in seconds
   * @param[in] v speed from GPS in knots
   * @param[in] hdg "course made good" from GPS in degrees
   */
   void set(int32_t lati, int32_t longi, double ti, double vi, double hdgi);
  int8_t is_new();/*<! Return the current value of 'new' */
  /** Set the value of GPS data object and compute x-y.
   *
   * We use 10^-7 degrees as the units for compatibility with Arduino, which lacks double floating-point precision.
   *
   * @param[in] lati latitutde reading in units of (10^-7 degrees)
   * @param[in] loni longitude reading in units of (10^-7 degrees)
   * @param[in] ti time from GPS in seconds
   */
  void set(int32_t lati,int32_t longi,double ti); /*!< Set the value of GPS data object and compute x-y. Uses long inputs to be compatible with comm protocol */
  void set_home(int32_t lat1,int32_t lon1); /*!< Change from default home lat/lon coordinates to new ones. Units are (10^-7 degrees) */
  void get(float*x,float*y);/*!< Return the current position state for control purposes */
private:
  int32_t lat_home;
  int32_t lon_home;
  int8_t new_value;/*!< Return 0 if the current state has already been accessed with get(). Set to 1 whenever set() is called. */
};

class emilyStatus
{
public:
  emilyStatus();
  gpsData gpsNow;
  gpsData gpsCmd;
  double control_rudder;
  double control_throttle;
  double command_heading;
  double command_speed;
  double Kp[2];/*!< proportional gains for the rudder (channel 0) and throttle (channel 1) */
  double Ki[2];/*!< integral gains for the rudder (channel 0) and throttle (channel 1) */
  double Kd[2];/*!< derivative gains for the rudder (channel 0) and throttle (channel 1) */
  uint8_t control_mode;
  commStatus comm_status;
private:
};

#endif
