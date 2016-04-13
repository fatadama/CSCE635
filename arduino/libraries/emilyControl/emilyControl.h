#ifndef __EMILYCONTROL_H_DEFINED__
#define __EMILYCONTROL_H_DEFINED__

#include <stdint.h>
#include "emilyStatus.h"
#include "controlPid.h"

/** Target automatic control rate execution in Hz */
#define EMILYCONTROL_RATE_MILLIS 100

/** Scale a floating point 'val between 'low' and 'high' to the PWM output range of [0,255] */
uint8_t scale_pwm(float val,float low, float high);

/*! Control class. Handles the mode logic and either passses through control values in DIRECT mode, sets zeros in PASSIVE mode, or computes onboard in INDIRECT mode */
class emilyControl{
public:
  /** Constructor
   *
   * @param[in] st pointer to the global status object
   */
  emilyControl(emilyStatus*st);
  /** Compute periodic tasks
   *
   * Write out control value, or compute new value & write if fly-by-wire.
   * @param[in] millis the current system time in milliseconds
   */
  void misc_tasks(uint32_t millis);
  /** Return the PWM values to apply for the rudder and throttle.
   *
   * @param[out] pwm_rudder rudder PWM expressed in [0,255].
   * @param[out] pwm_throttle throttle PWM expressed in [0,255]
   */
  void get_pwm(uint8_t* pwm_rudder,uint8_t* pwm_throttle);
  uint8_t new_control();/*<! Return 1 if the control value has been updated */
private:
  emilyStatus* status; /*<! Pointer to the global status object */
  float rudder;
  float throttle;
  uint8_t new_value;
  uint32_t millis_last;/*<! Mark the time of last computation for automatic control (INDIRECT mode) */
  controlPid rudder_pid;
  controlPid throttle_pid;
};

#endif
