#ifndef __EMILYCONTROL_H_DEFINED__
#define __EMILYCONTROL_H_DEFINED__

#include <stdint.h>
#include "emilyStatus.h"
#include "../controlPid/controlPid.h"
#include "../lowPassFilter/lowPassFilter.h"

/** Target automatic control rate execution in Hz */
#define EMILYCONTROL_RATE_MILLIS 100

/** Scale a floating point 'val' between 'low' and 'high' to the PWM output range of [1000,2000]
 *
 * @detail Background: Linear mapping such that b*y + c = x, such that:
                                                low*b + c = 1000
                                                high*b + c = 2000
                                            Solution:
                                                b = 1000/(high-low)
                                                c = (2000*low-1000*high)/(low-high)
                                            x = ((low-val)*2000+(val-high)*1000)/(low-high)

 */
uint16_t scale_pwm(float val,float low, float high);
/** Specific mapping for the throttle channel, which needs to be mapped from [1500,2000]
 *
 * x = ((low-val)*2000+(val-high)*1500)/(low-high)
 */
uint16_t scale_pwm_throttle(float val, float low, float high);

/*! Control class. Handles the mode logic and either passes through control values in DIRECT mode, sets zeros in PASSIVE mode, or computes onboard in INDIRECT mode */
class emilyControl{
public:
  /** Constructor
   *
   * Initializes the PID objects to zero
   * Initializes the lowpass objects for the commanded rudder and throttle to a filter constant of 0.1.
   */
  emilyControl();
  /** Compute periodic tasks
   *
   * Set the control value, or compute new value & write if INDIRECT mode.
   * @param[in] millis the current system time in milliseconds
   * @param[in] status COPY of the global status object - we need the PWM values from ground and the control mode
   */
  void misc_tasks(uint32_t millis,emilyStatus status);
  /** Return the PWM values to apply for the rudder and throttle.
   *
   * @param[out] pwm_rudder rudder PWM expressed in [1000,2000] ms.
   * @param[out] pwm_throttle throttle PWM expressed in [1000,2000] ms.
   */
  void get_pwm(uint16_t* pwm_rudder,uint16_t* pwm_throttle);
  uint8_t new_control();/*<! Return 1 if the control value has been updated */
private:
  lowPassFilter rudder;/*<! Lowpass filter object: always low pass the rudder */
  lowPassFilter throttle;/*<! Lowpass filter object: always low pass the throttle */
  uint8_t new_value;
  uint32_t millis_last;/*<! Mark the time of last computation for automatic control (INDIRECT mode) */
  controlPid rudder_pid;
  controlPid throttle_pid;
};

#endif
