#ifndef __CONTROLPID__H__DEFINED__
#define __CONTROLPID__H__DEFINED__

#include <stdint.h>
#include "lowPassFilter.h"

#define INTEGRAL_MAX 0.1

class controlPid{
public:
  controlPid(double Kp,double Ki, double Kd);
  controlPid();/*!< Default constructor */
  void update(double xnew,uint32_t millis);/*<! Receive a new state; update the control, given the clock time in milliseconds */
  void update(double xnew,double ref, uint32_t millis);/*!< Receive a new state AND target state; update the control, given clock time in milliseconds */
  double get_control();
  double get_state();/*<! Get the current value of (x-r): the state minus the reference */
  void set_integral_max(double val);
  void set_Kp(double);/*!< Set the proportional gain */
  void set_Ki(double);/*!< Set the integral gain */
  void set_Kd(double);/*!< Set the derivative gain */
  void set_x0(double);/*!< Set the initial state. Reset the initialization variable to zero. */
private:
  double Kp;
  double Ki;
  double Kd;
  double integral_max;
  double f_term; /*!< integral term */
  double u;
  lowPassFilter r;/*!< The most recent commanded value */
  lowPassFilter x;/*!< The most recent sensed value */
  uint32_t millis_last;/*!< Last update time */
  int8_t init;
};

#endif
