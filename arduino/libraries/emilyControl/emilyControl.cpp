#include "emilyControl.h"
#include <stdint.h>

uint8_t scale_pwm(float val,float low, float high){
  return uint8_t(((val-low)/(high-low))*255.0);
}

lowPassFilter::lowPassFilter(){
  alpha = 0.1;
  x = 0.0;
}

void lowPassFilter::set_alpha(float a){
  alpha = a;
}

float lowPassFilter::update(float x_input){
  x = alpha*x_input + (1.0-alpha)*x;
  return x;
}

float lowPassFilter::get_x(){
  return x;
}

emilyControl::emilyControl(emilyStatus*st){
  status = st;
  //rudder = 0.0;
  //throttle = 0.0;
  new_value = 0;
  millis_last = 0;
  // initialize the PID gain
  rudder_pid.set_Kp(0.0);
  rudder_pid.set_Ki(0.0);
  rudder_pid.set_Kd(0.0);
  rudder_pid.set_integral_max(0.1);
  throttle_pid.set_Kp(0.0);
  throttle_pid.set_Ki(0.0);
  throttle_pid.set_Kd(0.0);
  throttle_pid.set_integral_max(0.1);
}

void emilyControl::misc_tasks(uint32_t millis){
  if (status->control_mode == CONTROL_MODE_PASSIVE){ // do nothing
    // write out zeros
    throttle.update(0.0);
    rudder.update(0.0);
    // FLAG that there is new data in the control object
    new_value = 1;
  }
  else{
    // check the time
    if (millis - millis_last > EMILYCONTROL_RATE_MILLIS){
      millis_last = millis;
      if (status->control_mode == CONTROL_MODE_DIRECT){ //offboard control
        // read from status object
        throttle.update(status->control_throttle);
        rudder.update(status->control_rudder);
      }
      if (status->control_mode == CONTROL_MODE_INDIRECT){ // automatic control
        // set the gains if new values
        // check if the GPS data are new
        // copy local GPS
        // perform smoothing or estimation
        // compute PID control
        // not implemented yet: write out zeros
        throttle.update(0.0);
        rudder.update(0.0);
      }
      // FLAG that there is new data in the control object
      new_value = 1;
    }
  }
}

void emilyControl::get_pwm(uint8_t* pwm_rudder,uint8_t* pwm_throttle){
  *pwm_rudder = scale_pwm(rudder.get_x(),-1.0,1.0);
  *pwm_throttle = scale_pwm(throttle.get_x(),0.0,1.0);
  // FLAG that the control value has been accessed
  new_value = 0;
}

uint8_t emilyControl::new_control(){
  return new_value;
}
