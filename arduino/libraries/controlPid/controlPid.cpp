#include <stdint.h>
#include "controlPid.h"

controlPid::controlPid(){
  controlPid(0.0,0.0,0.0);
}

controlPid::controlPid(double Kpi,double Kii,double Kdi){
  Kp = Kpi;
  Ki = Kii;
  Kd = Kdi;
  init = 0;
  integral_max = INTEGRAL_MAX;
  millis_last = 0;
}

void controlPid::update(double xnew,uint32_t millis){
  // call the main update function with zero reference
  update(xnew,0.0,millis);
}

void controlPid::update(double xnew,double ref,uint32_t millis){
  if (!init){
    init = 1;
    millis_last = millis;
  }
  // get the time increment
  float dt = 1e-3*(millis - millis_last);//seconds
  // filter the input
  x.update(xnew);
  // filter the reference
  r.update(ref);
  // increment the integral term
  f_term += (x.get_x()-r.get_x())*dt;
  // bound the integral term
  if (f_term > integral_max)
    f_term = integral_max;
  if (f_term < -integral_max)
    f_term = -integral_max;
  // compute control
  u = -Kp*(x.get_x()-r.get_x())-Kd*(x.get_derivative(dt)-r.get_derivative(dt))-Ki*f_term;
  // store new time
  millis_last = millis;
}

double controlPid::get_control(){
  return u;
}

double controlPid::get_state(){
  return (x.get_x()-r.get_x());
}

void controlPid::set_integral_max(double val){
  integral_max = val;
}

void controlPid::set_Kp(double Kpi){
  Kp = Kpi;
}

void controlPid::set_Ki(double Kii){
  Ki = Kii;
}

void controlPid::set_Kd(double Kdi){
  Kd = Kdi;
}

void controlPid::set_x0(double x0){
  x.set_x(x0);
  init = 0;
}
