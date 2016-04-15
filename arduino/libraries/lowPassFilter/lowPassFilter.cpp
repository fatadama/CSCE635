#include <stdint.h>
#include "lowPassFilter.h"

lowPassFilter::lowPassFilter(){
  alpha = 0.1;
  x = 0.0;
  xlast = 0.0;
}

void lowPassFilter::set_alpha(float a){
  alpha = a;
}

float lowPassFilter::update(float x_input){
  xlast = x;
  x = alpha*x_input + (1.0-alpha)*x;
  return x;
}

float lowPassFilter::get_derivative(float dt){
  // check for dt = 0
  if (dt == 0.0)
    return 0.0;
  else
    return (x-xlast)/(dt);
}

float lowPassFilter::get_x(){
  return x;
}

void lowPassFilter::set_x(float x0){
  x = x0;
}
