#ifndef __LOWPASSFILTER_H_DEFINED__
#define __LOWPASSFILTER_H_DEFINED__

#include <stdint.h>

/*! Low pass filter class. Accepts a single value and updates a new value as x_new = self.alpha*x_input + (1.0-self.alpha)*x_old */
class lowPassFilter{
public:
  lowPassFilter();/*< Class constructor. Initializes alpha to 0.1 and x to zero. */
  void set_alpha(float a);/*< Set the filter constant. Values of 0.1-0.2 are common. HIGHER values mean more SLUGGISH response of x_old */
  float update(float x_input);/*!< Compute the new output and return it */
  float get_x();/*!< Return the current value of the state */
  float get_derivative(float dt);/*!< Return the derivative of the current state. @param[in] dt The time since the last call. */
  void set_x(float x0);/*!< Set the state value */
private:
  float x; /*!< The internal state */
  float xlast; /*!< The previous internal state, used for derivative */
  float alpha;/*!< The filter constant. Should be between 0 (always return old value) and 1 (always return newest value, unfiltered. */
};
#endif
