/** Uses simple Euler propagation to evaluate if the PID outputs seem to be in the ballpark.
 * Runs one case with regulation, second case with a cosine reference.
 */

#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "controlPid.h"

int main(){
  FILE * file;
  const uint32_t dt = 10;
  const double dtf = dt*1.0e-3;
  // create PID object
  controlPid pid;
  pid.set_Kp(0.25);
  pid.set_Kd(1.0);
  pid.set_Ki(0.0);
  pid.set_x0(5.0);

  // test regulation
  double x0[2] = {5.0,0.0};
  double x1[2];
  uint32_t millis = 0;
  for(int k = 0;k<500;k++){
    millis += dt;
    pid.update(x0[0],millis);
    double u = pid.get_control();
    // copy state
    x1[0] = x0[0];
    x1[1] = x0[1];
    // propagate
    x0[0] += dtf*x1[1];
    x0[1] += dtf*u;
    // print
    printf("%6.4f%12.6g%12.6g\n",(1.0e-3*millis),x0[0],x0[1]);
  }

  // test reference
  file = fopen("pidTest.csv","w");
  x0[0] = 5.0;
  x0[1] = 0.0;
  double r = 0.0,r1 = 0.0;
  millis = 0;
  pid.set_x0(x0[0]);
  pid.set_Ki(0.1);
  pid.set_integral_max(1.0);
  for(int k = 0;k<1500;k++){
    millis += dt;
    // update the reference
    double t = 1.0e-3*millis;
    r = cos(t);
    // call the control
    pid.update(x0[0],r,millis);
    double u = pid.get_control();
    // copy state
    x1[0] = x0[0];
    x1[1] = x0[1];
    // propagate
    x0[0] += dtf*x1[1];
    x0[1] += dtf*u;
    // get error from pid
    double e = pid.get_state();
    // print
    printf("%6.4f%12.6g%12.6g%12.6g\n",t,e,x0[0],r);
    fprintf(file,"%g,%g,%g,%g\n",t,e,x0[0],r);
  }
  fclose(file);

  return 0;
}
