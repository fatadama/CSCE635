#include <cstdio>
#include "emilyStatus.h"

int main(){
  gpsData gps;
  printf("%f,%f,%f,%f,%f\n",gps.lat,gps.lon,gps.t,gps.x,gps.y);
  gps.set(45.0,-167.83434234,90.238239);
  printf("%f,%f,%f,%f,%f\n",gps.lat,gps.lon,gps.t,gps.x,gps.y);
  gps.set(long(-450000000),long(1678343423),90.238239);
  printf("%f,%f,%f,%f,%f\n",gps.lat,gps.lon,gps.t,gps.x,gps.y);

  return 0;
}
