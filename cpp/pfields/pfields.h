#ifndef PFIELDS_H
#define PFIELDS_H

/** @file Defines functions used for pfields implementation */

void pfields(int profile, double goal_loc_x, double goal_loc_y, double X, double Y, double parameters[], int MP, double* fout);
void robot_control(double PFIELDS[], double goal_loc_x, double goal_loc_y, double X, double Y, double emily_heading, double pfield_parameters[], double * output);

#endif
