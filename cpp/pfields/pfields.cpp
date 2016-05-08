#include<math.h>
#include<iostream>
#include "pfields.h"


void pfields(int profile, double goal_loc_x, double goal_loc_y, double X, double Y, double parameters[], int MP, double* fout)
{
	using namespace std;
	double D, C, VIRTUAL_DISTANCE, ORIENTATION, D_TAN, VMAG, U, V;
	const double pi = 3.1415926535897;
	D = parameters[0];
	C = parameters[1];
	VIRTUAL_DISTANCE = parameters[2];
	ORIENTATION = parameters[3];
	D_TAN = parameters[4];

	// CALCULATING RIGHT ANGLE & EQUATION(TANGENTIAL)
	double VGX, VGY, M, B;
	VGX = goal_loc_x + VIRTUAL_DISTANCE*cos(ORIENTATION + 3*pi/2);
	VGY = goal_loc_y + VIRTUAL_DISTANCE*sin(ORIENTATION + 3*pi/2);
	M = (goal_loc_y - VGY) / (goal_loc_x - VGX);
	B = VGY - VGX*M;

	// SPECIAL: MIRROR TANGENT
	double MS, BS;
	MS = tan(ORIENTATION);
	BS = VGY - VGX*MS;

	// DISTANCE & ANGLE
	double Q, THETA;
	Q = sqrt(pow((VGX - X), 2) + pow((VGY - Y), 2));
	THETA = atan2((VGY - Y), (VGX - X));

	// SPECIAL CASE MP
	double MINT, BINT, X_CROSS, Y_CROSS, Q_CROSS;
	MINT = -MS;
	BINT = Y - MINT*X;

	X_CROSS = (BINT - BS) / (MS - MINT);
	Y_CROSS = MS*X_CROSS + BS;

	Q_CROSS = sqrt(pow((X - X_CROSS), 2) + pow((Y - Y_CROSS), 2));

	// MAGNITUTDE PROFILE
	switch (MP)
	{
	case 1:
		VMAG = C; // CONSTANT
		break;
	case 2:
		VMAG = C*(D - Q) / D; // LINEAR DROPOFF
		break;
	case 3:
		VMAG = C*pow((D - Q) / D, 2); // EXPONENTIAL
		break;
	case 4:
		// EXPONENTIAL DECREASE
		if (Q / D_TAN >= 1) {
			VMAG = 8;
		}
		else {
			VMAG = 8 * (Q / D_TAN);
		}
		break;
	case 5:
		if (pow((Q_CROSS / Q), 2) < 0.3) {
			VMAG = C*Q_CROSS / Q;
		}
		else {
			VMAG = C*Q;
		}
		break;
	case 6:
		if (Q_CROSS > 0 && Q > 0 && pow((Q_CROSS / Q), 2) < 0.3) {
			VMAG = C*Q / Q_CROSS;
		}
		else {
			VMAG = C;
		}
		if (Q == 0 || Q_CROSS == 0) {
			VMAG = C;
		}
		break;
	case 7:
		if (pow((Q_CROSS / Q), 2) < 0.3) {
			VMAG = C*Q_CROSS / Q;
		}
		else {
			VMAG = C*pow(((D - Q) / D), 2);
		}
		break;
	case 8:
		if (Q_CROSS > 0 && Q > 0 && pow((Q_CROSS / Q), 2) < 0.3) {
			VMAG = C*pow(((D - Q_CROSS) / D), 2);
		}
		else {
			VMAG = C;
		}
		if (Q == 0 || Q_CROSS == 0) {
			VMAG = C;
		}
		break;
	}

	// COMPUTING PFIELDS
	switch (profile)
	{
	case 1:
		if (Q <= D) {
			if (Y >= MS*X + BS) {
				if (Y < M*X + B) {
					U = VMAG*cos(THETA - pi / 2);
					V = VMAG*sin(THETA - pi / 2);
				}
				else {
					U = VMAG*cos(THETA + pi / 2);
					V = VMAG*sin(THETA + pi / 2);
				}
			}
			else if (Y < MS*X + BS) {
				if (Y > M*X + B) {
					U = VMAG*cos(THETA - pi / 2);
					V = VMAG*sin(THETA - pi / 2);
				}
				else {
					U = VMAG*cos(THETA + pi / 2);
					V = VMAG*sin(THETA + pi / 2);
				}
			}
		}
		else {
			U = 0;
			V = 0;
		}
		break;
	case 2:
		if (Q <= D) {
			U = VMAG*cos(THETA);
			V = VMAG*sin(THETA);
		}
		else {
			U = 0;
			V = 0;
		}
		break;
	}

	// cout << "D = " << D << endl;
	// cout << "C = " << C << endl;
	// cout << "VD = " << VIRTUAL_DISTANCE << endl;
	// cout << "ORIENTATION = " << ORIENTATION << endl;
	// cout << "XG = " << goal_loc_x << endl;
	// cout << "YG = " << goal_loc_y << endl;
	// cout << "XC = " << X << endl;
	// cout << "YC = " << Y << endl;
	// cout << "THETA = " << THETA << endl;
	// cout << "VMAG = " << VMAG << endl;
	// cout << "MP = " << MP << endl;
	// cout << "fx = " << U << endl;
	// cout << "fy = " << V << endl;
	// cout << "profile = " << profile << endl;

	// set the output variables
	fout[0] = U;
	fout[1] = V;
}

void robot_control(double PFIELDS[], double goal_loc_x, double goal_loc_y, double X, double Y, double emily_heading, double pfield_parameters[], double * output)
{
	using namespace std;
	// SETTING VARIABLES
	double D_TAN, C_TAN, D_ATT, C_ATT, D_BALL, C_BALL, VIRTUAL_DISTANCE, ORIENTATION, XG, YG;
	const double pi = 3.1415926535897;
	D_TAN = pfield_parameters[0]; // INFLUENCE REGION OF TANGENTIAL FIELD
	C_TAN = pfield_parameters[1]; // CONSTANT OF TANGENTIAL FIELD
	D_ATT = pfield_parameters[2]; // INFLUENCE REGION OF ATTRACTIVE FIELD
	C_ATT = pfield_parameters[3]; // CONSTANT OF ATTRACTIVE FIELD
	D_BALL = pfield_parameters[4]; // INFLUENCE REGION OF BALLISTIC FIELD
	C_BALL = pfield_parameters[5]; // CONSTANT OF BALLISTIC FIELD
	VIRTUAL_DISTANCE = pfield_parameters[6]; // VIRTUAL DISTANCE AWAY FROM GOAL
	ORIENTATION = pfield_parameters[7]; // ORIENTATION (FINAL POSE OF EMILY)

	int MP_TAN = 7; // MAGNITUDE PROFILE OF TANGENTIAL FIELD
	int MP_ATT = 8; // MAGNITUDE PROFILE OF ATTRACTIVE FIELD
	//int MP_ATT = 1; // MAGNITUDE PROFILE OF ATTRACTIVE FIELD
	int MP_BALL = 4; // MAGNITUDE PROFILE OF BALLISTIC FIELD
	
	XG = goal_loc_x; // X COORDINATE OF GOAL POSITION
	YG = goal_loc_y; // Y COORDINATE OF GOAL POSITION

	double TAN_PARS[5] = { D_TAN, C_TAN, VIRTUAL_DISTANCE, ORIENTATION, D_TAN };
	double ATT_PARS[5] = { D_ATT, C_ATT, VIRTUAL_DISTANCE, ORIENTATION, D_TAN };
	double ATT_BALL[5] = { D_BALL, C_BALL, VIRTUAL_DISTANCE, ORIENTATION, D_TAN };

	double VGX = XG + VIRTUAL_DISTANCE*cos(ORIENTATION + 3*pi/2);
	double VGY = YG + VIRTUAL_DISTANCE*sin(ORIENTATION + 3*pi/2);

	// ROBOT PATH LOOP

	// CURRENT DISTANCE AWAY
	double Q = sqrt(pow((VGX - X),2) + pow((VGY - Y),2));

	double fout[2] = { 0.0, 0.0 };
	double fx, fy;
	double FAX, FAY, FTX, FTY, FABX, FABY;

	// GOAL PFIELDS
	if (PFIELDS[0] == 1 && PFIELDS[1] == 0 && PFIELDS[2] == 0)
	{
		pfields(1,goal_loc_x,goal_loc_y, X, Y, TAN_PARS, MP_TAN, fout);
		FTX = fout[0];
		FTY = fout[1];
		fx = FTX;
		fy = FTY;
	}
				
	else if (PFIELDS[0] == 1 && PFIELDS[1] == 1 && PFIELDS[2] == 1)
	{
		pfields(1, goal_loc_x, goal_loc_y, X, Y, TAN_PARS, MP_TAN, fout);
		FTX = fout[0];
		FTY = fout[1];
		pfields(2, goal_loc_x, goal_loc_y, X, Y, ATT_PARS, MP_ATT, fout);
		FAX = fout[0];
		FAY = fout[1];
		fx = FTX + FAX;
		fy = FTY + FAY;
	}
	else if (PFIELDS[0] == 1 && PFIELDS[1] == 1 && PFIELDS[2] == 2)
	{
		pfields(1, goal_loc_x, goal_loc_y, X, Y, TAN_PARS, MP_TAN, fout);
		FTX = fout[0];
		FTY = fout[1];
		pfields(2, goal_loc_x, goal_loc_y, X, Y, ATT_BALL, MP_BALL, fout);
		FABX = fout[0];
		FABY = fout[1];
		fx = FTX + FABX;
		fy = FTY + FABY;
	}
	else if (PFIELDS[0] == 1 && PFIELDS[1] == 2 && PFIELDS[2] == 0)
	{
		pfields(1, goal_loc_x, goal_loc_y, X, Y, TAN_PARS, MP_TAN, fout);
		FTX = fout[0];
		FTY = fout[1];
		pfields(2, goal_loc_x, goal_loc_y, X, Y, ATT_BALL, MP_BALL, fout);
		FABX = fout[0];
		FABY = fout[1];
		pfields(2, goal_loc_x, goal_loc_y, X, Y, ATT_PARS, MP_ATT, fout);
		FAX = fout[0];
		FAY = fout[1];
		fx = FTX + FABX + FAX;
		fy = FTY + FABY + FAY;
	}
	else if (PFIELDS[0] == 0 && PFIELDS[1] == 1 && PFIELDS[2] == 1)
	{
		pfields(2, goal_loc_x, goal_loc_y, X, Y, ATT_PARS, MP_ATT, fout);
		FAX = fout[0];
		FAY = fout[1];
		fx = FAX;
		fy = FAY;
	}
	else if (PFIELDS[0] == 0 && PFIELDS[1] == 1 && PFIELDS[2] == 2)
	{
		pfields(2, goal_loc_x, goal_loc_y, X, Y, ATT_BALL, MP_BALL, fout);
		FABX = fout[0];
		FABY = fout[1];
		fx = FABX;
		fy = FABY;
	}

	// VELOCITY & ANGLE
	double V, TH;
	V = sqrt(pow(fx,2) + pow(fy,2))/8;
	TH = atan2(fy, fx);
	if (V > 1)
	{
		V = 1;
	}
	else
	{
		V = V;
	}
	
	while (emily_heading - TH > pi)
	{
		emily_heading = emily_heading - 2 * pi;
	}
	while (emily_heading - TH < -pi)
	{
		emily_heading = emily_heading + 2 * pi;
	}

	//HACK
	if (Q < 10.0) {
		V = V*Q*0.1;
	}

	//double EHC = emily_heading;

	// set the output
	output[0] = V;
	output[1] = -(emily_heading - TH);
}
