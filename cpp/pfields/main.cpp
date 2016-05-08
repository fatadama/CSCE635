#include <iostream>
#include "pfields.h"
#include<math.h>
#include<fstream>

/** @file Test implementation of pfields code */

int main()
{
	using namespace std;
	const double pi = 3.1415926535897;
	double PFIELDS[3] = { 1,2,0 };
	double goal_loc_x = 15;
	double goal_loc_y = 15;
	double emily_heading;
	double D_TAN, C_TAN, D_ATT, C_ATT, D_BALL, C_BALL, VIRTUAL_DISTANCE, ORIENTATION;
	// TANGENTIAL, ATTRACTIVE & BALLISTIC PFIELDS
	D_TAN = 100; C_TAN = 1;
	D_ATT = 100; C_ATT = 2 / 7.5;
	D_BALL = 2000; C_BALL = .1 / 7.5;
	VIRTUAL_DISTANCE = 8;
	ORIENTATION = pi / 6;
	double pfield_parameters[11] = { D_TAN,C_TAN,D_ATT,C_ATT,D_BALL,C_BALL,VIRTUAL_DISTANCE,ORIENTATION };
	int MP_parameters[3] = { 7,8,4 };

	double x[249],y[249];
	ifstream File;
	File.open("C:\\Users\\jesus\\Documents\\MATLAB\\pfields_sim\\xdata.txt");
	for (int i = 0; i < 249; i++)
	{
		File >> x[i];
		//cout << "x = " << x[i] << endl;
	}
	File.close();

	File.open("C:\\Users\\jesus\\Documents\\MATLAB\\pfields_sim\\ydata.txt");
	for (int i = 0; i < 249; i++)
	{
		File >> y[i];
		//cout << "y = " << y[i] << endl;
	}
	File.close();

	emily_heading = 120 * pi / 180;

	ofstream velocity("velocity_profile.txt");

	double VGX = goal_loc_x + VIRTUAL_DISTANCE*cos(ORIENTATION + 3 * pi / 2);
	double VGY = goal_loc_y + VIRTUAL_DISTANCE*sin(ORIENTATION + 3 * pi / 2);

	double V[249];
	int i = 0;
	double Q = sqrt(pow((x[i] - VGX), 2) + pow((y[i] - VGY), 2));
	while (Q > 3)
	{
		if (i > 248)
		{
			break;
		}
		Q = sqrt(pow((x[i] - VGX), 2) + pow((y[i] - VGY), 2));
		V[i] = robot_control(PFIELDS, goal_loc_x, goal_loc_y, x[i], y[i], emily_heading, pfield_parameters);
		velocity << V[i] << endl;
		cout << "Q = " << Q << endl;
		i = i + 1;

	}
	system("pause");
	return 0;
}
