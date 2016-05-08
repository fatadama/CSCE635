#include <Windows.h>
#include <stdio.h>
#include <nn.h>
#include <pair.h>
#define _USE_MATH_DEFINES // for C++
#include <cmath>
#include <cstring>
#include <fstream>
#include "msgParser.h"
#include "pfields.h"

/** The execution frequency in milliseconds: 50 Hz = 0.02 sec = 20 ms */
#define EXECUTE_PERIOD_MILLIS 20
/** Conversion factor for latitude/longitude in degrees to meters */
#define DEG2M 111318.845
/** Emily latitude (deg) */
double elat = 0.0;
/** Emily longitude (deg) */
double elon = 0.0;
/** EMILY GPS last time (sec) */
double etime = 0.0;
/** EMILY speed (m/s) */
double ev = 0.0;
/** EMILY heading (rad) */
double ehdg = 0.0;
/** Target latitude (deg) */
double tlat = 0.0;
/** Target longitude (deg) */
double tlon = 0.0;
/** Target GPS last message time (sec) */
double ttime = 0.0;
/** Target heading (rad) */
double thdg = 0.0;
/** EMILY coordinates in local frame */
double ex = 0.0;
/** EMILY Y coordinate in local frame */
double ey = 0.0;

using namespace std;

void handler(int msgid, int*msgInts, double*msgDoubles) {
	switch (msgid) {
	case MSG_EMILY_GPS:
		printf("MODE_EMILY_GPS:t=%f,lat=%f,lat=%f,v=%f,hdg=%f\n", msgDoubles[0], msgDoubles[1], msgDoubles[2], msgDoubles[3], msgDoubles[4]);
		etime = msgDoubles[0];
		elat = msgDoubles[1];
		elon = msgDoubles[2];
		ev = msgDoubles[3];
		ehdg = msgDoubles[4];
		break;
	case MSG_TARGET_GPS:
		printf("MODE_TARGET_GPS:t=%f,lat=%f,lat=%f,hdg=%f\n", msgDoubles[0], msgDoubles[1], msgDoubles[2], msgDoubles[3]);
		ttime = msgDoubles[0];
		tlat = msgDoubles[1];
		tlon = msgDoubles[2];
		thdg = msgDoubles[3];
		break;
	}
}


int read_bridge(int*sock) {
	static char *buf = NULL;
	static char msg[256];
	static int bytes = 0;
	static int msgid = 0;
	static int ints[8];
	static double doubles[8];

	bytes = nn_recv(*sock, &buf, NN_MSG, NN_DONTWAIT);
	if (bytes >= 0) {
		// parse the message
		parse_msg(buf, &msgid, ints, doubles);
		// handle the message
		handler(msgid, ints, doubles);
		// clear memory
		nn_freemsg(buf);
	}
	return bytes;
}

int write_bridge(int*sock,double*t,double*v,double*h) {
	char buffer[1024];
	sprintf_s(buffer, "%4s,%lf,%lf,%lf", "rspd", *t, *v, *h);
	//buffer[5 + 3*sizeof(double)+1] = 0;
	int lenz = strlen(buffer)+1;
	// send bytes
	int bytes = nn_send(*sock, buffer, lenz, NN_DONTWAIT);
	if (bytes > 0) {
		printf("Sent: %s\n", buffer);
	}
	return bytes;
}

/** Update EMILY's local coordinates using the current values of the target and EMILY GPS
 *
 */
void convert_cooordinates() {
	ex = (elat - tlat)*DEG2M;
	ey = (elon - tlon)*DEG2M;
	// heading is already set I think?
}

int main() {

	// nanomsg pair socket
	int sock = nn_socket(AF_SP, NN_PAIR);
	printf("%d\n", NN_DONTWAIT);
	/* initialize socket from settings */
	char pair_url[] = "ipc://test";
	if (nn_connect(sock, pair_url) >= 0) {
		printf("Connected to server\n");
	}
	else {
		printf("Failed to connect to server\n");
	}
	// output variables
	double syst = 0.0, v_ref = 1.0, hdg_ref = 0.56431;

	/* initialize pfields static parameters and vars */
	double PFIELDS[3] = { 1,2,0 };
	//double PFIELDS[3] = { 0,1,1 };
	double D_TAN, C_TAN, D_ATT, C_ATT, D_BALL, C_BALL, VIRTUAL_DISTANCE, ORIENTATION;
	// TANGENTIAL, ATTRACTIVE & BALLISTIC PFIELDS
	D_TAN = 100.0; C_TAN = 1.0;
	D_ATT = 100.0; 
	C_ATT = 2.0 / 7.5;
	//C_ATT = 8.0;
	D_BALL = 2000.0; C_BALL = 0.1 / 7.5;
	VIRTUAL_DISTANCE = 8.0;
	ORIENTATION = M_PI / 6.0;
	double pfield_parameters[8] = { D_TAN,C_TAN,D_ATT,C_ATT,D_BALL,C_BALL,VIRTUAL_DISTANCE,ORIENTATION };
	int MP_parameters[3] = { 7,8,4 };
	double output_pfield[2] = { 0.0,0.0 };
	double emilyx = 0.0, emilyy = 0.0, emilyhdg = 0.0;

	/* initialize timer vars */
	LARGE_INTEGER frequency;
	LARGE_INTEGER t1, t2;           // ticks
	double millis = 0.0, mnext = 0.0;

	/* Initialize clock */
	// get ticks per second
	QueryPerformanceFrequency(&frequency);
	QueryPerformanceCounter(&t1);
	// time of next execution (millis)
	mnext = t1.QuadPart*1000.0 / frequency.QuadPart + EXECUTE_PERIOD_MILLIS;

	/* Log file */
	SYSTEMTIME LocalTime;

	//To get the local time

	GetLocalTime(&LocalTime);
	char fname[256];
	sprintf(fname, "%04d%02d%02d_%02d%02d%02d.csv", LocalTime.wYear, LocalTime.wMonth, LocalTime.wDay, LocalTime.wHour, LocalTime.wMinute, LocalTime.wSecond);
	ofstream ofs;
	ofs.open(fname, ofstream::out);
	char output[1024];
	int size = sprintf(output, "t(sec),emily lon (deg), emily lat (deg),emily X(m),emily Y(m),emily heading (rad),v_ref(m/s),hdg_ref(rad),target lon (deg), target lat (deg)\n");
	ofs.write(output,size);

	// main loop
	while (1) {
		QueryPerformanceCounter(&t2);
		double millis = (t2.QuadPart)*1000.0 / frequency.QuadPart;//time in millis
		// if time_to_execute (50 Hz?)
		if (millis >= mnext) {
			// reset counter
			mnext = millis + EXECUTE_PERIOD_MILLIS;
			QueryPerformanceCounter(&t1);
			//update the system time
			syst = millis*0.001;
			// read from xbee_bridge until buffer empty
			int bytes = read_bridge(&sock);
			while (bytes > 0) {
				bytes = read_bridge(&sock);
			}
			// update input to local coordinates
			convert_cooordinates();
			// set pfields variables
			pfield_parameters[8] = thdg;
			// compute pfields, set the target position to the origin
			emilyx = ex;
			emilyy = ey;
			emilyhdg = ehdg;
			robot_control(PFIELDS, 0.0, 0.0, emilyx, emilyy, emilyhdg, pfield_parameters, output_pfield);
			// set the outputs
			v_ref = output_pfield[0];
			hdg_ref = output_pfield[1];

			// write to xbee_bridge
			//printf("t=%12.6g,v=%6.4f,h=%6.4f\n", syst, v_ref, hdg_ref);
			char output[1024];
			int size = sprintf(output, "%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g\n",syst,elon,elat,emilyx,emilyy,emilyhdg,v_ref, hdg_ref,tlon,tlat);
			ofs.write(output,size);
			write_bridge(&sock,&syst,&v_ref,&hdg_ref);
		}
	}

	ofs.close();

	return 0;
}