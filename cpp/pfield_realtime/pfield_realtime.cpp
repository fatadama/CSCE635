#include <Windows.h>
#include <stdio.h>
#include <nn.h>
#include <pair.h>
#include "msgParser.h"
#include "pfields.h"

/** The execution frequency in milliseconds: 50 Hz = 0.02 sec = 20 ms */
#define EXECUTE_PERIOD_MILLIS 20
/** Emily latitude (deg) */
double elat = 0.0;
/** Emily longitude (deg) */
double elon = 0.0;
double etime = 0.0;
double ev = 0.0;
double ehdg = 0.0;
/** Target latitude (deg) */
double tlat = 0.0;
double tlon = 0.0;
double ttime = 0.0;
double thdg = 0.0;

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


void read_bridge(int*sock) {
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
}

void write_bridge(int*sock,double*t,double*v,double*h) {
	char buffer[1024];
	sprintf_s(buffer, "%4s,%lf,%lf,%lf", "rspd", *t, *v, *h);
	buffer[4 + 3 * sizeof(double)] = 0;
	// send bytes
	int bytes = nn_send(*sock, buffer, 4 + 3*sizeof(double), NN_DONTWAIT);
	if (bytes > 0) {
		printf("Sent %s\n", buffer);
	}
}

int main() {

	// nanomsg pair socket
	int sock = nn_socket(AF_SP, NN_PAIR);
	printf("%d\n", NN_DONTWAIT);
	// initialize socket from settings
	char pair_url[] = "ipc://test";
	if (nn_connect(sock, pair_url) >= 0) {
		printf("Connected to server\n");
	}
	else {
		printf("Failed to connect to server\n");
	}
	// output variables
	double syst = 0.0, v_ref = 1.0, hdg_ref = 0.56431;


	// initialize pfields static parameters and vars
	// initialize timer vars
	LARGE_INTEGER frequency;
	LARGE_INTEGER t1, t2;           // ticks
	double millis = 0.0, mnext = 0.0;

	// get ticks per second
	QueryPerformanceFrequency(&frequency);
	QueryPerformanceCounter(&t1);

	// time of next execution (millis)
	mnext = t1.QuadPart*1000.0 / frequency.QuadPart + EXECUTE_PERIOD_MILLIS;

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
			// read from xbee_bridge
			read_bridge(&sock);
			// compute pfields
			// write to xbee_bridge
			write_bridge(&sock,&syst,&v_ref,&hdg_ref);
		}
	}

	return 0;
}