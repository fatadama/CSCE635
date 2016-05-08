#include "msgParser.h"
#include <stdio.h>

void handler(int msgid, int*msgInts, double*msgDoubles) {
	switch (msgid) {
	case MSG_MODE:
		printf("MODE MESSAGE:t=%f,mode=%d\n", msgDoubles[0], msgInts[0]);
		break;
	case MSG_EMILY_GPS:
		printf("MODE_EMILY_GPS:t=%f,lat=%f,lat=%f,v=%f,hdg=%f\n", msgDoubles[0], msgDoubles[1], msgDoubles[2], msgDoubles[3], msgDoubles[4]);
		break;
	case MSG_TARGET_GPS:
		printf("MODE_TARGET_GPS:t=%f,lat=%f,lat=%f,hdg=%f\n", msgDoubles[0], msgDoubles[1], msgDoubles[2], msgDoubles[3]);
		break;
	}
}

int main(){
	// build some test messages
	char msgBuffer[128];
	double doubles[8];
	int ints[8];
	int msgId = -1;

	printf("TEST MSG 1: MODE MESSAGE\n");
	sprintf_s(msgBuffer, "%4s,%f,%d", "mode", 0.281292, 0);
	parse_msg(msgBuffer, &msgId, ints, doubles);
	handler(msgId, ints, doubles);

	printf("TEST MSG 2: EMILY GPS MESSAGE\n");
	sprintf_s(msgBuffer, "%4s,%lf,%lf,%lf,%lf,%lf", "egps", -21388.12389213, 92.328932111,-238.3283123111,8.92827293,0.1282612);
	parse_msg(msgBuffer, &msgId, ints, doubles);
	handler(msgId, ints, doubles);

	printf("TEST MSG 3: TARGET GPS MESSAGE\n");
	sprintf_s(msgBuffer, "%4s,%lf,%lf,%lf,%lf", "tgps", -21388.12389213, 92.328932111, -238.3283123111,  0.1282612);
	parse_msg(msgBuffer, &msgId, ints, doubles);
	handler(msgId, ints, doubles);
  
	return 0;
}
