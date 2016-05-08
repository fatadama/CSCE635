#ifndef __MSGPARSER__H__DEFINED__
#define __MSGPARSER__H__DEFINED__

/** @file Header only library for simple nanomsg parsing */

#include <iostream>
#include <stdio.h>
#include <cstring>
//#include <windows.h>

// Message definitions
/** A mode message - swap teleoperation and pfields: msgid = mode  */
#define MSG_MODE 1
/** An EMILY position and pose message: msgid = egps  */
#define MSG_EMILY_GPS 2
/** A Target position and pose message: msgid = tgps  */
#define MSG_TARGET_GPS 4
/** A reference speed and heading message: msgid = rspd */
#define MSG_SPEED_REF 8

inline int get_msg_id(char*msg){
  char msgid[16];
  sscanf(msg,"%4s",msgid);
  msgid[4] = 0;
  if (!_strcmpi(msgid,"mode")){
    return MSG_MODE;
  }
  if (!_strcmpi(msgid,"egps")){
    return MSG_EMILY_GPS;
  }
  if (!_strcmpi(msgid,"tgps")){
    return MSG_TARGET_GPS;
  }
  if (!_strcmpi(msgid, "rspd")) {
	  return MSG_SPEED_REF;
  }
  return -1;
}

/** Unpack a MODE message
*
* Message format: <mode,(double)time(sec),(int)mode>
*/
inline void parse_msg_mode(char*msg, int*msgInts, double*msgDoubles) {
	sscanf(&msg[5], "%lf,%d", &msgDoubles[0], &msgInts[0]);
	return;
}

/** Unpack an EMILY_GPS message
*
* Message format: <egps,(double)time(sec),(double)gpslat(deg),(double)gpslon(deg),(double)speed(m/s),(double)heading(rad)>
*/
inline void parse_msg_emily_gps(char*msg, double*msgDoubles) {
	sscanf(&msg[5], "%lf,%lf,%lf,%lf,%lf", &msgDoubles[0], &msgDoubles[1], &msgDoubles[2], &msgDoubles[3], &msgDoubles[4]);
}

/** Unpack an TARGET_GPS message
*
* Message format: <tgps,(double)time(sec),(double)gpslat(deg),(double)gpslon(deg),(double)direction(rad)>
*/
inline void parse_msg_target_gps(char*msg, double*msgDoubles) {
	sscanf(&msg[5], "%lf,%lf,%lf,%lf", &msgDoubles[0], &msgDoubles[1], &msgDoubles[2], &msgDoubles[3]);
}

/** Unpack an SPEED_REF message
*
* Message format: <rspd,(double)time(sec),(double)v(normalized),(double)heading(radians)>
*/
inline void parse_msg_speed_ref(char*msg, double*msgDoubles) {
	sscanf(&msg[5], "%lf,%lf,%lf", &msgDoubles[0], &msgDoubles[1], &msgDoubles[2]);
}

/** Parse a message buffer; extract the message ID and contents into an integer and double array
 *
 * @param[in] msg: the raw message from nanomsg
 * @param[out] msgid: the message ID is put here
 * @param[out] msgInts: Any integers in the message are put here sequentially
 * @param[out] msgDoubles: Any doubles in the message are put here sequentially
 */
inline void parse_msg(char*msg,int*msgid,int*msgInts,double*msgDoubles){
  // count the number of ints and doubles received from the target message
  // get the message id
  *msgid = get_msg_id(msg);
  // the first four bytes are always the message id followed by a comma: parse the remaining bytes from msg[5] -> end
  switch(*msgid){
    case MSG_MODE:
      parse_msg_mode(msg,msgInts,msgDoubles);
      break;
    case MSG_EMILY_GPS:
      parse_msg_emily_gps(msg,msgDoubles);
      break;
    case MSG_TARGET_GPS:
      parse_msg_target_gps(msg,msgDoubles);
      break;
	case MSG_SPEED_REF:
		parse_msg_speed_ref(msg, msgDoubles);
		break;
  }
  return;
}

#endif
