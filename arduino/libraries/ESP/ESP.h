#ifndef __ESP_H_DEFINED__
#define __ESP_H_DEFINED__

#include <stdint.h>
#include <string.h>

// message header bytes
#define ESP_HEADER1 127
#define ESP_HEADER2 83

// return values from unpack() functions
/** The message header bytes were invalid */
#define ESP_BAD_MSG -1
/** The message checksum did not match the computed value */
#define ESP_BAD_CHECKSUM -2

// message IDs - must be positive and different powers of 2
#define MSG_GPS 1
#define MSG_CONTROL 2
#define MSG_COMMAND 4
#define MSG_SET_PID 8
#define MSG_GPS_POS 16

// message lengths
#define MSG_GPS_LEN 16
#define MSG_CONTROL_LEN 12
#define MSG_COMMAND_LEN 12
#define MSG_SET_PID_LEN 17
#define MSG_GPS_POS_LEN 25

// message format: ESP_HEADER1 ESP_HEADER2 MESSAGE_ID <data> CHECKSUM

/** @file */

/** Parse a message of known length. Determine if the checksum (last byte) in the message matches the checksum in the message.
 * @param[in] msg uint8_t* that contains a message
 * @param[in] msg_len length of the message
 * @param[out] checksum_valid boolean, true if the checksum in the message matches the computed value
 */
inline int8_t checksum_valid(uint8_t* msg, uint8_t msg_len){
	uint8_t chksum = 0,chksum_msg=0;
	for(uint8_t k=0;k<msg_len-1;k++)
		chksum+=msg[k];
	chksum_msg = msg[msg_len-1];
	return (chksum==chksum_msg);
}

/** Compute a message checksum.
 * @brief The checksum is the sum of all bytes in the message expect the last one, as an unsigned 8 bit integer
 * @param[in] msg uint8_t* that contains a message
 * @param[in] msg_len length of the message
 */
inline uint8_t compute_checksum(uint8_t*msg,uint8_t msg_len){
	uint8_t chksum = 0;
	for(uint8_t k=0;k<msg_len-1;k++)
		chksum+=msg[k];
	return chksum;
}

/** @brief Pack a GPS message
 *  @param[in] uint8_t*msg message buffer to pack the message into
 *	@param[in] int32_t lon Londgitude in (degrees x 10^-7) mapped from -180 to 180 deg
 * 	@param[in] int32_t lat Latitude in (degrees x 10^-7)
 *	@param[in] float time gps time, if available
 * 	\return {int8_t len: length of message}
 */
inline int8_t esp_pack_gps(uint8_t*msg,int32_t lon, int32_t lat, float time){
	uint8_t chksum = 0;
	int8_t msg_counter = 0;
	msg[msg_counter] = ESP_HEADER1;
	msg_counter++;//1
	msg[msg_counter] = ESP_HEADER2;
	msg_counter++;//2
	// message ID
	msg[msg_counter] = MSG_GPS;
	msg_counter++;//3
	// pack longitude
	memcpy(&msg[msg_counter],&lon,4);
	msg_counter += 4;//7
	// pack latitude
	memcpy(&msg[msg_counter],&lat,4);
	msg_counter += 4;//11
	// time
	memcpy(&msg[msg_counter],&time,4);
	msg_counter += 4;//15
	//checksum
	chksum = compute_checksum(msg,MSG_GPS_LEN);
	msg[msg_counter]=chksum;
	msg_counter++;//16
	return msg_counter;
};
/** Unpack a GPS message into longitude, latitude, and time
 *
 */
inline int8_t esp_unpack_gps(uint8_t*msg,int32_t* lon, int32_t* lat, float* time){
	//check the header bytes
	if (msg[0] != ESP_HEADER1 | msg[1] != ESP_HEADER2)
		return -2;
	//pass over the message ID byte (2)
	int8_t msg_counter=3;
	memcpy(lon,&msg[msg_counter],4);
	msg_counter+=4;
	//*lat = msg[msg_counter] + (msg[msg_counter+1]<<8) + (msg[msg_counter+2]<<16) + (msg[msg_counter+3]<<24);
	memcpy(lat,&msg[msg_counter],4);
	msg_counter+=4;
	memcpy(time,&msg[msg_counter],4);
	msg_counter = msg_counter + 1;
	// compute the checksum
	uint8_t chksum = compute_checksum(msg,MSG_GPS_LEN);
	// determine if message is valid
	if (checksum_valid(msg,MSG_GPS_LEN))
		return msg_counter+1;
	else
		return ESP_BAD_CHECKSUM;
};

/** @brief Pack a low-level control message
 *  @param[in] uint8_t*msg message buffer to pack the message into
 *	@param[in] float rudd rudder command mapped from -1 to 1
 * 	@param[in] float thro throttle command mapped from 0 to 1
 * 	\return {int8_t len: length of message}
 */
inline int8_t esp_pack_control(uint8_t*msg,float rudd, float thro){
	uint8_t chksum = 0;
	//header bytes
	int8_t msg_counter = 0;
	msg[msg_counter] = ESP_HEADER1;
	msg_counter++;//0
	msg[msg_counter] = ESP_HEADER2;
	msg_counter++;//1
	// message ID
	msg[msg_counter] = MSG_CONTROL;
	msg_counter++;//2
	// pack rudder
	memcpy(&msg[msg_counter],&rudd,4);
	msg_counter += 4;//6
	// pack throttle
	memcpy(&msg[msg_counter],&thro,4);
	msg_counter += 4;//10
	//end byte
	msg[msg_counter] = compute_checksum(msg,MSG_CONTROL_LEN);
	msg_counter++;//11
	return msg_counter;
};

/** Unpack a control message from buffer.
 * @param[in] uint8_t* msg parsed message array
 * @param[out] rudd commanded rudder, should be between -1.0 and 1.0
 * @param[out] thro commanded throttle, should be between 0.0 and 1.0
 */
inline int8_t esp_unpack_control(uint8_t*msg,float* rudd, float* thro){
	//check the header bytes
	if (msg[0] != ESP_HEADER1 | msg[1] != ESP_HEADER2)
		return ESP_BAD_MSG;
	// parse data
	int8_t msg_counter = 3;
	//rudder
	memcpy(rudd,&msg[msg_counter],4);
	msg_counter+=4;
	//throttle
	memcpy(thro,&msg[msg_counter],4);
	msg_counter+=4;
	// determine if message is valid
	if (checksum_valid(msg,MSG_CONTROL_LEN))
		return msg_counter+1;
	else
		return ESP_BAD_CHECKSUM;
}

/** @brief Pack a higher-level command with a desired direction and speed
 *  @param[in] uint8_t*msg message buffer to pack the message into
 * 	@param[in] float hdg; desired heading in radians
 *	@param[in] float speed; speed mapped from 0 to 1; equivalent to throttle mapping
 * 	\return {int8_t len: length of message}
 */
inline int8_t esp_pack_command(uint8_t*msg,float hdg, float speed){
	//header bytes
	int8_t msg_counter = 0;
	msg[msg_counter] = ESP_HEADER1;
	msg_counter++;
	msg[msg_counter] = ESP_HEADER2;
	msg_counter++;
	// message ID
	msg[msg_counter] = MSG_COMMAND;
	msg_counter++;
	// pack heading
	memcpy(&msg[msg_counter],&hdg,4);
	msg_counter += 4;
	// pack speed
	memcpy(&msg[msg_counter],&speed,4);
	msg_counter += 4;
	//end byte
	msg[msg_counter] = compute_checksum(msg,MSG_COMMAND_LEN);
	msg_counter++;
	return msg_counter;
};

inline int8_t esp_unpack_command(uint8_t*msg,float* hdg, float* speed){
	//check the header bytes
	if (msg[0] != ESP_HEADER1 | msg[1] != ESP_HEADER2)
		return ESP_BAD_MSG;
	// parse data
	int8_t msg_counter = 3;
	//hdg
	memcpy(hdg,&msg[msg_counter],4);
	msg_counter+=4;
	//speed
	memcpy(speed,&msg[msg_counter],4);
	msg_counter+=4;
	// determine if message is valid
	if (checksum_valid(msg,MSG_COMMAND_LEN))
		return msg_counter+1;
	else
		return ESP_BAD_CHECKSUM;
}

/** @brief Pack a PID message to set PID gains
 *  @param[in] uint8_t*msg message buffer to pack the message into
 *  @param[in] uint8_t ch channel to set gains; 0 is steering, 1 is throttle
 * 	@param[in] float KP proportional gain
 * 	@param[in] float KI integral gain
 * 	@param[in] float KD derivative gain
 * 	\return {int8_t len: length of message}
 */
inline int8_t esp_pack_set_pid(uint8_t*msg,uint8_t ch,float KP, float KI, float KD){
	//header bytes
	int8_t msg_counter = 0;
	msg[msg_counter] = ESP_HEADER1;
	msg_counter++;
	msg[msg_counter] = ESP_HEADER2;
	msg_counter++;
	// message ID
	msg[msg_counter] = MSG_SET_PID;
	msg_counter++;
	// pack channel
	msg[msg_counter] = ch;
	msg_counter++;
	// pack proportional gain
	memcpy(&msg[msg_counter],&KP,4);
	msg_counter += 4;
	// pack integral gain
	memcpy(&msg[msg_counter],&KI,4);
	msg_counter += 4;
	// pack derivative gain
	memcpy(&msg[msg_counter],&KD,4);
	msg_counter += 4;
	//end byte
	msg[msg_counter] = compute_checksum(msg,MSG_SET_PID_LEN);
	msg_counter++;
	return msg_counter;
};

inline int8_t esp_unpack_set_pid(uint8_t*msg,uint8_t* ch,float* KP, float* KI, float* KD){
	//check the header bytes
	if (msg[0] != ESP_HEADER1 | msg[1] != ESP_HEADER2)
		return ESP_BAD_MSG;
	// parse data
	int8_t msg_counter = 3;
	// channel
	memcpy(ch,&msg[msg_counter],1);
	msg_counter+=1;
	//proportional gain
	memcpy(KP,&msg[msg_counter],4);
	msg_counter+=4;
	//integral gain
	memcpy(KI,&msg[msg_counter],4);
	msg_counter+=4;
	//derivative gain
	memcpy(KD,&msg[msg_counter],4);
	msg_counter+=4;
	// determine if message is valid
	if (checksum_valid(msg,MSG_SET_PID_LEN))
		return msg_counter+1;
	else
		return ESP_BAD_CHECKSUM;
}

/** Pack a combined gps/pose message with GPS data plus speed and heading
 * @param[in] msg buffer for the message
 * @param[in] lon the longitude in (degrees) x 10^7
 * @param[in] lat the latitude in (degrees) x 10^7
 * @param[in] t the time
 * @param[in] v the speed in m/s
 * @param[in] hdg the heading in radians
 * @param[in] status the status of GPS. 0 == no lock, 1 == lock
 * @param[out] msg_len the number of bytes written to the buffer
 */
inline int8_t esp_pack_gps_pos(uint8_t*msg,int32_t lon, int32_t lat, float t, float v, float hdg, uint8_t status){
	uint8_t chksum = 0;
	int8_t msg_counter = 0;
	msg[msg_counter] = ESP_HEADER1;
	msg_counter++;//1
	msg[msg_counter] = ESP_HEADER2;
	msg_counter++;//2
	// message ID
	msg[msg_counter] = MSG_GPS_POS;
	msg_counter++;//3
	// pack longitude
	memcpy(&msg[msg_counter],&lon,4);
	msg_counter += 4;//7
	// pack latitude
	memcpy(&msg[msg_counter],&lat,4);
	msg_counter += 4;//11
	// time
	memcpy(&msg[msg_counter],&t,4);
	msg_counter += 4;//15
	//speed (m/s)
	memcpy(&msg[msg_counter],&v,4);
	msg_counter += 4;//19
	// heading (radians)
	memcpy(&msg[msg_counter],&hdg,4);
	msg_counter += 4;//23
	// the status byte
	memcpy(&msg[msg_counter],&status,1);
	msg_counter += 1;//24
	//checksum
	chksum = compute_checksum(msg,MSG_GPS_POS_LEN);
	msg[msg_counter]=chksum;
	msg_counter++;//25
	return msg_counter;
}

/** Unpack a combined gps/pose message with GPS data plus speed and heading
 * @param[in] msg buffer with the message
 * @param[in] lon pointer to place the longitude in (degrees) x 10^7
 * @param[in] lat pointer to place the latitude in (degrees) x 10^7
 * @param[in] t pointer to place the time
 * @param[in] v pointer to place the speed in m/s
 * @param[in] hdg pointer to place the heading in radians
 * @param[in] status pointer to place the status byte in
 * @param[out] msg_len the number of bytes written to the buffer
 */
inline int8_t esp_unpack_gps_pos(uint8_t*msg,int32_t* lon, int32_t* lat, float* t, float* v, float* hdg,uint8_t* status){
	//check the header bytes
	if (msg[0] != ESP_HEADER1 | msg[1] != ESP_HEADER2 | msg[2] != MSG_GPS_POS)
		return ESP_BAD_MSG;
	// parse data
	int8_t msg_counter = 3;
	// longitude
	memcpy(lon,&msg[msg_counter],4);
	msg_counter+=4;//7
	// latitude
	memcpy(lat,&msg[msg_counter],4);
	msg_counter+=4;//11
	// time
	memcpy(t,&msg[msg_counter],4);
	msg_counter+=4;//15
	// speed (m/s)
	memcpy(v,&msg[msg_counter],4);
	msg_counter+=4;//19
	// heading (rads)
	memcpy(hdg,&msg[msg_counter],4);
	msg_counter+=4;//23
	// status
	memcpy(status,&msg[msg_counter],1);
	msg_counter+=1;//24
	// determine if message is valid
	if (checksum_valid(msg,MSG_GPS_POS))
		return msg_counter+1;//25
	else
		return ESP_BAD_CHECKSUM;
}

/** Parse a single byte.
 *
 * Use internal static variable to track whether we've started parsing.
 * Return values: -2: bad checksum
 * 								-1: not parsing a message, looking for heading bytes
 * 								 0: currently parsing a message, still ongoing
 * 								 >0: just parsed a message, return the message type
 */
inline int8_t esp_parse_byte(uint8_t byte, uint8_t*buffy){
	static int msg_counter = 0;
	static int msg_len = -1;
	if (msg_counter==0){
		if (byte!=ESP_HEADER1){
			msg_len = -1;//reset the length
			return -1;
		}
	}
	if(msg_counter==1){
		if (byte!=ESP_HEADER2){
			msg_counter = 0;//reset the counter
			msg_len = -1;//reset the length
			return -1;
		}
	}
	if(msg_counter==2){//message id byte
		// set the target length based on the message type
		switch(byte){
			case MSG_GPS:
				msg_len = MSG_GPS_LEN;
				break;
			case MSG_COMMAND:
				msg_len = MSG_COMMAND_LEN;
				break;
			case MSG_CONTROL:
				msg_len = MSG_CONTROL_LEN;
				break;
			case MSG_SET_PID:
				msg_len = MSG_SET_PID_LEN;
				break;
			case MSG_GPS_POS:
				msg_len = MSG_GPS_POS_LEN;
				break;
			default:
				msg_counter=0;
				msg_len=-1;
				return -1;
				break;
		}
	}
	// store the new byte
	buffy[msg_counter] = byte;
	if(msg_counter==(msg_len-1)){
		// validate the checksum
		if (checksum_valid(buffy,msg_len)){
			msg_counter = 0;
			msg_len = -1;
			return 1;
		}
		else{
			msg_counter = 0;
			msg_len = -1;
			return ESP_BAD_CHECKSUM;
		}
	}
	msg_counter++;
	return 0;
};

#endif
