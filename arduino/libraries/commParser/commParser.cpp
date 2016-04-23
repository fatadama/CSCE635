#include <stdint.h>
#include <string.h>
#include "commParser.h"

commParser::commParser(){
  bad_checksums=0;
  bad_packets=0;
  // set the stream times to zero
  next_stream_time_millis[0] = 0;
  next_stream_time_millis[1] = 0;
  next_stream_time_millis[2] = 0;
  // sest number of messages to zero
  received_messages = 0;
  // default comm status is lost
  comm_status = COMM_STATUS_LOST;
  // default to passive control mode
  control_mode = CONTROL_MODE_PASSIVE;
  control_rudder = 0.0;
  control_throttle = 0.0;
  received_msg_bitstring = 0;
}

void commParser::newBytes(uint8_t*bytes,int len,uint32_t millis){
  for(int k = 0;k<len;k++){
    int8_t out = esp_parse_byte(bytes[k],msg);
    if (out > 0){
      //we got a new message: handle it
      handleMsg();
      // save the time
      last_message_millis = millis;
    }
    if (out == -2){//increment the bad checksum counter
      bad_checksums++;
    }
  }
}

void commParser::handleMsg(){
  //increment the counter
  received_messages++;
  //switch based on the message ID
  //update the bistring
  received_msg_bitstring |= msg[2];
  switch (msg[2]){
    case MSG_GPS:
      float t;
      int32_t lon,lat;
      if (esp_unpack_gps(msg,&lon,&lat,&t) > 0){
        //set status
        gpsCmd.set(lat,lon,t);
      }
      else//increment the bad packet counter
        bad_packets++;
      break;
    case MSG_CONTROL:/** Direct offboard control of rudder/throttle */
      if (esp_unpack_control(msg,&control_rudder, &control_throttle) > 0){
        //set status rudder, throttle, and mode
        control_mode = CONTROL_MODE_DIRECT;
      }
      else//increment the bad packet counter
        bad_packets++;
      break;
    case MSG_COMMAND:
      break;
    case MSG_SET_PID:
      //uint8_t*msg,uint8_t* ch,float* KP, float* KI, float* KD
      uint8_t ch;
      float Kpi, Kdi, Kii;
      if (esp_unpack_set_pid(msg,&ch,&Kpi,&Kii,&Kdi) > 0){
        // set the gains in the status variable
        Kp[ch] = Kpi;
        Kd[ch] = Kdi;
        Ki[ch] = Kii;
      }
      break;
  }
}

uint32_t commParser::get_bad_packets(){
  return bad_packets;
}

uint32_t commParser::get_bad_checksums(){
  return bad_checksums;
}

void commParser::misc_tasks(uint32_t millis,emilyStatus st){ // updates the values of comm_status, control_mode, and MAY put transmit data in the output buffer
  // check the time since last message and set the status appropriately
  if (millis-last_message_millis < TIMEOUT_WARNING_MILLIS){
    comm_status = COMM_STATUS_HEALTHY;
  }
  // if we're in a non-healthy comm status mode, set the control mode to passive and wait to see if things improve
  if (millis-last_message_millis > TIMEOUT_WARNING_MILLIS & millis-last_message_millis < TIMEOUT_LOST_MILLIS){
    comm_status = COMM_STATUS_WARNING;
    control_mode = CONTROL_MODE_PASSIVE;
  }
  if (millis-last_message_millis > TIMEOUT_LOST_MILLIS){
    comm_status = COMM_STATUS_LOST;
    control_mode = CONTROL_MODE_PASSIVE;
  }
  // reset the send buffer position
  send_buffer_counter = 0;
  send_buffer_counter_helper = 0;
  // send periodic messages
  for(int k = 0;k<2;k++){
    if (next_stream_time_millis[k] <= millis){
      if (k == 0){ // GPS stream, send current GPS
        next_stream_time_millis[k] = millis + STREAM_PERIOD_GPS;
        if (st.gpsNow.init){
          // send GPS if initialized
          if (esp_pack_gps_pos(&send_buffer[send_buffer_counter],st.gpsNow.lon,
            st.gpsNow.lat,st.gpsNow.t,st.gpsNow.v,st.gpsNow.hdg,(uint8_t)st.gpsNow.health) > 0)
            send_buffer_counter+=MSG_GPS_POS_LEN;
        }
      }
      if (k == 1){ // control-related message stream
        next_stream_time_millis[k] = millis + STREAM_PERIOD_CONTROL;
        if (control_mode == CONTROL_MODE_DIRECT){// in DIRECT mode, echo the rudder and throttle commands
          if (esp_pack_control(&send_buffer[send_buffer_counter],st.control_rudder,st.control_throttle) > 0)
            send_buffer_counter+=MSG_CONTROL_LEN;
        }
        else if(control_mode == CONTROL_MODE_INDIRECT){// in INDIRECT mode, send the current rudder and throttle settings
          continue;//TODO stream messages after we add this functionality
        }
      }
      if (k == 2){ // send a heartbeat
        next_stream_time_millis[k] = millis + STREAM_PERIOD_HEARTBEAT;
        esp_pack_heartbeat(&send_buffer[send_buffer_counter],ESP_ID_BOAT,ESP_ID_GROUNDSTATION,(1.0e-3*millis));
        send_buffer_counter+=MSG_HEARTBEAT_LEN;
      }
    }
  }
}

uint8_t commParser::bytes_to_send(){
  return send_buffer_counter-send_buffer_counter_helper;
}

uint8_t commParser::get_next_byte(){
  send_buffer_counter_helper++;
  return send_buffer[send_buffer_counter_helper-1];
}

uint32_t commParser::get_number_received_messages(){
  return received_messages;
}

void commParser::sync(emilyStatus*st){
  st->control_mode = control_mode;
  st->comm_status = comm_status;
}

void commParser::sync_after_receive(emilyStatus*st){
  if (received_msg_bitstring & MSG_GPS){
    if (gpsCmd.is_new()){
      st->gpsCmd.set(gpsCmd.lat,gpsCmd.lon,gpsCmd.t);
      // HACK: call get() to make gpsCmd no longer new
      float x,y;
      gpsCmd.get(&x,&y);
    }
  }
  if (received_msg_bitstring & MSG_CONTROL){
    st->control_rudder = control_rudder;
    st->control_throttle = control_throttle;
  }
  if (received_msg_bitstring & MSG_COMMAND){
    // TODO
  }
  if (received_msg_bitstring & MSG_SET_PID){
    // copy the PID gains
    for(int j = 0;j<2;j++){
      st->Kp[j] = Kp[j];
      st->Ki[j] = Ki[j];
      st->Kd[j] = Kd[j];
    }
  }
  //reset the receive bitstring
  received_msg_bitstring = 0;
}
