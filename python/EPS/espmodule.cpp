/* Decent reference for python extensions: http://www.tutorialspoint.com/python/python_further_extensions.htm */

#include <Python.h>
#include <stdint.h>
#include <stdio.h>
#include "../../arduino/libraries/ESP/ESP.h"

/** Pack data into a GPS message */
static PyObject * pack_gps(PyObject *self, PyObject *args){
  int32_t lat=0,lon=0;
  float t = 0.0;
  if (!PyArg_ParseTuple(args,"llf",&lon,&lat,&t)){
    return NULL;
  }
  else{
    // pack message and return the string...
    char byt[MSG_GPS_LEN];
    int8_t len = esp_pack_gps((uint8_t*)byt,lon,lat,t);
    return Py_BuildValue("s#i",byt,len,len);
  }
};

/** Pack data into a control message */
static PyObject * pack_control(PyObject *self, PyObject *args){
  float rudd, thro;
  if (!PyArg_ParseTuple(args,"ff",&rudd,&thro)){
    return NULL;
  }
  else{
    // pack message and return the string...
    char byt[MSG_CONTROL_LEN];
    int8_t len = esp_pack_control((uint8_t*)byt,rudd,thro);
    return Py_BuildValue("s#i",byt,len,len);
  }
};

/** Pack data into a command message */
static PyObject * pack_command(PyObject *self, PyObject *args){
  float heading, speed;
  if (!PyArg_ParseTuple(args,"ff",&heading,&speed)){
    return NULL;
  }
  else{
    // pack message and return the string...
    char byt[MSG_COMMAND_LEN];
    int8_t len = esp_pack_command((uint8_t*)byt,heading,speed);
    return Py_BuildValue("s#i",byt,len,len);
  }
};

/** Pack data into a command message
 *
 * @param[in] ch channel (0 = rudder, 1 = throttle)
 * @param[in] Kp proportional gain
 * @param[in] Ki integral gain
 * @param[in] Kd derivative gain
 */
static PyObject * pack_set_pid(PyObject *self, PyObject *args){
  float heading, speed;
  uint8_t ch;
  float KP, KI, KD;

  if (!PyArg_ParseTuple(args,"Bfff",(char*)&ch,&KP,&KI,&KD)){
    return NULL;
  }
  else{
    // pack message and return the string...
    char byt[MSG_SET_PID_LEN];
    int8_t len = esp_pack_set_pid((uint8_t*)byt,ch,KP,KI,KD);
    return Py_BuildValue("s#i",byt,len,len);
  }
};

/** Read a GPS message
 * @param[in] a packed GPS message
 */
static PyObject * unpack_gps(PyObject *self, PyObject *args){
  const char* byt;
  int len = MSG_GPS_LEN;
  if (!PyArg_ParseTuple(args,"s#",(&byt),&len)){
    return NULL;
  }
  else{
    // unpack message
    int32_t lat=0,lon=0;
    float t = 0.0;
    if ( len < MSG_GPS_LEN){
      // return fail condition
      return Py_BuildValue("llfi",lon,lat,t,-3);
    }
    int8_t len2 = esp_unpack_gps((uint8_t*)byt,&lon,&lat,&t);
    return Py_BuildValue("llfi",lon,lat,t,len2);
  }
}

/** Read a control message
 * @param[in] a packed control message
 */
static PyObject * unpack_control(PyObject *self, PyObject *args){
  const char* byt;
  int len = MSG_CONTROL_LEN;
  if (!PyArg_ParseTuple(args,"s#",(&byt),&len)){
    return NULL;
  }
  else{
    // unpack message
    float rudd=0.0, thro=0.0;
    if ( len < MSG_CONTROL_LEN){
      // return fail condition
      return Py_BuildValue("ffi",rudd,thro,-3);
    }
    int8_t len2 = esp_unpack_control((uint8_t*)byt,&rudd,&thro);
    return Py_BuildValue("ffi",rudd,thro,len2);
  }
}

/** Read a command message
 * @param[in] a packed command message
 */
static PyObject * unpack_command(PyObject *self, PyObject *args){
  const char* byt;
  int len = MSG_COMMAND_LEN;
  if (!PyArg_ParseTuple(args,"s#",(&byt),&len)){
    return NULL;
  }
  else{
    // unpack message
    float hdg=0.0, spd=0.0;
    if ( len < MSG_COMMAND_LEN){
      // return fail condition
      return Py_BuildValue("ffi",hdg,spd,-3);
    }
    int8_t len2 = esp_unpack_command((uint8_t*)byt,&hdg,&spd);
    return Py_BuildValue("ffi",hdg,spd,len2);
  }
}

/** Read a set_pid message
 * @param[in] a packed set_pid message
 * @param[out] ch channel (0=rudder,1=throttle)
 * @param[out] Kp proportional gain
 * @param[out] Ki integral gain
 * @param[out] Kd derivative gain
 * @param[out] len Number of bytes parsed; returns -3 if message is too short; -2 if message header wrong; -1 if message checksum wrong
 */
static PyObject * unpack_set_pid(PyObject *self, PyObject *args){
  const char* byt;
  int len = MSG_SET_PID_LEN;
  if (!PyArg_ParseTuple(args,"s#",(&byt),&len)){
    return NULL;
  }
  else{
    // unpack message
    uint8_t ch;
    float Kp=0.0,Ki = 0.0,Kd = 0.0;
    int8_t len2 = -3;
    if ( len >= MSG_SET_PID_LEN){
      len2 = esp_unpack_set_pid((uint8_t*)byt,&ch,&Kp,&Ki,&Kd);
    }
    return Py_BuildValue("Ifffi",ch,Kp,Ki,Kd,len2);
  }
}

static PyObject * parse_buffer(PyObject *self, PyObject *args){
  const char* buffy;
  int len;
  if (!PyArg_ParseTuple(args,"s#",(&buffy),&len)){
    return NULL;
  }
  else{
    int nMsg = 0;
    int msgId[3];
    int msgPos[3];
    for(int j = 0;j<len-2;j++){
      // check for the header bytes, then message id
      if ( (buffy[j] == ESP_HEADER1) && (buffy[j+1] == ESP_HEADER2) && (
        buffy[j+2] == MSG_GPS || buffy[j+2] == MSG_CONTROL || buffy[j+2] == MSG_COMMAND || buffy[j+2] == MSG_SET_PID ) ){
          // index 'j' is the start of a message
          msgId[nMsg] = buffy[j+2];
          msgPos[nMsg] = j;
          nMsg++;
          if (nMsg >= 3)
            break;
      }
    }
    // ultra-crude method for parsing up to three messages from the buffer
    if (nMsg == 0){
      return Py_BuildValue("i",nMsg);
    }
    if (nMsg == 1){
      return Py_BuildValue("iii",nMsg,msgPos[0],msgId[0]);
    }
    if (nMsg == 2){
      return Py_BuildValue("iiiii",nMsg,msgPos[0],msgId[0],msgPos[1],msgId[1]);
    }
    if (nMsg == 3){
      return Py_BuildValue("iiiiiii",nMsg,msgPos[0],msgId[0],msgPos[1],msgId[1],msgPos[2],msgId[2]);
    }
  }
}

/** Return the message ID for gps message */
static PyObject * message_gps(PyObject *self, PyObject *args){
  return Py_BuildValue("i",MSG_GPS);
}

/** Return the message ID for control message */
static PyObject * message_control(PyObject *self, PyObject *args){
  return Py_BuildValue("i",MSG_CONTROL);
}

/** Return the message ID for control message */
static PyObject * message_command(PyObject *self, PyObject *args){
  return Py_BuildValue("i",MSG_COMMAND);
}

/** Return the message ID for set_pid message */
static PyObject * message_set_pid(PyObject *self, PyObject *args){
  return Py_BuildValue("i",MSG_SET_PID);
}

static PyMethodDef EspMethods[] = {
  {"pack_gps",  pack_gps, METH_VARARGS,"Pack a GPS message."},
  {"pack_control",  pack_control, METH_VARARGS,"Pack a control message."},
  {"pack_command",  pack_command, METH_VARARGS,"Pack a command message."},
  {"pack_set_pid",  pack_set_pid, METH_VARARGS,"Pack a set_pid message."},
  {"unpack_gps",  unpack_gps, METH_VARARGS,"Unpack a GPS message."},
  {"unpack_control",  unpack_control, METH_VARARGS,"Unpack a control message."},
  {"unpack_command",  unpack_command, METH_VARARGS,"Unpack a command message."},
  {"unpack_set_pid",  unpack_set_pid, METH_VARARGS,"Unpack a set_pid message."},
  {"parse_buffer",  parse_buffer, METH_VARARGS,"Parse a buffer into messages"},
  {"message_gps",  message_gps, METH_VARARGS,"Get the message ID for GPS message"},
  {"message_control",  message_control, METH_VARARGS,"Get the message ID for control message"},
  {"message_command",  message_command, METH_VARARGS,"Get the message ID for command message"},
  {"message_set_pid",  message_set_pid, METH_VARARGS,"Get the message ID for set_pid message"},
  {NULL, NULL, 0, NULL}        /* Sentinel */
};

PyMODINIT_FUNC
initesp(void)
{
    (void) Py_InitModule("esp", EspMethods);
};
