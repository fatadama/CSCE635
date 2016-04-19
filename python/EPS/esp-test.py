import sys
sys.path.append("build/lib.linux-i686-2.7")

import eps
import struct

print("help")

lon = 1587654321
lat = 960000000
t = 0.1
print("Wrote to eps: %d,%d,%f" % (lon,lat,t))
(ret,lenv) = eps.pack_gps(lon,lat,t)
# try to unpack
out = eps.parse_buffer(ret)
for k in range(out[0]):
    j = 2*k+1 # 1, 3, 5
    # message id
    msg_id = int(out[j+1])
    if msg_id == eps.message_gps():
        (lon,lat,t,len2) = eps.unpack_gps(ret[out[j]:])
        print("Unpacked: %d,%d,%f" % (lon,lat,t))
# pack a control message
rudd = -0.5
thro = 0.66667
print("Packing: %f,%f" % (rudd,thro))
(ret,lenv) = eps.pack_control(rudd,thro)
# try to unpack
out = eps.parse_buffer(ret)
for k in range(out[0]):
    j = 2*k+1 # 1, 3, 5
    # message id
    msg_id = int(out[j+1])
    if msg_id == eps.message_control():
        (rudd,thro,len2) = eps.unpack_control(ret[out[j]:])
        print("Unpacked: %f,%f" % (rudd,thro))
# pack a command message
hdg = 0.25
spd = 0.7465
print("Packing: %f,%f" % (hdg,spd))
(ret,lenv) = eps.pack_command(hdg,spd)
# try to unpack
out = eps.parse_buffer(ret)
for k in range(out[0]):
    j = 2*k+1 # 1, 3, 5
    # message id
    msg_id = int(out[j+1])
    if msg_id == eps.message_command():
        (hdg,spd,len2) = eps.unpack_command(ret[out[j]:])
        print("Unpacked: %f,%f" % (hdg,spd))
# pack a set_pid message
ch = 1
Kp = 0.21249
Ki = -0.23823
Kd = 239.23890
print("Packing: %i,%f,%f,%f" % (ch,Kp,Ki,Kd))
(ret,lenv) = eps.pack_set_pid(ch,Kp,Ki,Kd)
# try to unpack
out = eps.parse_buffer(ret)
for k in range(out[0]):
    j = 2*k+1 # 1, 3, 5
    # message id
    msg_id = int(out[j+1])
    if msg_id == eps.message_set_pid():
        (ch,Kp,Ki,Kd,len2) = eps.unpack_set_pid(ret[out[j]:])
        print("Unpacked: %i,%f,%f,%f" % (ch,Kp,Ki,Kd))

print("exit")
