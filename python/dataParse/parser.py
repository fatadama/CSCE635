import numpy as np
import matplotlib.pyplot as plt
import math

# path to files
path = '../joystick/20160430'
# load files
control = np.genfromtxt(path+'/controlLog.csv',delimiter=',',skip_header=1)
controlOut = np.genfromtxt(path+'/controlOutLog.csv',delimiter=',',skip_header=1)
gps = np.genfromtxt(path+'/gpsLog.csv',delimiter=',',skip_header=1)

# rescale gps values to doubles
gps[:,2] = gps[:,2]*1.0e-7 # lon
gps[:,3] = gps[:,3]*1.0e-7 # lat
# FIRST GPS ONLY: correct heading to radians
gps[:,5] = gps[:,5]*(1.0/0.0174533)

# system time 0
t0 = controlOut[0,0]
control[:,0] = control[:,0]-t0
controlOut[:,0] = controlOut[:,0]-t0
gps[:,0] = gps[:,0]-t0

# bounding box for Bush lake
# 30.597427, -96.352148
# 30.597367, -96.351322
# 30.596259, -96.351322
# 30.596259, -96.352148
gpsLonLim = np.array([-96.352148,-96.351322])
gpsLatLim = np.array([30.596259,30.597427])

# find outliers in GPS that we can easily reject - anything outside the bounding box and anything where the time is more than 1000 seconds different
diffgpslogtime = np.zeros(len(gps))
diffgpslogtime[1:] = np.diff(gps[:,1])
# find gps home
gpsHome = gps[0,2:4].flatten()
print("Gps home lon/lat: %g,%g" % (gpsHome[0],gpsHome[1]))

# rejection criteria for GPS
def rejectGps(gpsdata):
    return ( (gps[:,2] > gpsLonLim[1]) | (gps[:,2] < gpsLonLim[0]) | (gps[:,3] > gpsLatLim[1]) | (gps[:,3] < gpsLatLim[0]) | (np.abs(diffgpslogtime) > 100.0) | (gps[:,1] > gps[-1,1]) | (gps[:,1] < gps[0,1]) | (gps[:,4] > 15.0) | (gps[:,5] < -2.0*math.pi) | (gps[:,5] > 2.0*math.pi) )

gpsOutliers = np.nonzero(rejectGps(gps))[0] #np.nonzero( (gps[:,2] > gpsLonLim[1]) | (gps[:,2] < gpsLonLim[0]) | (gps[:,3] > gpsLatLim[1]) | (gps[:,3] < gpsLatLim[0]) | (np.abs(diffgpslogtime) > 100.0) | (gps[:,1] > gps[-1,1]) | (gps[:,1] < gps[0,1]) )[0]
gpsInliers = np.setdiff1d(np.arange(0,len(gps)),gpsOutliers)
print("%d/%d GPS values outside bounding box" % (len(gpsOutliers),len(gps)))

# compute time between messages for control
dtc = np.zeros(len(control))
dtc[1:] = np.diff(control[:,0])
# compute time between messages for gps
dtg = np.zeros(len(gps))
dtg[1:] = np.diff(gps[:,0])

# compute gps XY position
gpsXY = np.zeros((len(gps),2))
gpsXY[:,0] = (gps[:,3]-gpsHome[1])*111318.845 # X
gpsXY[:,1] = (gps[:,2]-gpsHome[0])*111318.845 # Y
# compute distance from home
gpsRange = np.sqrt( np.sum(np.power(gpsXY,2.0),axis=1))

# plot the time interval between received messages
fig1 = plt.figure()
ax = []
ax.append( fig1.add_subplot(121,ylabel='dt',title='Time interval for control messages') )
ax[0].plot(control[:,0],dtc)
ax[0].grid()

ax.append( fig1.add_subplot(122,ylabel='dt',title='Time interval for GPS messages') )
ax[1].plot(gps[:,0],dtg)
ax[1].grid()

fig1.show()

print("Mean dt for control = %g sec" % np.mean(dtc))
print("Mean dt for GPS = %g sec" % np.mean(dtg))

# plot the position
fig2 = plt.figure()
ax = []
ax.append(fig2.add_subplot(511,ylabel='lat',xlabel='lon'))
ax[0].plot(gps[gpsInliers,2],gps[gpsInliers,3])
ax[0].set_xlim(gpsLonLim)
ax[0].set_ylim(gpsLatLim)
ax[0].grid()

ax.append(fig2.add_subplot(512,ylabel='vel (m/s)',xlabel='t (sec)'))
ax[1].plot(gps[gpsInliers,0],gps[gpsInliers,4])
ax[1].grid()

ax.append(fig2.add_subplot(513,ylabel='hdg (rad)',xlabel='t (sec)'))
ax[2].plot(gps[gpsInliers,0],gps[gpsInliers,5])
ax[2].grid()

ax.append(fig2.add_subplot(514,ylabel='X (m)',xlabel='t (sec)'))
ax[3].plot(gps[gpsInliers,0],gpsXY[gpsInliers,0])
ax[3].grid()

ax.append(fig2.add_subplot(515,ylabel='Y (m)',xlabel='t (sec)'))
ax[4].plot(gps[gpsInliers,0],gpsXY[gpsInliers,1])
ax[4].grid()

fig2.show()

fig3 = plt.figure()

ax = []
ax.append(fig3.add_subplot(211,title='Distance from home (m)',xlabel='time (sec)'))
ax[0].plot(gps[gpsInliers,0],gpsRange[gpsInliers])
ax[0].grid()

ax.append(fig3.add_subplot(212,title='Distance from home (m)',xlabel='delta-t of gps msg'))
ax[1].plot(dtg[gpsInliers],gpsRange[gpsInliers],'rx')
ax[1].grid()

fig3.show()

raw_input("Return to exit")

plt.close(fig1)
plt.close(fig2)
plt.close(fig3)
