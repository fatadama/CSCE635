import numpy as np
import scipy as sp
import matplotlib.pyplot as plt
import math
import sys
sys.path.append('../../../estimation/filters/python/ekf')
import ekf
sys.path.append('../xbee_bridge')
from filter_dynamics import propagate, propGradient, processMatrix, measurement, measurementGradient, sigma_jerk, sigma_gps, acceptGps

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
# find gps home
gpsHome = gps[0,2:4].flatten()
# system time 0
t0 = controlOut[0,0]
control[:,0] = control[:,0]-t0
controlOut[:,0] = controlOut[:,0]-t0
gps[:,0] = gps[:,0]-t0
# Time to start at
tstart = 150.0
# truncate the input vectors
gi = np.nonzero(gps[:,0]>=tstart)[0][0]
ci = np.nonzero(control[:,0]>=tstart)[0][0]
gps = gps[gi:,:]
control = control[ci:,:]
# GPS to X-Y position relative to home
gpsXY = np.zeros((len(gps),2))
gpsXY[:,0] = (gps[:,3]-gpsHome[1])*111318.845 # X
gpsXY[:,1] = (gps[:,2]-gpsHome[0])*111318.845 # Y

Qkin = np.diag([math.pow(sigma_jerk,2.0),math.pow(sigma_jerk,2.0)])
Rkin = np.diag([math.pow(sigma_gps,2.0),math.pow(sigma_gps,2.0)])
EKF = ekf.ekf(6,0,propagate,propGradient,processMatrix,Qk=Qkin)

# initialize filter
xk0 = np.array([gpsXY[0,0],gpsXY[0,1],0.0,0.0,0.0,0.0])
Pk0 = np.diag([math.pow(sigma_gps,2.0),math.pow(sigma_gps,2.0),1.0,1.0,1.0,1.0])
EKF.init_P(xk0,Pk0,gps[0,0])
# previous value of GPS time, X, Y, speed, heading (rad) - used for outlier rejection
stateLast = np.array([gps[0,0],gpsXY[0,0],gpsXY[0,1],gps[0,4],gps[0,5]])
# count bad GPS
rejectCount = 0
# log the state
LEN = 3000
xLog = np.zeros((LEN,6))
xLog[0,:] = EKF.xhat.copy()
Plog = np.zeros((LEN,6))
Plog[0,:] = np.diag(EKF.Pk.copy())
Pfull = np.zeros((LEN,6,6))
Pfull[0,:,:] = EKF.Pk.copy()
Ylog = np.zeros((LEN,5))
Ylog[0,:] = stateLast.copy()
uLog = np.zeros((LEN,2))
uLog[0,:] = control[0,1:3]
# count how many good data points we've used
logCounter = 0;
# count how many total log entries we've looked at
k = 0
gpsOutliers = []
while(logCounter < LEN-1):
    try:
        k = k+1
        # for each measurement: determine if reject or not
        state = np.array([gps[k,0],gpsXY[k,0],gpsXY[k,1],gps[k,4],gps[k,5]])
        if acceptGps(state,stateLast,False):
            logCounter = logCounter+1
            # find the control index before the current index
            ci = np.nonzero(control[:,0] <= state[0])[0][-1]
            uk = control[ci,1:3]
            # log control
            uLog[logCounter-1,:] = control[ci,1:3]
            # process measurement
            dt = state[0]-stateLast[0]
            # propagate
            EKF.propagateOde(dt)
            # update
            EKF.update(state[0],state[1:3],measurement,measurementGradient,Rkin)
            # log
            xLog[logCounter,:] = EKF.xhat.copy()
            Plog[logCounter,:] = np.diag(EKF.Pk.copy())
            Pfull[logCounter,:,:] = EKF.Pk.copy()
            Ylog[logCounter,:] = state.copy()
            #print('%g,%g,%g,%g,%g|%g,%g' % (state[0],EKF.xhat[0],EKF.xhat[1],EKF.u[0],EKF.u[1],state[1],state[2]))
            stateLast = state
        else:
            rejectCount=rejectCount+1
            gpsOutliers.append(k)
            print("%d bad gps" % rejectCount)
    except IndexError:
        # out of range
        LEN = logCounter-1
        xLog = xLog[:LEN,:]
        Plog = Plog[:LEN,:]
        Pfull = Pfull[:LEN,:,:]
        Ylog = Ylog[:LEN,:]
        uLog = uLog[:LEN,:]
        print("Breaking at index %d" % (k))
        break
print("%d bad gps" % rejectCount)
gpsInliers = np.setdiff1d(np.arange(0,len(gps)),gpsOutliers)
# compute the velocity magnitude and heading, assuming velocity along the direction of heading
V = np.sqrt( np.sum(np.power(xLog[:,2:4],2.0),axis=1))
hdg = np.arctan2(xLog[:,3],xLog[:,2])
# put heading on 0 to 2pi
for k in range(LEN):
    if hdg[k] < 0.0:
        hdg[k] = hdg[k]+2.0*math.pi
# transform state residuals to unit variance
vk = np.zeros((LEN,2))
for k in range(LEN):
    vk[k,:] = np.dot(sp.linalg.sqrtm(np.linalg.inv(Pfull[k,0:2,0:2])),xLog[k,0:2]-Ylog[k,1:3])
# plot
fig = plt.figure()
ax = []

# write data to file
fout = open(path+'/jerkOut.csv','w+')
fout.write('time(sec),X(m),Y(m),VX(m/s),VY(m/s),AX(m/s^2),AY(m/s^2),Xm(m),Ym(m),Vm(m),Hdgm(rad),u_rudder,u_throttle')
for k in range(6):
    for j in range(6):
        fout.write(',P_%d%d' % (k+1,j+1))
fout.write('\n')
# write data
for k in range(LEN):
    fout.write('%f,%f,%f,%f,%f,%f,%f' % (Ylog[k,0],xLog[k,0],xLog[k,1],xLog[k,2],xLog[k,3],xLog[k,4],xLog[k,5]))
    fout.write(',%f,%f,%f,%f' % (Ylog[k,1],Ylog[k,2],Ylog[k,3],Ylog[k,4]))
    fout.write(',%f,%f' % (uLog[k,0],uLog[k,1]))
    for kk in range(6):
        for j in range(6):
            fout.write(',%f' % (Pfull[k,kk,j]))
    fout.write('\n')
fout.close()
print("Wrote %d lines to file" % (LEN))

lbls = ['x(m)','y(m)','vx(m/s)','vy(m/s)','ax(m/s^2)','ay(m/s^2)','ex(m)','ey(m)']
for k in range(8):
    ax.append(fig.add_subplot(4,2,k+1,ylabel=lbls[k]))
    if k < 6:
        ax[k].plot(Ylog[:,0],xLog[:,k])
        ax[k].plot(Ylog[:,0],xLog[:,k]+3.0*np.sqrt(Plog[:,k]),'r--')
        ax[k].plot(Ylog[:,0],xLog[:,k]-3.0*np.sqrt(Plog[:,k]),'r--')
        if k < 2:
            # plot the measurement
            ax[k].plot(Ylog[:,0],Ylog[:,k+1],'k:d')
    elif k < 8: # plot GPS errors
        ax[k].plot(Ylog[:,0],xLog[:,k-6]-Ylog[:,k-5])
        ax[k].plot(Ylog[:,0],3.0*np.sqrt(Plog[:,k-6]),'r--')
        ax[k].plot(Ylog[:,0],-3.0*np.sqrt(Plog[:,k-6]),'r--')
    ax[k].grid()
fig.show()

fig2 = plt.figure()
ax = []
lbls = ['V(m/s)','hdg(rad)','ERR(v)','ERR(hdg)']
plty = np.array([V,hdg]).transpose()
for k in range(4):
    ax.append(fig2.add_subplot(2,2,k+1,ylabel=lbls[k]))
    if k < 2:
        ax[k].plot(Ylog[:,0],plty[:,k])
        ax[k].plot(Ylog[:,0],Ylog[:,k+3],'r--')
        ax[k].set_ylim([plty[:,k].min(),plty[:,k].max()])
    elif k < 4:
        ax[k].plot(Ylog[:,0],plty[:,k-2]-Ylog[:,k+1])
        ax[k].set_ylim([-plty[:,k-2].max(),plty[:,k-2].max()])
    ax[k].grid()
fig2.show()

fig3 = plt.figure()
ax = []
ax.append(fig3.add_subplot(1,2,1,ylabel='x_1'))
vkp = np.nonzero(np.abs(vk[:,0]) <= 3.0)[0]
(n,bins,patches) = plt.hist(vk[vkp,0],bins=25)
# plot normal PDF
x = np.linspace(-3.0,3.0,100.0)
pdf = 1/math.sqrt(2.0*math.pi)*np.exp(-0.5*np.power(x,2.0))
ax[0].plot(x,pdf*LEN*np.mean(np.diff(bins)),'r--')

ax.append(fig3.add_subplot(1,2,2,ylabel='x_2'))
vkp = np.nonzero(np.abs(vk[:,1]) <= 3.0)[0]
(n,bins,patches) = plt.hist(vk[vkp,1],bins=25)
ax[1].plot(x,pdf*LEN*np.mean(np.diff(bins)),'r--')
#fig3 = plt.gcf()
fig3.show()

raw_input('Return to close')

plt.close(fig)
plt.close(fig2)
plt.close(fig3)
