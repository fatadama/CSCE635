import numpy as np
import matplotlib.pyplot as plt
import math
import sys
sys.path.append('../../../estimation/filters/python/ekf')
import ekf

# define filter functions
## Propagation for dynamics
def propagate(x,t,u):
    dx = np.zeros(6)
    dx[0] = x[2]*math.cos(x[3])#r1
    dx[1] = x[2]*math.sin(x[3])#r2
    dx[2] = x[5]*u[1]#v
    dx[3] = x[2]*x[4]*math.tan(u[0])#psi
    dx[4] = 0.0#K_psi
    dx[5] = 0.0#K_t
    return dx
## x[0] = r1, x[1] = r2, x[2] = V, x[3] = PSI, x[4] = K_psi, x[5] = K_t
def propagateGradient(x,t,u):
    F = np.zeros((6,6))
    F[0,2] = math.cos(x[3])
    F[1,2] = math.sin(x[3])
    F[3,2] = x[4]*math.tan(u[0])
    F[0,3] = x[2]*-math.sin(x[3])
    F[1,3] = x[2]*math.cos(x[3])
    F[2,5] = u[1]
    F[3,4] = x[2]*math.tan(u[0])
    return F

def processNoiseMatrix(x,t,u):
    G = np.zeros((6,2))
    G[4:6,:] = np.eye(2)
    return G

def measurementFunction(x,t):
    y = x[0:4]
    return y

def measurementGradient(x,t):
    H = np.zeros((4,6))
    H[0:4,0:4] = np.eye(4)
    return H

# state: GPS time, X, Y, speed, heading (rad) - used for outlier rejection
def acceptGps(state,stateLast):
    if state[0] < stateLast[0]:
        # backward in time - reject
        return False
    if abs(state[0]-stateLast[0]) > 100.0:
        # time parse error - reject
        return False
    if np.abs( np.sqrt(np.sum(np.power(state[1:3],2.0)))-np.sqrt(np.sum(np.power(stateLast[1:3],2.0))) ) > 10.0:
        # moved more than 10 meters: reject
        return False
    if abs(state[3]-stateLast[3]) > 10.0:
        # speed changed too much
        return False
    if abs(state[4]-stateLast[4]) > math.pi:
        # heading changed too much
        return False
    if abs(state[4]) > 2.0*math.pi:
        # headiing invalid
        return False
    if abs(state[3]) > 100.0:
        # velocity bad
        return False
    return True

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

# 1-sigma in the process noise for heading gain
sigma_psi = 0.5
sigma_t = 0.5
sigma_gps = 0.5*3.0/3.0# meters, 1-sigma
sigma_gps_v = 0.5/3.0#m/s
sigma_gps_h = 0.5/3.0#rads

Qkin = np.diag([sigma_psi*sigma_psi,sigma_t*sigma_t])
Rkin = np.diag([sigma_gps*sigma_gps,sigma_gps*sigma_gps,sigma_gps_v*sigma_gps_v,sigma_gps_h*sigma_gps_h])
EKF = ekf.ekf(6,2,propagate,propagateGradient,processNoiseMatrix,Qk=Qkin)

# initialize filter
xk0 = np.array([gpsXY[0,0],gpsXY[0,1],gps[0,4],gps[0,5],2.0,2.0])
Pk0 = np.diag([sigma_gps*sigma_gps,sigma_gps*sigma_gps,0.5,0.5,sigma_psi*sigma_psi,sigma_t*sigma_t])
EKF.init_P(xk0,Pk0,gps[0,0])
# previous value of GPS time, X, Y, speed, heading (rad) - used for outlier rejection
stateLast = np.array([gps[0,0],gpsXY[0,0],gpsXY[0,1],gps[0,4],gps[0,5]])
# count bad GPS
rejectCount = 0
# log the state
LEN = 600
xLog = np.zeros((LEN,6))
xLog[0,:] = EKF.xhat.copy()
Plog = np.zeros((LEN,6))
Plog[0,:] = np.diag(EKF.Pk.copy())
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
    k = k+1
    # for each measurement: determine if reject or not
    state = np.array([gps[k,0],gpsXY[k,0],gpsXY[k,1],gps[k,4],gps[k,5]])
    if acceptGps(state,stateLast):
        logCounter = logCounter+1
        # find the control index before the current index
        ci = np.nonzero(control[:,0] <= state[0])[0][-1]
        uk = control[ci,1:3]
        # set the control
        EKF.u = uk.copy()
        # process measurement
        dt = state[0]-stateLast[0]
        # propagate
        EKF.propagateOde(dt)
        # make the heading state minimize error to the measurement
        while( EKF.xhat[3]-state[4] > math.pi):
            EKF.xhat[3] = EKF.xhat[3]-2.0*math.pi
        while( EKF.xhat[3]-state[4] < -math.pi):
            EKF.xhat[3] = EKF.xhat[3]+2.0*math.pi
        # update
        EKF.update(state[0],state[1:5],measurementFunction,measurementGradient,Rkin)
        # log
        xLog[logCounter,:]=EKF.xhat.copy()
        # make heading be on 0 to 2pi
        while(xLog[logCounter,3] > 2.0*math.pi):
            xLog[logCounter,3] = xLog[logCounter,3]-2.0*math.pi
        while(xLog[logCounter,3] < 0.0):
            xLog[logCounter,3] = xLog[logCounter,3]+2.0*math.pi
        Plog[logCounter,:] = np.diag(EKF.Pk.copy())
        Ylog[logCounter,:] = state.copy()
        uLog[logCounter,:] = uk.copy()
        #print('%g,%g,%g,%g,%g|%g,%g' % (state[0],EKF.xhat[0],EKF.xhat[1],EKF.u[0],EKF.u[1],state[1],state[2]))
    else:
        rejectCount=rejectCount+1
        gpsOutliers.append(k)
        print("%d bad gps" % rejectCount)
    stateLast = state
print("%d bad gps" % rejectCount)
gpsInliers = np.setdiff1d(np.arange(0,len(gps)),gpsOutliers)
# difference the GPS to estimate the velocity and heading
gpsDiff = np.zeros((len(Ylog),2))
gpsV = np.diff(Ylog[:,1:3],axis=0)
# derive heading purely from the change in position
gpsDiff[1:,1] = np.arctan2(gpsV[:,1],gpsV[:,0])
# now scale by time to get speed
gpsV[:,0] = gpsV[:,0]/np.diff(Ylog[:,0],axis=0)
gpsV[:,1] = gpsV[:,1]/np.diff(Ylog[:,0],axis=0)
gpsDiff[1:,0] = np.sqrt(np.sum(np.power(gpsV,2.0),axis=1))
gpsDiff[1:,1] = np.arctan2(gpsV[:,1],gpsV[:,0])
# make the headings continuous
'''
for k in range(1,LEN):
    while ( Ylog[k,4]-Ylog[k-1,4] ) > math.pi:
        Ylog[k:,4] = Ylog[k:,4]-math.pi
    while ( Ylog[k,4]-Ylog[k-1,4] ) < -math.pi:
        Ylog[k:,4] = Ylog[k:,4]+math.pi
    while ( xLog[k,3]-xLog[k-1,3] ) > math.pi:
        xLog[k:,3] = xLog[k:,3]-math.pi
    while ( xLog[k,3]-xLog[k-1,3] ) < -math.pi:
        xLog[k:,3] = xLog[k:,3]+math.pi
'''

# plot
fig = plt.figure()
ax = []

lbls = ['x(m)','y(m)','v(m/s)','hdg(rad)','K_psi','K_t','u_r','u_t']
for k in range(8):
    ax.append(fig.add_subplot(4,2,k+1,ylabel=lbls[k]))
    if k < 6:
        ax[k].plot(Ylog[:,0],xLog[:,k])
        ax[k].plot(Ylog[:,0],xLog[:,k]+3.0*np.sqrt(Plog[:,k]),'r--')
        ax[k].plot(Ylog[:,0],xLog[:,k]-3.0*np.sqrt(Plog[:,k]),'r--')
        if k < 4:
            # plot the measurement
            ax[k].plot(Ylog[:,0],Ylog[:,k+1],'k:')
            if k >= 2:
                #ax[k].plot(Ylog[:,0],gpsDiff[:,k-2],'r:')
                pass
    elif k < 8:
        # plot control
        ax[k].plot(Ylog[:,0],uLog[:,k-6])
    ax[k].grid()
fig.show()

raw_input('Return to close')

plt.close(fig)
