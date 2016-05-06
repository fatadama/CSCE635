## @file Test the EKF by passing logged data through an xbee_bridge_state object
#

import xbee_bridge_state
import numpy as np
import math
import matplotlib.pyplot as plt
import time

## directory of the log file to use
# parse raw GPS messages for lols
logdir = '20160506_094356/'
#logdir = '20160506_093329/'

def main():
    # log file of GPS
    logged = np.genfromtxt(logdir+'gpsLog.csv',delimiter=',',skiprows=1)
    # control log: time, rudder, throttle
    control = np.genfromtxt(logdir+'controlLog.csv',delimiter=',',skiprows=1)
    # log of  the EKF state
    logged2 = np.genfromtxt(logdir+'bridgeStateLog.csv',delimiter=',',skiprows=1)
    # create state with no log out
    state = xbee_bridge_state.xbee_bridge_state()
    # rescale the GPS velocity
    logged[:,4] = logged[:,4]/0.514444
    # fix the GPS heading degrees -> radians
    logged[:,5] = logged[:,5]*math.pi/180.0
    # convert the GPS heading to -pi to pi
    for k in range(len(logged)):
        if logged[k,5] > math.pi:
            logged[k,5] = logged[k,5]-2.0*math.pi
        if logged[k,5] < -math.pi:
            logged[k,5] = logged[k,5]+2.0*math.pi
    # convert the GPS raw log to X-Y, assume first waypoint is home
    gpsHome = logged[0,2:4]
    gpsXY = np.zeros((len(logged),2))
    for k in range(len(logged)):
        dlon = logged[k,2]-gpsHome[0]
        dlat = logged[k,3]-gpsHome[1]
        x = dlat*1.0e-7*xbee_bridge_state.GPS_D2R_RE
        y = dlon*1.0e-7*xbee_bridge_state.GPS_D2R_RE
        gpsXY[k,0] = x
        gpsXY[k,1] = y
    # log to array
    xlog = np.zeros((len(logged),7))
    plog = np.zeros((len(logged),6,6))
    # log the filter state
    statelog = np.zeros((len(logged),4))
    counter = 0
    tNow = time.time()
    for k in range(len(logged)):
        lon = logged[k,2]
        lat = logged[k,3]
        v=logged[k,4]/0.514444
        h=logged[k,5]*math.pi/180.0
        t=logged[k,1]
        state.update(tNow,lon,lat,t,v,h)
        #print(t,state.filterState)
        statelog[k,:] = state.filterState.copy()
        xlog[k,0] = state.EKF.t
        xlog[k,1:7] = state.EKF.xhat.copy()
        plog[k,:,:] = state.EKF.Pk.copy()
        #print("%8.1f,%f,%f,%f,%f" % (t,1.0e-7*(lon-logged[0,2]),1.0e-7*(lat-logged[0,3]),state.filterState[0],state.filterState[1]))
        if k < len(logged)-1:
            # sleep for time
            dt = logged[k+1,0]-logged[k,0]
            if dt > 0:
                tNow = tNow+dt
                pass
                #time.sleep(dt)
        counter = k
    if counter < len(xlog):
        xlog=xlog[:counter,:]
        plog=plog[:counter,:]
        statelog=statelog[:counter,:]
    fig = plt.figure()
    ax = []
    lbl=['X(m)','Y(m)','Vx(m/s)','Vy(m/s)','Ax(m/s2)','Ay(m/s2)']
    for k in range(6):
        ax.append( fig.add_subplot(3,2,k+1,ylabel=lbl[k]) )
        if k < 2:
            ax[k].plot(logged[:,1],gpsXY[:,k],'k-')
            #ax[k].plot(logged2[:,0],logged2[:,k+1],'r--')#plot what the logged filter did
            ax[k].set_ylim([xlog[:,k+1].min(),xlog[:,k+1].max()])
            ax[k].set_xlim([xlog[:,0].min(),xlog[:,0].max()])
        ax[k].plot(xlog[:,0],xlog[:,k+1])
        ax[k].grid()

    fig.show()

    V = np.sqrt(np.sum(np.power(xlog[:,3:5],2.0),axis=1))
    hdg = np.arctan2(xlog[:,4],xlog[:,5])
    fig2 = plt.figure()
    ax = []
    lbl=['V(m/s)','hdg(rad)','throttle']
    for k in range(3):
        ax.append( fig2.add_subplot(3,1,k+1,ylabel=lbl[k]) )
        if k == 0:
            ax[k].plot(logged[:,1],logged[:,4],'k-')
        elif k == 1:
            ax[k].plot(logged[:,1],logged[:,5],'k-')
        if k < 2:
            ax[k].plot(xlog[:,0],statelog[:,k+2])
            ax[k].set_ylim([statelog[:,k+2].min(),statelog[:,k+2].max()])
            ax[k].set_xlim([xlog[:,0].min(),xlog[:,0].max()])
        else:
            ax[k].plot(control[:,0],control[:,2])
        ax[k].grid()

    fig2.show()

    raw_input('Return to close')
    plt.close(fig)
    plt.close(fig2)

    return

if __name__=="__main__":
    main()
