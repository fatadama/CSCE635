import sys
import numpy as np
import gpsParser
import scipy.interpolate
import matplotlib.pyplot as plt

## @file
## @package curveFit Does curve fitting to GPS data using splines to estimate radius of curvature.
# @brief Works badly, data are too noisy to get accelerations.
#
# Dependencies gpsParser package, numpy, scipy, matplotlib, python2

# load file: use one of the two cases we're set up for
#file = '20160328172944.csv'
file = '20160328172331.csv'
dat = gpsParser.parseFile(file)

# shorten file to relevant interval (i.e. nonzero speed)
if file == '20160328172944.csv':
    #k1 = np.nonzero((dat[:,0]-dat[0,0] > 322))
    #k2 = np.nonzero((dat[:,0]-dat[0,0] < 375))
    k1 = np.nonzero((dat[:,0]-dat[0,0] > 220))
    k2 = np.nonzero((dat[:,0]-dat[0,0] < 400))
    kp = np.intersect1d(k1[0],k2[0])
    dat = dat[kp,:]
if file == '20160328172331.csv':
    k1 = np.nonzero((dat[:,0]-dat[0,0] > 170))
    k2 = np.nonzero((dat[:,0]-dat[0,0] < 300))
    kp = np.intersect1d(k1[0],k2[0])
    dat = dat[kp,:]

t = dat[:,0]-dat[0,0]
x = dat[:,6]-dat[0,6]
y = dat[:,7]-dat[0,7]

ti = np.linspace(t[0],t[-1],10*len(t))

xtck = scipy.interpolate.splrep(t,x,k=3,s=1.0)
ytck = scipy.interpolate.splrep(t,y,k=3,s=1.0)

# evaluate the spatial derivatives
xs = scipy.interpolate.splev(ti,xtck,der=0)
ys = scipy.interpolate.splev(ti,ytck,der=0)
vx = scipy.interpolate.splev(ti,xtck,der=1)
vy = scipy.interpolate.splev(ti,ytck,der=1)
ax = scipy.interpolate.splev(ti,xtck,der=2)
ay = scipy.interpolate.splev(ti,ytck,der=2)

# calculate curvature
K = np.zeros(len(ti),)
# radius of curvature
R = np.zeros(len(ti),)
for k in range(len(ti)):
    K[k] = (vx[k]*ay[k]-vy[k]*ax[k])/np.power(vx[k]*vx[k]+vy[k]*vy[k],1.5)
    R[k] = 1.0/K[k]

# plot 2D path
mp = plt.figure()
axu = mp.add_subplot(1,1,1,ylabel='y(m)',xlabel='x(m)')
axu.plot(x,y,lw=2)
axu.plot(xs,ys,'r--',lw=2)
mp.show()

# plot time histories
fig = plt.figure()
axi = []
lbls = ['x(m)','y(m)','vx(m/s)','vy(m/s)','ax(m/s2)','ay(m/s2)','curvature (m^-1)','radius of curvature (m)']
for k in range(4):
    axi.append(fig.add_subplot(2,2,k+1,ylabel=lbls[k],xlabel='time(sec)'))
    if k == 0:
        axi[k].plot(t,x,lw=2)
        axi[k].plot(ti,xs,'r--',lw=2)
    elif k == 1:
        axi[k].plot(t,y,lw=2)
        axi[k].plot(ti,ys,'r--',lw=2)
    elif k == 2:
        axi[k].plot(ti,vx,lw=2)
        #axi[k].plot(ti,vxs,'r--')
    elif k == 3:
        axi[k].plot(ti,vy,lw=2)
        #axi[k].plot(ti,vys,'r--')
    elif k == 4:
        axi[k].plot(ti,ax,lw=2)
    elif k == 5:
        axi[k].plot(ti,ay,lw=2)
    elif k == 6:
        axi[k].plot(ti,np.abs(K),lw=2)
    elif k == 7:
        axi[k].plot(ti,np.abs(R),lw=2)

mp.set_size_inches((6.25,9),forward=True)
fig.set_size_inches((6.25,9),forward=True)
fig.show()
#fig.set_size_inches(6.0,4.0,forward=True)
raw_input('Return to continue')
plt.close(fig);
plt.close(mp);
