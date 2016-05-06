## @file
#
# Filter dynamic equations of motion and constants

import numpy as np
import math

## 1-sigma standard error in the GPS readings in meters
sigma_gps = 3.0/3.0# meters, 1-sigma
## 1-sigma assumed value for the jerk process noise
sigma_jerk = 0.5
## Measurement covariance matrix
Rkin = np.diag([math.pow(sigma_gps,2.0),math.pow(sigma_gps,2.0)])

## Propagate the equations of motion
# x[0:1] = position
# x[2:3] = velocity
# x[4:5] = acceleration
def propagate(x,t,u):
    dx = np.zeros(6)
    dx[0:2] = x[2:4]
    dx[2:4] = x[4:6]
    dx[4:6] = 0.0
    return dx

def propGradient(x,t,u):
    F = np.zeros((6,6))
    F[0,2] = 1.0;
    F[1,3] = 1.0
    F[2,4] = 1.0
    F[3,5] = 1.0
    return F

def processMatrix(x,t,u):
    G = np.zeros((6,2))
    G[4:6,:] = np.eye(2)
    return G

def measurement(x,t):
    y = x[0:2].copy()
    return y

def measurementGradient(x,t):
    H=np.zeros((2,6))
    H[0:2,0:2]=np.eye(2)
    return H
