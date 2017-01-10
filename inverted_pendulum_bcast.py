# single pendulum formula translated from the C code at
# http://www.physics.usyd.edu.au/~wheat/dpend_html/solve_dpend.c

from numpy import sin, cos
import numpy as np
import matplotlib.pyplot as plt
import scipy.integrate as integrate
import matplotlib.animation as animation

from InvpStateMsg import *
import socket
import math

class InvpStateBroadcaster(object):

    bcast_addr = ('127.0.0.1',24777)

    def __init__(self):
        self.s=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    def sendState(self, state_str):
        self.s.sendto(state_str.encode('utf-8'),self.bcast_addr)


# In ver 1.0, the equations of motion follows the model defined in the paper "Event-Based Control of the Inverted Pendulum"
# However, this paper has several significant errors/typos in Eq.(1), which results in some unmatched result with the controller design
# In ver 2.0, I corrected the errors by looking at another paper Andrew K. Stimac's "Standup and Stabilization of the Inverted Pendulum"
# In this version, ver 3.0, try to convert the batch processing to discrete-time events with time stamps in an event queue following the model of TESim
# Ver 3.a, both stabilization and swing-up controllers are employed, and the force is bounded by n (However, the control interval does not work because 
# the calculation of new Force is within the ODE function, as the Force will be updated when the states are updated within ODE. Therefore, the control frequency
# is higher than we designed.
# In Ver 3.b, I solved this problem by introducing constant force in the state vector and update the force outside the ODE. In this way, we can manipulate the control 
# frequency. 

G = 9.8  # acceleration due to gravity, in m/s^2
M = 2.57 # mass of cart, in kg
m = 1.47 # mass of pendulum, in kg
L = 0.28 # length of pendulum center to the pivot point, in m 
I = 0.024 # the moment of inertia wrt the pivot point, in kg*m^2
c = 1e-3  # friction force of the floor
b = 1e-3  # friction coeffience of the pivot
ro = 3 # old setting in ver 1.0, convert from voltage

K1 = M+m
K2 = I + m*L*L 
F=0

nf = 1.5 # the scaling weight of the Force to the cart
ke = 0.3 # the scaling weight to calculate the force  

thetaTh = np.radians(15.0)

# state := (theta, theta', x, x', F) where theta is positive counter-clockwisely from the upright vertical direction, 
# x is positive to the right horizontally, F is the force applied to the cart
# This is the motion of equations to calculate the next state for the pendulum-cart system
# The calculation of the force, i.e., the controlled process variable, should be out of this function because otherwise this value will vary in the ODE according to other PVs
# Therefore, we can put this "constant" PV as additional state variable while keep its differential as ZERO
def derivs(state, t):

    dydx = np.zeros_like(state)
        
    #print('start printing time')
    #print('In ODE, t= ', t, ' F= ', state[4])  
    
    lambda1 = state[4] - c*state[3] + m*L*sin(state[0])*state[1]*state[1]
    lambda2 = -1*b*state[1] + m*G*L*sin(state[0])
    K3 = 1*m*L*cos(state[0])
    K4 = K1*K2 - K3*K3
    dydx[0] = state[1]
    dydx[1] = (K1*lambda2 - K3*lambda1)/K4
    dydx[2] = state[3]
    dydx[3] = (K2*lambda1 - K3*lambda2)/K4
    dydx[4] = 0
    
    return dydx

    
def controller(state):
    F = 0 # initialized the output, i.e., the force. F=0 means an uncontrolled system
    #return F
    if cos(state[0]) >= cos(thetaTh): # around the upright unstable equilibrium, LQR stabilization controller
        #F = 1595.3*state[0] + 216*state[1] + 15.8*state[2] + 13.5*state[3]  # with R=3e-3, Q=diag([4 0 0.75 0])
        #F = 1987.5*state[0] + 269.6*state[1] + 36.5*state[2] + 23.9*state[3]  # with R=3e-3, Q=diag([4 0 4 0])
        F = 3112.9*state[0] + 423.1*state[1] + 115.5*state[2] + 56.2*state[3]  # with R=3e-4, Q=diag([4 0 4 0])
    else: # swing-up energy controller
        E = 0.5*I*state[1]*state[1] + m*G*L*(cos(state[0])-1)
        u = ke*E*np.sign(state[1]*cos(state[0]))
        F = (M+m)*u 
    
    # #bound the Force
    if F < -1*nf*G*(M+m):
        F = -1*nf*G*(M+m)
    if F > nf*G*(M+m):
        F = nf*G*(M+m)
    
    return F

# Main()
# initial state
# th1 and th2 are the initial angles (degrees)
# w10 and w20 are the initial angular velocities (degrees per second)
# not set the two conditions at the same time: 1) th = 180 and 2) thp = 0
th = 90.0
thp = 0.0
x = 0
xp =0.0 
state = [np.radians(th), np.radians(thp), x, xp, 0] # state at t = 0
F0 = controller(state)
state[4] = F0
print('init state: ', state)

# create a time array from 0..numT-1 sampled at dt second steps
dt = 0.02 # time interval length
numT = 1000001  
timestamps = [i*dt for i in range(numT)] # time sequence: 0, dt, 2dt, ..., (numT-1)dt
output = np.zeros((numT, 6)) # each row contains [time, theta, theta', x, x', F]

t = np.arange(0.0, 2*dt, dt) # This time sequence is used in ODE solver for the next state in dt, i.e., t = [0 dt]

bcast = InvpStateBroadcaster()

for i in range(numT):
    #print('timestamp=', timestamps[i])
    output[i][0] = timestamps[i]
    for j in range(5):
        output[i][j+1] = state[j]
    #print('entering ODE')    
    sol = integrate.odeint(derivs, state, t)
    #print('i=', i, 'ODE F= ', sol[1, 4])
    state = sol[1, :] # new state will be recorded in the next update moment
    Fn = controller(state)
    state[4] = Fn
    #print(' Cal F= ', Fn)

    # transmit the state information
    if not (i%250):
        msg = InvpStateMsg(output[i,0],output[i,3],output[i,1])
        print(msg)
        msg = msg.tojson()
        bcast.sendState(msg)
    

