import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from filterpy.kalman import KalmanFilter as KF
from filterpy.common import Q_discrete_white_noise

# Input Parameters and import data

dt = 0.05 # seconds

Data = pd.read_csv('MotorData.csv')
npData = Data.to_numpy()

position = npData[20:-1,0]
velocity = npData[20:-1,1]

if(1):
    position += np.random.normal(0, 0.2, position.shape)

# Initialize Kalman Filter
x0 = np.array([position[0], velocity[0]])

f = KF(dim_x=2, dim_z=1)

f.x = x0

f.F = np.array([[1.0, dt],
                [0.0, 1.0]])

f.H = np.array([[1.0, 0.0]])

f.P *= 0

f.R = 0.0000001

f.Q = Q_discrete_white_noise(dim=2, dt=0.05, var=0.013)

p_hat = []
v_hat = []
v_diff = []

for k in range(len(position)):
    z = position[k]
    f.predict()
    f.update(np.array(z))
    p_hat.append(f.x[0])
    v_hat.append(f.x[1])
    v_diff.append((position[k] - position[k-1])/dt)

t = np.linspace(0,len(position)*dt,num=len(position))

plt.plot(t,position)
plt.plot(t,p_hat)
plt.show()
plt.plot(t,velocity)
plt.plot(t,v_hat) 
plt.plot(t[0:len(v_diff)],v_diff)
plt.show()