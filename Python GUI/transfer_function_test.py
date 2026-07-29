import numpy as np
import scipy.signal
import control as ct
import matplotlib.pyplot as plt

Motor_Electrical_tf = ct.TransferFunction([1],[2.2, 0.005])
Motor_Mechanical_tf = ct.TransferFunction([1],[0.05, 0.5])
Motor_Plant_tf = (Motor_Electrical_tf * Motor_Mechanical_tf) / (1 + Motor_Electrical_tf * Motor_Mechanical_tf)
print(f"Motor Plant Transfer Function = {Motor_Plant_tf}")
print(f"Poles = {Motor_Plant_tf.poles()} Zeros = {Motor_Plant_tf.zeros}")

# Transient Response
step1 = ct.step_response(Motor_Plant_tf)
plt.plot(step1.time, step1.outputs)
plt.show()

# Frequency Response
ct.bode_plot(Motor_Plant_tf,dB=True, Hz=True)
plt.show()

# Pole Zero Plot
ct.pole_zero_plot(Motor_Plant_tf)
plt.show()