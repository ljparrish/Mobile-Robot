import numpy as np
import matplotlib.pyplot as plt
import math


class StateEstimator:
    def __init__(self, wheel_radius, wheel_base, ticks_per_rev=1120):
        self.wheel_radius = wheel_radius 
        self.wheel_base = wheel_base
        self.ticks_per_rev = ticks_per_rev
    
    def estimateState(self, prev_state, encoder_data):
        x, y, theta = prev_state
        dist_l = (encoder_data[0]/self.ticks_per_rev) * (2 * math.pi * self.wheel_radius)
        dist_r = (encoder_data[1]/self.ticks_per_rev) * (2 * math.pi * self.wheel_radius)
        d = (dist_l+dist_r)/2

        delta_theta = (dist_r-dist_l)/self.wheel_base
        print(f"dist_l: {dist_l}, dist_r: {dist_r}, d: {d}, delta_theta: {delta_theta}")

        x += d * np.cos(theta+delta_theta/2)
        y += d * np.sin(theta+delta_theta/2)
        theta += delta_theta

        return (x, y, theta)


if __name__ == '__main__':

    se = StateEstimator(3, 20, 1120)

    encoder_left = [0, 1, 2, 3, 4, 5, 6, 7, 8, 5, 5, 5, 5, 5]
    encoder_right = [0, 1, 2, 3, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5]

    i = 0
    prevState = (0, 0, 0)
    x, y = [], []

    while i < len(encoder_left) and i < len(encoder_right):

        encoder_data = (encoder_left[i], encoder_right[i])

        prevState = se.estimateState(prevState, encoder_data)
        x.append(prevState[0])
        y.append(prevState[1])

        i += 1
    
    for j in range(i):
        print(f"(x, y) = {(x[j], y[j])}")

    
    plt.scatter(x, y, s= 1)
    plt.show()