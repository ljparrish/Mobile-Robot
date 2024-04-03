"""
Using python to test algorithm first
(The file will be removed/changed to C)

Current Issue

Given   encoder_left = [0] + [5] * 30
        encoder_right = [0] + [5] * 30 
Expected motion: continue moving straight (in the plot, to the x direction)

But because our displacement becomes zero when both delta (left side and right side) are zero,
position doesn't get updated as we desired.

"""

import numpy as np
import matplotlib.pyplot as plt


def estimateState(wheel_radius, wheelbase, cumulative_theta, position, prevEncoder, newEncoder):

    # Reading two consecutive encoder readings
    delta_encoder_left = newEncoder[0] - prevEncoder[0]
    delta_encoder_right = newEncoder[1] - prevEncoder[1] 

    # Convert delta encoder counts to linear distances traveled by each wheel
    distance_left = delta_encoder_left * wheel_radius
    distance_right = delta_encoder_right * wheel_radius

    # Calculate the change in orientation (delta_theta) using differential kinematics
    delta_theta = (distance_right - distance_left) / wheelbase

    # Update cumulative orientation
    cumulative_theta += delta_theta

    # Calculate linear displacement of the robot
    displacement = (distance_left + distance_right) / 2.0

    # Update position of the robot
    position[0] += displacement * np.cos(cumulative_theta)  # Update x position
    position[1] += displacement * np.sin(cumulative_theta)  # Update y position
    print(f"Disp: {displacement}, Cumul Theta: {cumulative_theta}")

    return position, cumulative_theta


if __name__ == '__main__':
    wheelRadius, wheelbase = 5, 20
    # encoder_left = list(range(1, 10)) + [10] * 19
    # encoder_right = list(range(1, 10)) + [10] * 10 + [9, 8, 7, 6, 5, 4, 3, 2, 1]
    encoder_left = [0] + [5] *30
    encoder_right = [0] + [5] *30

    i = 1
    pos, theta = [0,0], 0
    estimated_x, estimated_y = [0], [0]

    while i < len(encoder_left) and i < len(encoder_right):

        prevEncoder = (encoder_left[i-1], encoder_right[i-1])
        newEncoder = (encoder_left[i], encoder_right[i])

        pos, theta = estimateState(wheelRadius, wheelbase, theta, pos, prevEncoder, newEncoder)
        estimated_x.append(pos[0])
        estimated_y.append(pos[1])

        print(f"Estimated Point: {pos}\n")
        print()
        i += 1
    
    plt.scatter(estimated_x, estimated_y, s= 1)
    plt.show()