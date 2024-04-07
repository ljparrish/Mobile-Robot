#include <stdio.h>
#include <math.h>

#define PI 3.14159265
#define WHEEL_BASE 20
#define WHEEL_DIAMETER 6
#define TICKS_PER_REVOLUTION 1120


void estimate_state(double prev_state[3], double encoder_data[2]) {

    double dist_l = (encoder_data[0]/TICKS_PER_REVOLUTION) * (PI * WHEEL_DIAMETER);
    double dist_r = (encoder_data[1]/TICKS_PER_REVOLUTION) * (PI * WHEEL_DIAMETER);
    double d = (dist_l+dist_r)/2;

    double delta_theta = (dist_r-dist_l)/WHEEL_BASE;

    prev_state[0] += d * cos(prev_state[2]+delta_theta/2);
    prev_state[1] += d * sin(prev_state[2]+delta_theta/2);
    prev_state[2] += delta_theta;
}

int main() {

    int encoder_left[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 5, 5, 5, 5, 5};  // 14 elements
    int encoder_right[] = {0, 1, 2, 3, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5};  // 14 elements

    int i = 0;
    double prevState[3] = {0, 0, 0};
    double x[14], y[14];

    while (i < sizeof(encoder_left) / sizeof(encoder_left[0]) && i < sizeof(encoder_right) / sizeof(encoder_right[0])) {
    
        double encoder_data[] = {encoder_left[i], encoder_right[i]};

        estimate_state(prevState, encoder_data);
        x[i] = prevState[0];
        y[i] = prevState[1];

        i++;
    }

    // Print the result
    for (int j = 0; j < i; j++) {
        printf("(x, y) = (%lf, %lf)\n", x[j], y[j]);
    }

    return 0;
}
