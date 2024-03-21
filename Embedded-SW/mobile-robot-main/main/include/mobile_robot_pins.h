/*  Mobile Robot Pinout

    This header file contains signal names defined by the mobile robot adapter board schematic.
    Note: Names are exactly as written on the schematic.
*/

#include <stdio.h>
#include "sdkconfig.h"
#include "driver/gpio.h"

// Motor I/O
#define MOTOR_L_A GPIO_NUM_18
#define MOTOR_L_B GPIO_NUM_17
#define MOTOR_R_A GPIO_NUM_14
#define MOTOR_R_B GPIO_NUM_8

// Encoder I/O
#define ENC_L_A GPIO_NUM_16
#define ENC_L_B GPIO_NUM_15
#define ENC_R_A GPIO_NUM_38
#define ENC_R_B GPIO_NUM_39

// Ultrasonic Sensors
#define TRIG_1 GPIO_NUM_11
#define TRIG_2 GPIO_NUM_10
#define TRIG_3 GPIO_NUM_9
#define TRIG_4 GPIO_NUM_6
#define TRIG_5 GPIO_NUM_5

#define ECHO_1 GPIO_NUM_37
#define ECHO_2 GPIO_NUM_35
#define ECHO_3 GPIO_NUM_36
#define ECHO_4 GPIO_NUM_13
#define ECHO_5 GPIO_NUM_12

// Neopixel LED
#define NEOPIXEL_POWER GPIO_NUM_21
#define NEOPIXEL_ADDRESS GPIO_NUM_33