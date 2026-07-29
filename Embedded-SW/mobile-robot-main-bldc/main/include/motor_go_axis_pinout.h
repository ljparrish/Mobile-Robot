/*  Mobile Robot Pinout

    This header file contains signal names defined by the mobile robot adapter board schematic.
    Note: Names are exactly as written on the schematic.
*/

#include <stdio.h>
#include "sdkconfig.h"
#include "driver/gpio.h"

// Motor I/O
#define MOTOR_0_UH GPIO_NUM_3
#define MOTOR_0_UL GPIO_NUM_17
#define MOTOR_0_VH GPIO_NUM_8
#define MOTOR_0_VL GPIO_NUM_16
#define MOTOR_0_WH GPIO_NUM_18
#define MOTOR_O_WL GPIO_NUM_15

#define MOTOR_1_UH GPIO_NUM_41
#define MOTOR_1_UL GPIO_NUM_11
#define MOTOR_1_VH GPIO_NUM_42
#define MOTOR_1_VL GPIO_NUM_10
#define MOTOR_1_WH GPIO_NUM_12
#define MOTOR_1_WL GPIO_NUM_9

// Encoder I/O
#define ENC_0_CS GPIO_NUM_6
#define ENC_1_CS GPIO_NUM_7
#define ENC_SDA GPIO_NUM_4
#define ENC_SCL GPIO_NUM_5

// IMU
#define IMU_SDA GPIO_NUM_13
#define IMU_SCL GPIO_NUM_14

// Status LED
#define STATUS_LED GPIO_NUM_44