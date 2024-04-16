/* Mobile Robot ESPNOW Communication Data Structures

For using the ESPNOW protocol define the datastructures to be leveraged here

*/

typedef struct 
{
    u_int8_t w_right;               // Right motor angular velocity in pulses per second
    u_int8_t w_left;                // Left motor angular velocity in pulses per second
    u_int8_t ultrasonic_left;       // Left ultrasonic sensor distance value in centimeters
    u_int8_t ultrasonic_center;     // Center ultrasonic sensor distance value in centimeters
    u_int8_t ultrasonic_right;      // Right ultrasonic sensor distance value in centimeters
    u_int8_t counter;               // Counter for debugging
} mobile_robot_state_info_t;

typedef struct 
{
    u_int8_t w_right_cmd;           // Right motor commanded angular velocity in pulses per second 
    u_int8_t w_left_cmd;            // Left motor commanded angular velocity in pulses per second 
} mobile_robot_command_t;
