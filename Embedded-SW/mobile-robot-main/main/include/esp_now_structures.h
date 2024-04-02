/* Mobile Robot ESPNOW Communication Data Structures

For using the ESPNOW protocol define the datastructures to be leveraged here

*/

typedef struct 
{
    float x_position;           // Robot X Position in Odometry Frame
    float y_position;           // Robot Y Position in Odometry Frame
    float theta;                // Robot Heading angle in Odometry Frame
    int w_right;                // Right motor angular velocity in pulses per second
    int w_left;                 // Left motor angular velocity in pulses per second
    float ultrasonic_left;      // Left ultrasonic sensor distance value
    float ultrasonic_center;    // Center ultrasonic sensor distance value
    float ultrasonic_right;     // Right ultrasonic sensor distance value
} mobile_robot_state_info_t;