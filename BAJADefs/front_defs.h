#ifndef FRONT_DEFS_
#define FRONT_DEFS_

#ifndef MBED
    #define MBED
    #include "mbed.h"
    #include "FIR.h"
#endif

/* IMU convertions */
#define PI                  3.1416
#define RAD_TO_DEGREE       180.0/PI
#define TO_G                2.0/32768.0
#define TO_DPS              245.0/32768.0
#define IMU_TRIES           10
/* SERVO state */ 
#define MID_MODE            0x00
#define RUN_MODE            0x01
#define CHOKE_MODE          0x02
#define SERVO_RESET         0x03
#define SERVOR_ERROR        0x04

typedef enum {
    IDLE_ST,        // wait
    IMU_ST,         // Process the acceleration and angular velocity 
    RPM_ST,         // Process the RPM of Motor
    //RADIO_ST,
    VERIFY_4x4_ST,  // Verify if the 4x4 is enable
    THROTTLE_ST,    // Write throttle position (PWM)
    FLAGS_ST,       // Enable the bit if the condition is true, emergency and logging bits
    DISPLAY_ST,     // Send the data to panel
    DEBUG_ST        // Send data to Debug

} state_t;

typedef struct {
    
  uint16_t speed;
  uint16_t rpm;
  uint8_t battery;
  uint16_t level;
  uint8_t temp_cvt;
  uint8_t temp_motor;
  uint8_t sot;

} Txtmng;

#endif
