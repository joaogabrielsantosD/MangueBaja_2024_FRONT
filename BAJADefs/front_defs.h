#ifndef FRONT_DEFS_
#define FRONT_DEFS_

#ifndef MBED
    #define MBED
    #include "mbed.h"
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
/* Radio definitions */
#define NETWORK_ID          101
#define BOXRADIO_ID1        69
#define BOXRADIO_ID2        70
#define MB1_ID              11
#define MB2_ID              22
#define FREQUENCY_915MHZ    91
#define NORMAL_THRESHOLD    68

typedef enum {
    IDLE_ST,
    IMU_ST,
    RPM_ST,
    RADIO_ST,
    THROTTLE_ST,
    FLAGS_ST,
    DISPLAY_ST,
    DEBUG_ST

} state_t;

typedef struct {
    
  uint16_t speed;
  uint16_t rpm;
  uint16_t battery;
  uint16_t level;
  uint16_t temp_cvt;
  uint16_t temp_motor;
  uint16_t sot;

} Txtmng;

#endif
