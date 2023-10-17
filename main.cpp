/* IDLE_ST E DEBUG_ST obrigatorios
    * REAR: TEMP_MOTOR_ST, FUEL_ST, RPM_ST, THROTTLE_ST, RADIO_ST
    * FRONT: SLOWACQ_ST, IMU_ST, SPEED_ST, THROTTLE_ST, DISPLAY_ST
    * BMU: Voltage_ST, TEMP_CVT_ST, SystemCurrent_ST
*/

/* Novos:
    * REAR:  TEMP_MOTOR_ST, FUEL_ST, TEMP_CVT_ST, SPEED_ST, SystemCurrent_ST, Voltage_ST, THROTTLE_ST
    * FRONT: THROTTLE_ST, RADIO_ST, IMU_ST, RPM_ST, FLAGS_ST, DISPLAY_ST
*/
#include "mbed.h"
//#include "stats_report.h"
/* User libraries */
#include "defs.h"
#include "front_defs.h"
#include "FIR.h"
#include "CANMsg.h"
#include "LSM6DS3.h"
#include "Kalman.h"
#include "RFM69.h"
//#include <cstdint>

//#define MB1   // Uncomment this line if MB1
#define MB2     // Uncomment this line if MB2

#ifdef MB1
    #define NODE_ID MB1_ID
#endif

#ifdef MB2
    #define NODE_ID MB2_ID
#endif

/* Communication protocols */
CAN can(PB_8, PB_9, 1000000);
Serial serial(PA_2, PA_3, 115200);
LSM6DS3 LSM6DS3(PB_7, PB_6); 
//RFM69(PinName mosi, PinName miso, PinName sclk, PinName slaveSelectPin, PinName interrupt)
RFM69 radio(PB_15, PB_14, PB_13, PB_12, PA_8); 
/* I/O pins */
InterruptIn freq_sensor(PB_4, PullNone);
InterruptIn choke_switch(PA_6, PullUp);     // servomotor CHOKE mode
InterruptIn run_switch(PA_5, PullUp);       // servomotor RUN mode
DigitalOut led(PC_13);
/* Debug pins */
PwmOut signal(PA_7);
DigitalOut db(PB_11);

/* Mbed OS tools */
Thread eventThread;
EventQueue queue(1024);
CircularBuffer<state_t, BUFFER_SIZE> state_buffer;
CircularBuffer<imu_t*, 20> imu_buffer;
Ticker ticker1Hz;
Ticker ticker5Hz;
Ticker ticker20Hz;
Ticker tickerTrottle;
Timeout debounce_throttle;

/* Debug variables */
Timer t;
bool buffer_full = false;
unsigned int t0, t1;
/* Global variables */
FIR filter;
Txtmng strc_data;
packet_t data;
state_t current_state = IDLE_ST;
uint8_t imu_failed = 0;                  // number of times before a new connection attempt with imu 
uint8_t pulse_counter = 0, sot = 0;
int16_t angle_roll = 0, angle_pitch = 0; 
uint16_t dt = 0;
uint16_t array_data[sizeof(Txtmng)];
uint32_t imu_last_acq = 0;
uint64_t last_acq = 0;
uint64_t current_period = 0, last_count = 0;
float rpm_hz;
const float a=0.6, b=0.6;
bool switch_clicked = false;

/* Interrupt handlers */
void canHandler();
void throttleDebounceHandler();
/* Interrupt services routine */
void canISR();
void frequencyCounterISR();
void servoSwitchISR();
void ticker1HzISR();
void ticker5HzISR();
void ticker20HzISR();
/* General functions*/
void setupInterrupts();
void initRadio();
void filterMessage(CANMsg msg);
void calcAngles(int16_t accx, int16_t accy, int16_t accz, int16_t grx, int16_t gry, int16_t grz, int16_t dt);
void Servo_flag(uint8_t state);
void displayData(uint16_t vel, uint16_t Hz, uint16_t temp, uint16_t comb, uint16_t tempCVT, uint16_t SOC, uint16_t SOT);

/* CAN Variables */
uint8_t switch_state = 0x00; // THROTTLE_ID = 1by
//imu -> acc=6by(2by+2by+2by) -> dps=6y(2by+2by+2by)

int main ()
{
    /* Main variables */
    CANMsg txMsg; 
    /* Initialization */
    t.start();
    //horn = horn_button.read();                               // horn OFF
    //headlight = headlight_switch.read();                          // headlight OFF
    eventThread.start(callback(&queue, &EventQueue::dispatch_forever));
    t0 = t.read_us();
    uint16_t lsm_addr = LSM6DS3.begin(LSM6DS3.G_SCALE_245DPS, LSM6DS3.A_SCALE_2G, LSM6DS3.G_ODR_26_BW_2, LSM6DS3.A_ODR_26); 
    t1 = t.read_us();
    //serial.printf("%d\r\n", (t1 - t0));
    setupInterrupts();
    initRadio();

    while(true)
    {
        if(state_buffer.full())
        {
            buffer_full = true;
            //led = 0;
            state_buffer.pop(current_state);
        } else {
            //led = 1;
            buffer_full = false;
            if(!state_buffer.empty())
                state_buffer.pop(current_state);
            else
                current_state = IDLE_ST;
        }

        //serial.printf("current state = %d", current_state);
        //if(current_state==0) serial.printf("IDLE_ST\n");
        //if(current_state==1) serial.printf("IMU_ST\n");
        //if(current_state==2) serial.printf("RPM_ST\n");
        //if(current_state==3) serial.printf("RPM_ST\n");
        //if(current_state==4) serial.printf("THROTTLE_ST\n");
        //if(current_state==5) serial.printf("RADIO_ST\n");
        //if(current_state==6) serial.printf("DEBUG_ST\n");

        switch(current_state) 
        {
            case IDLE_ST:
                //serial.printf("idle\r\n");
                //Thread::wait(2);
                break;

            case IMU_ST:
                //serial.printf("imu\r\n");
                t0 = t.read_us();

                if(lsm_addr)
                {
                    bool nack = LSM6DS3.readAccel();     // read accelerometer data into LSM6DS3.aN_raw
                    if(!nack)
                        nack = LSM6DS3.readGyro();       //  "   gyroscope data into LSM6DS3.gN_raw

                    if(nack)
                    {
                        lsm_addr = 0;
                        LSM6DS3.ax_raw = 0;
                        LSM6DS3.ay_raw = 0;
                        LSM6DS3.az_raw = 0;
                        LSM6DS3.gx_raw = 0;
                        LSM6DS3.gy_raw = 0;
                        LSM6DS3.gz_raw = 0;
                    }
                }

                else if(imu_failed == IMU_TRIES)
                {
                    lsm_addr = LSM6DS3.begin(LSM6DS3.G_SCALE_245DPS, LSM6DS3.A_SCALE_2G, LSM6DS3.G_ODR_26_BW_2, LSM6DS3.A_ODR_26);                                    
                    t1 = t.read_us();
                    imu_failed = 0;
                    //serial.printf("%d\r\n", (t1 - t0));
                } else {
                    imu_failed++;
                }

                last_acq = t.read_ms();
                calcAngles(LSM6DS3.ax_raw, LSM6DS3.ay_raw, LSM6DS3.az_raw, LSM6DS3.gx_raw, LSM6DS3.gy_raw, LSM6DS3.gz_raw, dt);

                data.imu.acc_x = LSM6DS3.ax_raw;
                data.imu.acc_y = LSM6DS3.ay_raw;
                data.imu.acc_z = LSM6DS3.az_raw;

                data.imu.dps_x = LSM6DS3.gx_raw;
                data.imu.dps_y = LSM6DS3.gx_raw;
                data.imu.dps_z = LSM6DS3.gx_raw;

                /* Send accelerometer data */
                txMsg.clear(IMU_ACC_ID);
                txMsg << data.imu.acc_x << data.imu.acc_y << data.imu.acc_z;
                if(can.write(txMsg))
                {
                    /* Send gyroscope data only if accelerometer data succeeds */
                    imu_last_acq = t.read_ms();

                    txMsg.clear(IMU_DPS_ID);
                    txMsg << data.imu.dps_x << data.imu.dps_y << data.imu.dps_z << dt;
                    can.write(txMsg);
                }

                break;

            case RPM_ST:
                //serial.printf("rpm\r\n");
                freq_sensor.fall(NULL);         // disable interrupt

                if(current_period!=0) 
                {
                    //rpm_hz = ((float)pulse_counter/(current_period/1000000.0));    //calculates frequency in Hz
                    //if (switch_state != CHOKE_MODE)
                    rpm_hz = pulse_counter*5*60;
                        Servo_flag(CHOKE_MODE);
                    //engine_counter.start();
                } else {
                    rpm_hz = 0;
                    //writeServo(switch_state);
                    //engine_counter.stop();
                }

                data.rpm = (uint16_t)(filter.filt(rpm_hz, a, b)/2);

                /* Send rpm data */
                txMsg.clear(RPM_ID);
                txMsg << data.rpm;
                can.write(txMsg);

                /* prepare to re-init rpm counter */
                pulse_counter = 0;
                current_period = 0;                                   // reset pulses related variables
                last_count = t.read_us();
                freq_sensor.fall(&frequencyCounterISR);               // enable interrupt

                break;

            case RADIO_ST:
                //serial.printf("ra\n");
                imu_t* temp_imu;
                /*if(radio.receiveDone()) 
                {
                    led = !led;
                    serial.printf("Received from TNODE: %d ", radio.SENDERID);
                    serial.printf((char *)radio.DATA);
                    if(radio.ACKRequested())
                    {
                        theNodeID = radio.SENDERID;
                        radio.sendACK();
                        serial.printf(" - ACK sent. Receive RSSI: %d\r\n", radio.RSSI);
                    } 
                    else 
                        serial.printf("Receive RSSI: %d\r\n",radio.RSSI);
                }*/
                //dbg4 = 1;
                //if(radio.receiveDone())
                //{
                //if (radio.ACKRequested())
                //radio.sendACK();
                //led = 0;
                //}

                //serial.printf("%d,%d,%d\r\n", (!imu_buffer.empty()), (!d10hz_buffer.empty()), (!temp_buffer.empty()));
                //if((!imu_buffer.empty()) && (!d10hz_buffer.empty()) && (!temp_buffer.empty()))
                //{
             
                    //serial.printf("Radio state\n");
                //led = !led;

                    //(radio.canSend()) ? serial.printf("É possivel mandar\n\r->") : serial.printf("Não é possivel mandar\n\r---");

                if(!imu_buffer.empty()) 
                {
                    //serial.printf("imu buffer\n");
                    imu_buffer.pop(temp_imu);
                    memcpy(&data.imu, temp_imu, sizeof(imu_t));
                    //data.rpm = ((uint16_t)rpm_hz * 60)*65536.0/5000.0;
                    data.timestamp = t.read_us();
                    #ifdef MB1
                        if(!radio.sendWithRetry((uint8_t)BOXRADIO_ID1, &data, sizeof(packet_t), true))
                            db = !db; /*serial.printf("ok with ack\n");*/
                        //else
                        //    serial.printf("ok without ack\n"); 
                        //radio.send((uint8_t)BOXRADIO_ID1, &data, sizeof(packet_t), true, false);     // request ACK with 1 retry (waitTime = 40ms)
                    #endif

                    #ifdef MB2
                        if(!radio.sendWithRetry((uint8_t)BOXRADIO_ID2, &data, sizeof(packet_t), true))
                            db = !db; /*serial.printf("ok with ack\n");*/
                        //else
                        //    serial.printf("ok without ack\n"); 
                        //radio.send((uint8_t)BOXRADIO_ID2, &data, sizeof(packet_t), true, false);     // request ACK with 1 retry (waitTime = 40ms)
                        //serial.printf("entrou no primeito if!!\n\n");
                    #endif
                } 
            
                else if(t.read_ms() - imu_last_acq > 500) 
                {
                    //serial.printf("not imu buffer acq\n");
                    memset(&data.imu, 0, sizeof(imu_t));
                    //data.rpm = ((uint16_t)rpm_hz * 60)*65536.0/5000.0;
                    #ifdef MB1
                        if(!radio.sendWithRetry((uint8_t)BOXRADIO_ID1, &data, sizeof(packet_t), true))
                            db = !db; /*serial.printf("ok with ack\n");*/
                        //else
                        //    serial.printf("ok without ack\n");
                        //radio.send((uint8_t)BOXRADIO_ID1, &data, sizeof(packet_t), true, false);     // request ACK with 1 retry (waitTime = 40ms)
                    #endif

                    #ifdef MB2
                        //radio.send((uint8_t)BOXRADIO_ID2, &data, sizeof(packet_t), true, false);     // request ACK with 1 retry (waitTime = 40ms)
                        if(!radio.sendWithRetry((uint8_t)BOXRADIO_ID2, &data, sizeof(packet_t), true))
                            db = !db; /*serial.printf("ok with ack\n");*/
                        //else
                        //    serial.printf("ok without ack\n");
                    #endif
                }
                //}
                //radio.receiveDone();
                //dbg4 = 0;
                //tim2 = t.read_us() - tim1;
                //serial.printf("%d\r\n",tim2);

                break;

            case THROTTLE_ST:
                //serial.printf("throttle ok\r\n");

                if(switch_clicked)
                {                    
                    switch_state = (!choke_switch.read() << 1) | (!run_switch.read() << 0);
                    //serial.printf("switch_state = %d\r\n", switch_state);
                    
                    /* Send CAN message */
                    txMsg.clear(THROTTLE_ID);
                    if(data.rpm!=0)
                    {
                        Servo_flag(CHOKE_MODE);
                        txMsg << CHOKE_MODE;
                    } else {
                        Servo_flag(switch_state);
                        txMsg << switch_state;
                    }
                    can.write(txMsg);

                    //serial.printf("can ok\r/n");                  // append data (8 bytes max)
                    //if(can.write(txMsg)) led = !led;
                    //ThisThread::sleep_for(300);
                    
                    switch_clicked = false;
                }

                break;

            case FLAGS_ST:
                //serial.printf("flags\r\n");   
                (data.fuel < 18) ? data.flags |= (0x10) : data.flags &= ~(0x10); // (00010000)

                /* Send flags message */
                txMsg.clear(FLAGS_ID);
                txMsg << data.flags;
                can.write(txMsg);
            
                break;

            case DISPLAY_ST:
                //serial.printf("display\r\n");  
                displayData(data.speed, data.rpm, data.tempMOTOR, data.fuel, data.tempCVT, data.soc, sot);
                break;

            case DEBUG_ST:
                //serial.printf("Debug state\r\n");
                //serial.printf("Accx = %d\r\n", LSM6DS3.ax_raw);
                //serial.printf("Accy = %d\r\n", LSM6DS3.ay_raw);
                //serial.printf("Accz = %d\r\n", LSM6DS3.az_raw);
                //serial.printf("DPSx = %d\r\n", LSM6DS3.gx_raw);
                //serial.printf("DPSy = %d\r\n", LSM6DS3.gy_raw);
                //serial.printf("DPSz = %d\r\n", LSM6DS3.gz_raw);
                //serial.printf("RPM = %d\r\n", data.rpm);
                //serial.printf("switch state = %d", switch_state);
                break;
        }
    }
}

/* Global Functions */ 
void setupInterrupts()
{
    can.attach(&canISR, CAN::RxIrq);
    freq_sensor.fall(&frequencyCounterISR);
    choke_switch.rise(&servoSwitchISR);     // trigger throttle interrupt in both edges
    choke_switch.fall(&servoSwitchISR);     // trigger throttle interrupt in both edges
    run_switch.rise(&servoSwitchISR);       // trigger throttle interrupt in both edges
    run_switch.fall(&servoSwitchISR);       // trigger throttle interrupt in both edges
    ticker1Hz.attach(&ticker1HzISR, 1.0);
    ticker5Hz.attach(&ticker5HzISR, 0.2); // T = 1/5hz = 0.2 s
    ticker20Hz.attach(&ticker20HzISR, 0.05); // T = 1/20hz = 0.05 s
    signal.period_ms(1000);
    signal.write(0.5f);
}

void initRadio()
{
    if(!radio.initialize(FREQUENCY_915MHZ, NODE_ID, NETWORK_ID))
        data.flags |= (0x08); // (00001000) enable radio flag
    radio.encrypt(0);
    radio.setPowerLevel(31);
    radio.setHighPower();
}

void filterMessage(CANMsg msg)
{
    led = !led;

    if(msg.id==SPEED_ID)
    {
        msg >> data.speed;
    }

    if(msg.id==TEMPERATURE_ID)
    {
        msg >> data.tempMOTOR;
        data.flags |= (data.tempMOTOR > 110) ? 0x01 << 5 : 0; //(00100000)
    }

    if(msg.id==FUEL_ID)
    {
        msg >> data.fuel;
    }

    if(msg.id==SOC_ID)
    {
        msg >> data.soc;
        (data.soc < 20) ? data.flags |= (0x80) : 0;
    }

    if(msg.id==CVT_ID)
    {
        msg >> data.tempCVT;
        data.flags |= (data.tempCVT > 110) ? 0x01 << 6 : 0;  //(01000000)
    }

    if(msg.id==VOLTAGE_ID)
    {
        msg >> data.voltage;
    }

    if(msg.id==LAT_ID)
    {
        msg >> data.latitude;
    }

    if(msg.id==LNG_ID)
    {
        msg >> data.longitude;
    }

    if(msg.id==SOT_ID)
    {
        msg >> sot;
    }
}

/* Function adapted from Kristian Lauszus library example, source: https://github.com/TKJElectronics/KalmanFilter */
void calcAngles(int16_t accx, int16_t accy, int16_t accz, int16_t grx, int16_t gry, int16_t grz, int16_t dt)
{
    static Kalman kalmanX, kalmanY;
    float kalAngleX, kalAngleY;
    float pitch, roll;
    float gyroXrate, gyroYrate;
    float ax, ay, az;
    static bool first_execution = true;
    
    ax = (float) accx * TO_G;
    ay = (float) accy * TO_G;
    az = (float) accz * TO_G;
    pitch = atan2(ay, az) * RAD_TO_DEGREE;
    roll = atan(-ax / sqrt(ay * ay + az * az)) * RAD_TO_DEGREE;
    gyroXrate = grx / TO_DPS;                            // Convert to deg/s
    gyroYrate = gry / TO_DPS;                            // Convert to deg/s

    if(first_execution)
    {
        // set starting angle if first execution
        first_execution = false;
        kalmanX.setAngle(roll);
        kalmanY.setAngle(pitch);
    }
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90))
    {
        kalmanX.setAngle(roll);
        kalAngleX = roll;
    } else {
        kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
    }

    if(abs(kalAngleX) > 90)
    {
        gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
    }
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);

    angle_roll = roll;
    angle_pitch = pitch;
}

void Servo_flag(uint8_t state)
{
    data.flags &= ~(0x07); // (00001000) reset servo-related flags

    switch(state) 
    {
        case MID_MODE:
            //dbg3 = !dbg3;
            //servo.pulsewidth_us(SERVO_MID);
            data.flags &= ~(SERVO_RESET); // (00001000) reset run and choke flags
            break;

        case RUN_MODE:
            //dbg3 = !dbg3;
            //servo.pulsewidth_us(SERVO_RUN);
            data.flags |= RUN_MODE; // (00000001) set run flag
            break;

        case CHOKE_MODE:
            //dbg3 = !dbg3;
            //servo.pulsewidth_us(SERVO_CHOKE);
            data.flags |= CHOKE_MODE;    // (00000010) set choke flag
            break;

        default:
            //serial.printf("Choke/run error\r\n");
            data.flags |= SERVOR_ERROR;    // (00000100) set servo error flag
            break;
    }
}

void displayData(uint16_t vel, uint16_t Hz, uint16_t temp, uint16_t comb, uint16_t tempCVT, uint16_t SOC, uint16_t SOT)
{
    //db = !db; 
    
    strc_data.speed = vel;
    strc_data.rpm = Hz;
    strc_data.battery = SOC;
    strc_data.level = comb;
    strc_data.temp_motor = temp;
    strc_data.temp_cvt = tempCVT;
    strc_data.sot = sot;

    memcpy(array_data, (uint16_t *)&strc_data, sizeof(strc_data));

    for(uint8_t CountBytes = 0; CountBytes < sizeof(Txtmng); CountBytes++) 
    {
        serial.putc(array_data[CountBytes]);
    }
}

/* Interrupts routine */
void canISR()
{
    CAN_IER &= ~CAN_IER_FMPIE0;                 // disable RX interrupt
    queue.call(&canHandler);                    // add canHandler() to events queue
}

void frequencyCounterISR()
{
    //db = !db;
    pulse_counter++;
    current_period += t.read_us() - last_count;
    last_count = t.read_us();
}

void servoSwitchISR()
{
    choke_switch.rise(NULL);     //  throttle interrupt in both edges dettach
    choke_switch.fall(NULL);     //  throttle interrupt in both edges dettach
    run_switch.rise(NULL);       //  throttle interrupt in both edges dettach
    run_switch.fall(NULL);       //  throttle interrupt in both edges dettach
    switch_clicked = true;
    debounce_throttle.attach(&throttleDebounceHandler, 0.1);
}

void ticker1HzISR()
{
    state_buffer.push(FLAGS_ST);
}

void ticker5HzISR()
{
    state_buffer.push(RPM_ST);
    state_buffer.push(RADIO_ST);
}

void ticker20HzISR()
{
    state_buffer.push(IMU_ST);
}

/* Interrupts handlers */
void canHandler()
{
    CANMsg rxMsg;

    can.read(rxMsg);
    filterMessage(rxMsg);
    CAN_IER |= CAN_IER_FMPIE0;                  // enable RX interrupt
}

void throttleDebounceHandler()
{
    state_buffer.push(THROTTLE_ST);
    choke_switch.rise(&servoSwitchISR);     // trigger throttle interrupt in both edges
    choke_switch.fall(&servoSwitchISR);     // trigger throttle interrupt in both edges
    run_switch.rise(&servoSwitchISR);       // trigger throttle interrupt in both edges
    run_switch.fall(&servoSwitchISR);       // trigger throttle interrupt in both edges
}
