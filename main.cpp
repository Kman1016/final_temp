#include "mbed.h"
#include "bbcar.h"


#include "drivers/DigitalOut.h"

#include "erpc_simple_server.hpp"
#include "erpc_basic_codec.hpp"
#include "erpc_crc16.hpp"
#include "UARTTransport.h"
#include "DynamicMessageBufferFactory.h"
#include "bbcar_control_server.h"

#include "TextLCD.h"
I2C i2c_lcd(D14, D15);// SDA, SCL
//TextLCD_SPI lcd(&spi_lcd, p8, TextLCD::LCD40x4);   // SPI bus, 74595 expander, CS pin, LCD Type
TextLCD_I2C lcd(&i2c_lcd, 0x4E, TextLCD::LCD16x2);   // I2C bus, PCF8574 Slaveaddress, LCD Type
                                                     //TextLCD_I2C lcd(&i2c_lcd, 0x42, TextLCD::LCD16x2, TextLCD::WS0010);
                                                     // I2C bus, PCF8574 Slaveaddress, LCD Type, Device Type
                                                     //TextLCD_SPI_N lcd(&spi_lcd, p8, p9);
                                                     // SPI bus, CS pin, RS pin, LCDType=LCD16x2, BL=NC, LCDTCtrl=ST7032_3V3
//TextLCD_I2C_N lcd(&i2c_lcd, ST7032_SA, TextLCD::LCD16x2, NC, TextLCD::ST7032_3V3);
// I2C bus, Slaveaddress, LCD Type, BL=NC, LCDTCtrl=ST7032_3V3
// main() runs in its own thread in the OS
ep::UARTTransport uart_transport(D1, D0, 9600);
ep::DynamicMessageBufferFactory dynamic_mbf;
erpc::BasicCodecFactory basic_cf;
erpc::Crc16 crc16;
erpc::SimpleServer rpc_server;

Ticker servo_ticker;
Ticker servo_feedback_ticker;

PwmIn servo0_f(D9), servo1_f(D10);
PwmOut servo0_c(D11), servo1_c(D12);
BBCar car(servo0_c, servo0_f, servo1_c, servo1_f, servo_ticker, servo_feedback_ticker);

BusInOut qti_pin(D4,D5,D6,D7);
DigitalInOut pin8(D8);

parallax_qti qti1(qti_pin);
parallax_laserping  ping1(pin8);

BBCarService_service remote_control_service;

Thread t_DeterDist, t_prepare, t_erpc;

//BufferedSerial serdev(D1, D0, 9600);


bool ControlMode = 0;
int npattern, pattern, QTiValue;
double lastspeed = 40, speed = 20, MaxSpeed = 45, minSpeed = -30;
float LaserPingValue;


void eRPCmain();
void stop(uint8_t cars);
void goStraight(uint8_t cars, int32_t  speed);
void turn(uint8_t cars, int32_t speed, double factor);
void RemoteControlAction(int mode, int value);
void RemoteTuneSpeed(double max, double min);
void RemoteShowSpeed();
void RemoteShowPattern();
void ctl_mode();

void sampling();

void DeterDist();
int check(int errorDistance_Range);
void spin(float distance);
void Prepare();
void AutoMode();


int main() {

    t_DeterDist.start(DeterDist);
    //t_prepare.start(Prepare);
    t_erpc.start(AutoMode);

    // Initialize the rpc server
    uart_transport.setCrc16(&crc16);

  printf("Initializing server.\n");
  rpc_server.setTransport(&uart_transport);
  rpc_server.setCodecFactory(&basic_cf);
  rpc_server.setMessageBufferFactory(&dynamic_mbf);

  // Add the led service to the server
  printf("Adding LCD server.\n");
  rpc_server.addService(&remote_control_service);

  // Run the server. This should never exit
  
  
    printf("Running server.\n");
    rpc_server.run();

    
}

//////////////////////////////// fuction ////////////////////////////////
void sampling() {
    while (1) {
        QTiValue = (int)qti1;
        LaserPingValue = ping1;
        ThisThread::sleep_for(1ms);
    }
}


void AutoMode() {
    int lastPattern = (int)qti1;;
    int pat;
    car.goStraight(50);
    while (!ControlMode) {
        pattern = (int)qti1;

        speed = pattern == lastPattern? lastspeed + 3: lastspeed -2;
        speed = speed > 45? 45 : speed;
        speed = speed < -20? -20: speed;
        // bright 0
        printf("%f  %d\n",speed, pattern);
        
        switch (pattern) {
                // turn right
                case 0b0111: car.turn(speed, -0.001); break;
                case 0b0011: car.turn(speed, -0.08); break;
                case 0b0010: car.turn(speed, -0.05); break;
                case 0b0001: car.turn(speed, -0.05); break;

                case 0b0110: car.goStraight(speed); break;
                // turn left
                case 0b1000: car.turn(speed, 0.05); break;
                case 0b0100: car.turn(speed, 0.05); break;
                case 0b1100: car.turn(speed, 0.08); break;
                case 0b1110: car.turn(speed, 0.001); break;
                case 0b0000: 
                        car.goStraight(28);
                        break;

                case 0b1010:    
                        {
                        car.goStraight(28);
                        ThisThread::sleep_for(10ms);
                        car.turn(35, -0.001);
                        ThisThread::sleep_for(10ms);
                        car.turn(40, 0.1);
                        ThisThread::sleep_for(10ms);
                        }
                        break;
                case 0b0101: 
                        {
                        car.goStraight(28);
                        ThisThread::sleep_for(10ms);
                        car.turn(35, 0.001);
                        ThisThread::sleep_for(10ms);
                        car.turn(40, 0.1);
                        ThisThread::sleep_for(10ms);
                        
                        }
                        break;
                //case 0b1101:
                //case 0b1011:
                // slit 
                case 0b1001: 
                    lcd.cls();
                    lcd.printf("Y\n");
                    car.stop();
                    ThisThread::sleep_for(10ms);
                    car.turn(35, 0.001);
                    ThisThread::sleep_for(10ms);

                default: car.goStraight(40);
            }
        lastPattern = pattern;
        ThisThread::sleep_for(10ms);
    }    
}
 
void DeterDist() {
    int angles0[2] = {0};
    int angles1[2] = {0};
    int cnt = 0;
    int ang_temp0, ang_temp1;
    float dist1, dist0;
    int pat = int(qti1);
    while (1) {
        pat = int(qti1);
        ang_temp0 = car.servo0.angle;
        ang_temp1 = car.servo1.angle;
        if (pat == 15) {
            printf("%d: %d\n",cnt++ ,pat);
            lcd.printf("%d cnt: %d\n",pat, cnt);
            angles0[1] = angles0[0];
            angles0[0] = ang_temp0;
            angles1[1] = angles1[0];
            angles1[0] = ang_temp1;
            if ((angles0[1] != 0 && angles0[0] != 0) || (angles1[1] != 0 && angles1[0] != 0)) {
                dist0 = abs(angles0[0] - angles0[1])*6.5*3.14159/360;
                dist1 = abs(angles1[0] - angles1[1])*6.5*3.14159/360;
                if (dist0>dist1) {
                    printf("%f\n",dist0);
                    lcd.cls();
                    lcd.printf("dist: %d\n",(int)dist0);
                    ThisThread::sleep_for(1s);
                    
                    }
                else{
                    printf("%f\n",dist1);
                    lcd.cls();
                    lcd.printf("dist: %d\n",(int)dist0);
                    ThisThread::sleep_for(1s);
                    
                }
            }
            ThisThread::sleep_for(2s);
        }
        ThisThread::sleep_for(1ms);
    }
}


void spin(float distance) {
    car.servo0.targetAngle = (int)(distance*360/(6.5*3.14)) + car.servo0.angle;
    car.servo1.targetAngle = (int)(distance*360/(6.5*3.14)) + car.servo1.angle;
    while (check(1)) {
        ThisThread::sleep_for(1ms);
    }
    car.stop();
}

int check(int errorDistance_Range){
    int speed, offset;                                                              // Control system variables
    float errorDistance, factor=1;                 

    errorDistance = (car.servo0.targetAngle - car.servo0.angle)*6.5*3.14/360;       // Calculate error
    
    speed = int(errorDistance);

    if(errorDistance > 0)                        // Add offset
        offset = 40;
    else if(errorDistance < 0)
        offset = -40;
    else
        offset = 0;    

    car.servo0.set_factor(factor);
    car.servo1.set_factor(factor);
    car.servo0.set_speed(speed + offset);  
    car.servo1.set_speed(speed + offset);

    if ( abs(errorDistance) > (errorDistance_Range) )
        return 1;   
    else return 0;
}

void ctl_mode() {
    if (!ControlMode)
        ControlMode = 1;
    else
        ControlMode = 0;
    printf("Mode: %d\n", ControlMode);
    lcd.printf("Mode: %d\n", int(ControlMode));
    ThisThread::sleep_for(1s);
    lcd.cls();
}

void eRPCmain() { 
  // Initialize the rpc server
  uart_transport.setCrc16(&crc16);

  printf("Initializing server.\n");
  rpc_server.setTransport(&uart_transport);
  rpc_server.setCodecFactory(&basic_cf);
  rpc_server.setMessageBufferFactory(&dynamic_mbf);

  // Add the led service to the server
  printf("Adding LCD server.\n");
  rpc_server.addService(&remote_control_service);

  // Run the server. This should never exit
  
  
    printf("Running server.\n");
    rpc_server.run();

}

void RemoteShowPattern() {
    printf("pattern: %d\n",(int)qti1);
    lcd.printf("pattern: %d\n", int(qti1));
    ThisThread::sleep_for(1s);
    lcd.cls();
}

void RemoteShowSpeed() {
    printf("speed: %f\n",speed);
    lcd.printf("speed: %d\n", int(speed));
    ThisThread::sleep_for(1s);
    lcd.cls();
}

void RemoteTuneSpeed(double max, double min) {
    MaxSpeed = max;
    minSpeed = min;
}

void RemoteControlAction(uint8_t mode, int32_t value, double factor) {
    ControlMode = 1;
    switch (mode) {
        case 1:
            car.goCertainDistance(value);
            break;
        case 2:
            spin(10*3.14159*value/360);
            break;
        case 3:
            car.turn(value, factor);
            break;
        case 4:
            car.goStraight(speed);
        case 5:
            car.stop();
    }
    ThisThread::sleep_for(100ms);
    
}
void stop(uint8_t cars){
    if (ControlMode) {
        car.stop();
        printf("Car %d stop.\n", cars);
    }
    
}

void goStraight(uint8_t cars, int32_t  speed){
    if (ControlMode) {
        car.goStraight(speed);
        printf("Car %d go straight at speed %d.\n", cars, speed);
    }
}

void turn(uint8_t cars, int32_t speed, double factor){
    if (ControlMode) {
        car.turn(speed, factor);
        printf("Car %d turn at speed %d with a factor of %f.\n", cars, speed, factor);
    }
}