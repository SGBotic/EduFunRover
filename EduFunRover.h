#ifndef EduFunRover_h
#define EduFunRover_h

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Wire.h>
//#if defined(__AVR__) || defined(ESP8266)
 // #include <SoftwareSerial.h>
//#endif

// Pin aliases for the motor controller and battery sensor
#define Dir_R       11
#define Dir_L       10
#define PWM_R       6
#define PWM_L       5
#define BatLevel    A7
const uint8_t I2CCam = 0x12;

class EduFunRover
{
  public:
  
    EduFunRover();  // Constructor. Mainly sets up pins.
    EduFunRover(SoftwareSerial *ss);
    EduFunRover(HardwareSerial *hs);
    void begin(uint32_t baudrate);
    void begin();
   
    void rightMotor(int speed); 
	void leftMotor(int speed);  

    void stop();            
    void rightStop();       
    void leftStop();        
    
    float supplyVoltage();             //check battery level
  
    uint16_t getButtonA();
    uint16_t getButtonB();
    uint16_t getButtonC();
    void getJoystick(uint16_t *posMsg);
    
  private:
  
    const uint8_t ESP32Cam= 0x01;
    const uint8_t STX0 = 0x55;
    const uint8_t STX1 = 0x55;
    const uint8_t pktSize = 14;
    const uint8_t DID_btnA = 0x01;
    const uint8_t DID_btnB = 0x02;
    const uint8_t DID_btnC = 0x03;
    const uint8_t DID_JOYSTICK = 0X10;
    const uint8_t SERIAL_TIMEOUT = 0XFF;
    const uint8_t SERIAL_OK = 0X00;
    uint16_t prevBtnA = 0, prevBtnB = 0, prevBtnC = 0;
    uint16_t i2cButtonPkt(uint8_t btn);
    
    uint16_t cntBtnA = 0, cntBtnB = 0, cntBtnC = 0;
    uint16_t jsPosX, jsPosY, jsDistance, jsAngle;
    
    void leftFwd(byte speed); // These functions are pretty self-explanatory,
    void leftRev(byte speed); //  and are called by the above functions once
    void rightFwd(byte speed);//  sign has been used to determine direction.
    void rightRev(byte speed);
    uint8_t  getSerialPackage();
    uint8_t getCRC(uint8_t message[], uint8_t count);
    
     Stream *mySerial;

#if defined(__AVR__) || defined(ESP8266)
  SoftwareSerial *swSerial;
#endif
  HardwareSerial *hwSerial; 
    
};

class EduFunRoverSensor
{
  public:

    EduFunRoverSensor(int pin);
    int read();
    
  private:
    int _pin;
};

#endif