/********************************
Main CPP for EduFun Rover control.
*********************************/

#include "EduFunRover.h"
#include <Arduino.h>

//#define SERIAL_DEBUG
//#define BTN_DEBUG


// default constructor
//no serial comm
EduFunRover::EduFunRover()
{
  begin();
}

#if defined(__AVR__) || defined(ESP8266)
EduFunRover::EduFunRover(SoftwareSerial *ss) {
 
    hwSerial = NULL;
    swSerial = ss;
    mySerial = swSerial;
  
}
#endif

EduFunRover::EduFunRover(HardwareSerial *hs) {
#if defined(__AVR__) || defined(ESP8266)
  swSerial = NULL;
#endif
 
    hwSerial = hs;
    mySerial = hwSerial;
}

void EduFunRover::begin(uint32_t baudrate)
{
    if (hwSerial) hwSerial->begin(baudrate);
    
#if defined(__AVR__) || defined(ESP8266)
  if (swSerial) swSerial->begin(baudrate);
#endif
    
    begin();
    
}

void EduFunRover::begin()
{
    pinMode(Dir_R, OUTPUT);
    pinMode(Dir_L, OUTPUT);
    
    Wire.begin(); 
    prevBtnA = 0, prevBtnB = 0, prevBtnC = 0;
}

void EduFunRover::stop()
{
    leftStop();
    rightStop(); 
}


void EduFunRover::rightMotor(int speed)
{
  if (speed > 0)
  {
    rightFwd((byte)speed);
  }
  else
  {
    rightRev((byte)speed);
  }
}

void EduFunRover::rightFwd(byte speed)
{
  digitalWrite(Dir_R, LOW);
  analogWrite(PWM_R, speed);
}

void EduFunRover::rightRev(byte speed)
{
  digitalWrite(Dir_R, HIGH);
  analogWrite(PWM_R, (255 - speed));
}

void EduFunRover::rightStop() 
{
  analogWrite(PWM_R, 0);
}

void EduFunRover::leftMotor(int speed)
{
  if (speed > 0)
  {
      leftFwd((byte)(speed));
  }
  else
  {
    leftRev((byte)(speed));
  }
}

void EduFunRover::leftFwd(byte speed)
{
  digitalWrite(Dir_L, HIGH);
  analogWrite(PWM_L, speed);
}

void EduFunRover::leftRev(byte speed)
{
  digitalWrite(Dir_L, LOW);
  analogWrite(PWM_L, (255 - speed));
}

void EduFunRover::leftStop() 
{
  analogWrite(PWM_L, 0);
}


float EduFunRover::supplyVoltage()
{
    float bat;
    bat = analogRead(BatLevel);
    bat = (bat*2*5)/1024;
    return bat;
}

uint16_t EduFunRover::i2cButtonPkt(uint8_t btn)
{
    Wire.beginTransmission(0x12);
    Wire.write(btn);      //register of button
    Wire.endTransmission();
    
    delay(80); //short delay for camera to respond
    
    Wire.requestFrom(0x12, 2);   // request 6 bytes from slave device #8

    uint8_t c[2];
    while (Wire.available() < 2); // slave may send less than requested
    c[0] = Wire.read();
    c[1] = Wire.read();
   
    uint16_t cntBtn = (uint16_t)c[1];
    cntBtn = (cntBtn << 8) | c[0];
    
    return cntBtn;
}
uint16_t EduFunRover::getButtonA()
{
    uint16_t cntBtnA = i2cButtonPkt(DID_btnA);
    if (cntBtnA == prevBtnA)
        return 0;
    else{
        prevBtnA = cntBtnA;
        return 1;
    }
}

uint16_t EduFunRover::getButtonB()
{
    uint16_t cntBtnB = i2cButtonPkt(DID_btnB);
    if (cntBtnB == prevBtnB)
        return 0;
    else{
        prevBtnB = cntBtnB;
        return 1;
    }
}

uint16_t EduFunRover::getButtonC()
{
    uint16_t cntBtnC = i2cButtonPkt(DID_btnC);
    if (cntBtnC == prevBtnC)
        return 0;
    else{
        prevBtnC = cntBtnC;
        return 1;
    }
}

void EduFunRover::getJoystick(uint16_t *posMsg)
{
    int16_t xVal, yVal;
    uint8_t msgLen = 4;
    
    Wire.beginTransmission(I2CCam);
    Wire.write(DID_JOYSTICK);      //register of button
    Wire.endTransmission();
    
    delay(50); //short delay for camera to respond
    
    Wire.requestFrom(I2CCam, msgLen);   // request 6 bytes from slave device #8
    uint8_t c[msgLen];
    while (Wire.available() < msgLen); // slave may send less than requested
    c[0] = Wire.read();
    c[1] = Wire.read();
    c[2] = Wire.read();
    c[3] = Wire.read();
    
    jsPosX = (uint16_t)c[1];
    jsPosX = (jsPosX << 8) | c[0];
                  
    jsPosY = (uint16_t)c[3];
    jsPosY = (jsPosY << 8) | c[2];
   
    xVal = (int16_t)jsPosX;
    if(xVal > 200)
        xVal = 200;
    if(xVal < -200)
        xVal = -200;
    
     yVal = (int16_t)jsPosY;
    if(yVal > 200)
        yVal = 200;
    if(yVal < -200)
        yVal = -200;
    yVal = map(yVal, -200, 200, 250, -250);
    //xVal = map(xVal, -200, 200, -255, 255);
    
    posMsg[0] = xVal;
    posMsg[1] = yVal;
}

uint8_t EduFunRover::getSerialPackage()
{
    uint8_t revByte[pktSize];
    
#ifdef SERIAL_DEBUG
	Serial.println("receiving package....");
#endif 
   
    if(!mySerial->available()) {
        return 0;
    }
    
    // Read a byte at a time until we get to the special '0x55' start-byte
    if (mySerial->peek() != STX0) {
        mySerial->read();
        return 0;
    } 
    
    // Now read all 14 bytes
    if (mySerial->available() < 14) {
        return 0;
    }
    mySerial->readBytes(revByte, 14);
    
    if(revByte[4] == DID_btnA)
    {
        cntBtnA = (uint16_t)revByte[6];
        cntBtnA = (cntBtnA << 8) | revByte[5];
#ifdef BTN_DEBUG
	Serial.println(cntBtnA, HEX);   
#endif 
    }else if(revByte[4] == DID_btnB)
    {
        cntBtnB = (uint16_t)revByte[6];
        cntBtnB = (cntBtnB << 8) | revByte[5];
    }else if(revByte[4] == DID_btnC)
    {
        cntBtnC = (uint16_t)revByte[6];
        cntBtnC = (cntBtnC << 8) | revByte[5];
    }else if(revByte[4] == DID_JOYSTICK)
    {
        jsPosX = (uint16_t)revByte[6];
        jsPosX = (jsPosX << 8) | revByte[5];
                  
        jsPosY = (uint16_t)revByte[8];
        jsPosY = (jsPosY << 8) | revByte[7];
                  
        jsDistance = (uint16_t)revByte[10];
        jsDistance = (jsDistance << 8) | revByte[9];
        if(jsDistance > 2400)
            jsDistance = 2400;
        
            jsAngle = (uint16_t)revByte[12];
            jsAngle = (jsAngle << 8) | revByte[11];
            jsAngle = (int)(jsAngle / 10);
    }
        return SERIAL_OK;
}

uint8_t EduFunRover::getCRC(uint8_t message[], uint8_t length)
{
  const uint8_t CRC7_POLY = 0x91;
  uint8_t i, j, crc = 0;
  
  for (i = 0; i < length; i++)
  {
    crc ^= message[i];
    for (j = 0; j < 8; j++)
    {
      if (crc & 1)
        crc ^= CRC7_POLY;
      crc >>= 1;
    }
  }
  return crc;
}

