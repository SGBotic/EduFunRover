/****************************************************************
MagBot IR reflective line sensors.
****************************************************************/

#include "EduFunRover.h"
#include <Arduino.h>

EduFunRoverSensor::EduFunRoverSensor(int pin)
{
  _pin = pin;

}

int EduFunRoverSensor::read()
{
    int data;

    data = analogRead(_pin);
    
    if(data > 10)
        return HIGH;
    else 
        return LOW;
    
}