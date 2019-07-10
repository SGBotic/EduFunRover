/***********************************************************************
 * MagBot Experiment 1: First Test
 * 
 * This sketch is to check the MagBot controller and to ensure the driver  
 * of the serial port is installed correctly.
 *
 ***********************************************************************/

#define LEDPin 13 // On-board LED is connected to pin 13

// setup() function runs once at the very beginning.
void setup()
{
  // Configures the LEDPin as an OUTPUT
  pinMode(LEDPin, OUTPUT); 
}

// loop() function repeats over and over... forever!
void loop()
{
  // Blink sequence
  digitalWrite(LEDPin, HIGH); // Set LEDPin to 5V to turn on LED 
  delay(500);                 // Pause the program for 500 milliseconds
  digitalWrite(LEDPin, LOW);  // Set LEDPin to 0V to turn off LED
  delay(500);                 // Pause the program for 500 milliseconds
}
