/***********************************************************************
 * EduFun Rover Experiment 2: DriveForward
 * 
 * Drive EduFun Rover forward for two second and stop.
 * 
 * Double check the hardware to ensure:
 * - the motors is connected
 * - the battery pack is connected to the controller
 * - the power switch must be in 'Om' position
 *
 ***********************************************************************/

#include <EduFunRover.h>  // to use the EduFun Rover library in your sketch.

// Instantiate the EduFunRover object. This only needs to be done once.
EduFunRover eduFunRover; 

void setup()
{
    //Move right motor forward at full speed.
    // 1 to 255     : Forward (Maximum power: 255)
    // 0            : Stop
    // -1 to -255   : Reverse (Maximum power: -255)
    eduFunRover.rightMotor(255); 
    
    //Move left motor forward at full speed.
    // 1 to 255     : Forward (Maximum power: 255)
    // 0            : Stop
    // -1 to -255   : Reverse (Maximum power: -255)
    eduFunRover.leftMotor(255);
        
    delay(2000);       // Waits for 2 seconds
    eduFunRover.stop();       // Stops both motors
    
    delay(2000);       // Waits for 2 seconds
    
    //Reverse the right motor at full speed.
    // 1 to 255     : Forward (Maximum power: 255)
    // 0            : Stop
    // -1 to -255   : Reverse (Maximum power: -255)
    eduFunRover.rightMotor(-255); 
    
    //Reverse the left motor full speed.
    // 1 to 255     : Forward (Maximum power: 255)
    // 0            : Stop
    // -1 to -255   : Reverse (Maximum power: -255)
    eduFunRover.leftMotor(-255);
    
    delay(2000);       // Waits for 2 seconds
    eduFunRover.stop();       // Stops both motors
}

void loop()
{
  // Nothing here. We'll get to this in the next experiment.
}


