/*
 * Activity 01 -- Move it!
 */ 

#include <Arduino.h>
#include <wpi-32u4-lib.h>

#include <Chassis.h>

// Declare a chassis object with nominal dimensions
Chassis chassis(7.0, 1440, 14.9);

void setup() 
{
  // This will initialize the Serial at a baud rate of 115200 for prints
  // Be sure to set your Serial Monitor appropriately
  Serial.begin(115200);

  // initialize the chassis (which also initializes the motors)
  chassis.init();

  Serial.println("/setup()");
}

/*
 * The main loop for the program. The loop function is repeatedly called
 * after setup() is complete.
 */
void loop()
{
  delay(100);
  chassis.printEncoderCounts();
}
