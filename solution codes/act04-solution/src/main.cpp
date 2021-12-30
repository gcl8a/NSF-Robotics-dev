/*
 * Activity 04 -- Where are we headed?
 *
 * Make a delivery using an internal map of the arena.
 */ 

#include <Arduino.h>
#include <wpi-32u4-lib.h>

#include <IRdecoder.h>
#include <ir_codes.h>

#include <Chassis.h>

// TODO: Include Servo32u4 library
#include <Servo32u4.h>

// TODO: Include rangefinder library
#include <Rangefinder.h>

// TODO: Include navigator
#include "delivery.h"

// TODO: Create delivery object
Delivery delivery;

uint16_t darkThreshold = 500;
float speed = 10;

// Declare a chassis object with nominal dimensions
// TODO: Adjust the parameters: wheel diam, encoder counts, wheel track
Chassis chassis(7.0, 1440, 14.9);

// TODO: Declare a servo object
// Due to library constraints, servo MUST be connected to pin 5
Servo32U4 servo;

// TODO: Define the servo positions for each of the platforms
#define SERVO_A 1000
#define SERVO_B 1300
#define SERVO_C 1600

// TODO: Declare rangefinder object
Rangefinder rangefinder(11, 4);

// Setup the IR receiver/decoder object
const uint8_t IR_DETECTOR_PIN = 1;
IRDecoder decoder(IR_DETECTOR_PIN);

// Helper function for debugging
#define LED_PIN 13
void setLED(bool value)
{
  Serial.println("setLED()");
  digitalWrite(LED_PIN, value);
}

// TODO: Add bagging state
enum ROBOT_STATE {ROBOT_IDLE, ROBOT_DRIVE_FOR, ROBOT_LINE_FOLLOWING, ROBOT_TURNING, ROBOT_BAGGING, ROBOT_DROPPING};
ROBOT_STATE robotState = ROBOT_IDLE;

//Action handleIntersection(Delivery& del);
void handleIntersection(void);

// A helper function to stop the motors
void idle(void)
{
  Serial.println("idle()");
  setLED(LOW);

  //stop motors 
  chassis.idle();

  //set state to idle
  robotState = ROBOT_IDLE;
}

// A helper command to drive a set distance
void drive(float dist, float speed)
{
  Serial.println("drive()");
  setLED(HIGH);
  chassis.driveFor(dist, speed);
  robotState = ROBOT_DRIVE_FOR;
}

// A helper function to turn a set angle
void turn(float ang, float speed)
{
  Serial.println("turn()");
  setLED(HIGH);
  chassis.turnFor(ang, speed);
  robotState = ROBOT_DRIVE_FOR;
}

void beginLineFollowing(void)
{
  setLED(HIGH);
  robotState = ROBOT_LINE_FOLLOWING;
}

// Used to check if the motions above are complete
void handleMotionComplete(void)
{
  idle();
}

// TODO: Add function to begin bagging
void beginBagging(void)
{
  robotState = ROBOT_BAGGING;
  speed = 5;
}

// TODO: Add function to detect if bag is close enough
bool checkBagEvent(uint16_t threshold)
{
  static uint16_t prevDistance = 99;

  bool retVal = false;

  uint16_t currDistance = rangefinder.getDistance();
  // Serial.println(String("dist: ") + String(currDistance));

  if(prevDistance > threshold && currDistance <= threshold) retVal = true;
  prevDistance = currDistance;

  return retVal;
}

// TODO: Add function to pick up bag
void pickupBag(void)
{
  Serial.print("Bagging...");

  servo.writeMicroseconds(1000);

  chassis.driveFor(8, 2);
  while(!chassis.checkMotionComplete()) {delay(1);} //blocking
  Serial.println("done!");
  servo.writeMicroseconds(2000);

  delivery.currDest = delivery.deliveryDest;

  turn(180, 45); //do a u-turn
}

// TODO: Add function to start dropping sequence
void driveToDrop(void)
{
  robotState = ROBOT_DROPPING;
  speed = 5;
}

// TODO: Add function to detect if platform is close enough
bool checkForPlatform(uint16_t threshold)
{
  static uint16_t prevDistance = 99;

  bool retVal = false;

  uint16_t currDistance = rangefinder.getDistance();
  // Serial.println(String("dist: ") + String(currDistance));

  if(prevDistance > threshold && currDistance <= threshold) retVal = true;
  prevDistance = currDistance;

  return retVal;
}

// TODO: Add function to drop off bag
void dropoffBag(Destination dest)
{
  Serial.print("Dropping...");

  /**
   * House A is at ground level. We do not check for a platform, because the dropoff
   * is indicated by a line.
   * */
  if(dest == HOUSE_A) 
  {
    // Adjust position
    Serial.println("Backing up.");
    chassis.driveFor(-10, 5);
    while(!chassis.checkMotionComplete()) {delay(1);} // blocking

    // Release the bag
    Serial.println("Dropping.");
    servo.writeMicroseconds(SERVO_A);
    delay(500); //blocking, but we need to make sure servo has moved
  }

  // For B and C, we need to drive forward a bit
  else if(dest == HOUSE_B) 
  {  
    Serial.println("Crawling forward.");
    chassis.driveFor(8, 2);
    while(!chassis.checkMotionComplete()) {delay(1);} //blocking
    
    // Release the bag
    Serial.println("Dropping.");
    servo.writeMicroseconds(SERVO_B);
    delay(500); //blocking, but we need to make sure servo has moved
  }

  else if(dest == HOUSE_C) 
  {
    Serial.println("Crawling forward.");
    chassis.driveFor(8, 2);
    while(!chassis.checkMotionComplete()) {delay(1);} //blocking

    // Release the bag
    Serial.println("Dropping.");
    servo.writeMicroseconds(SERVO_C);
    delay(500); //blocking, but we need to make sure servo has moved
  }

  delivery.currDest = START;

  // Back up a little so the hook clears the handle
  Serial.println("Backing up.");
  chassis.driveFor(-5, 5);
  while(!chassis.checkMotionComplete()) {delay(1);} // blocking    

  // Now command a U-turn (needed for all deliveries)
  Serial.println("U-turn");
  turn(180, 45); 
}

// Handles a key press on the IR remote
void handleKeyPress(int16_t keyPress)
{
  Serial.println("Key: " + String(keyPress));

  //ENTER_SAVE idles, regardless of state -- E-stop
  if(keyPress == ENTER_SAVE) idle(); 

  switch(robotState)
  {
    case ROBOT_IDLE:
      if(keyPress == UP_ARROW) drive(50, 10);
      else if(keyPress == DOWN_ARROW) drive(-50, 10);
      else if(keyPress == LEFT_ARROW) turn(90, 45);
      else if(keyPress == RIGHT_ARROW) turn(-90, 45);
      else if(keyPress == SETUP_BTN) beginLineFollowing();

      // TODO: Handle house 1
      if(keyPress == NUM_1)
      {
        delivery.deliveryDest = HOUSE_A;
        delivery.currDest = PICKUP;
        beginLineFollowing();
      }

      // TODO: Handle house 2
      if(keyPress == NUM_2)
      {
        delivery.deliveryDest = HOUSE_B;
        delivery.currDest = PICKUP;
        beginLineFollowing();
      }

      // For testing drop-offs
      if(keyPress == NUM_7)
      {
        delivery.deliveryDest = HOUSE_A;
        delivery.currDest = HOUSE_A;
        delivery.currLocation = ROAD_ABC; // We'll make a left turn then deliver
        beginLineFollowing();
      }

      if(keyPress == NUM_8)
      {
        delivery.deliveryDest = HOUSE_B;
        delivery.currDest = HOUSE_B;
        delivery.currLocation = ROAD_ABC; // We'll drive straight then deliver
        beginLineFollowing();
      }

      // TODO: Handle rewind button -> initiate bag pickup
      else if(keyPress == REWIND) beginBagging();
      break;
      
    case ROBOT_LINE_FOLLOWING:
      if(keyPress == VOLplus)  //VOL+ increases speed
      {
        speed += 5;
      }

      if(keyPress == VOLminus)  //VOL- decreases speed
      {
        speed -= 5;
      }
      break;
 
     default:
      break;
  }
}

void handleLineFollowing(float baseSpeed)
{
  const float Kp = 0.1;

  int16_t leftADC = analogRead(LEFT_LINE_SENSE);
  int16_t rightADC = analogRead(RIGHT_LINE_SENSE);
  
  int16_t error = leftADC - rightADC;
  float turnEffort = Kp * error;
  
  chassis.setTwist(baseSpeed, turnEffort);
}

// //here's a nice opportunity to introduce boolean logic
bool checkIntersectionEvent(int16_t darkThreshold)
{
  static bool prevIntersection = false;

  bool retVal = false;

  bool leftDetect = analogRead(LEFT_LINE_SENSE) > darkThreshold ? true : false;
  bool rightDetect = analogRead(RIGHT_LINE_SENSE) > darkThreshold ? true : false;

  bool intersection = leftDetect && rightDetect;
  if(intersection && !prevIntersection) retVal = true;
  prevIntersection = intersection;

  return retVal;
}

// void handleIntersection(void)
// {
//   Serial.println("Intersection!");

//   // Drive forward by dead reckoning to center the robot
//   chassis.driveFor(8, 5);

//   // We'll block for this one to reduce the complexity
//   while(!chassis.checkMotionComplete()) {}

//   Serial.println("Cleared");

//   Action nextAction = handleIntersection(delivery);

//   Serial.println(nextAction);
//   switch(nextAction)
//   {
//     case TASK_IDLE:
//       idle();
//       break;
//     case TURN_LEFT: // Left
//       turn(90, 45);
//       break;
//     case TURN_RIGHT: // Right
//       turn(-90, 45);
//       break;
//     case TURN_STRAIGHT: // Right
//       beginLineFollowing();
//       break;
//     case TURN_UTURN: // Right
//       turn(180, 45);
//       break;
//     case TASK_PICKUP:
//       beginBagging();
//       break;
//     case TASK_DROPOFF0:
//       dropoffBag(delivery.deliveryDest);
//       break;
//     case TASK_DROPOFF4:
//     case TASK_DROPOFF8:
//       driveToDrop();
//       break;
//     default:
//       break;
//   }
// }


/*
 * This is the standard setup function that is called when the board is rebooted
 * It is used to initialize anything that needs to be done once.
 */
void setup() 
{
  // This will initialize the Serial at a baud rate of 115200 for prints
  // Be sure to set your Serial Monitor appropriately
  Serial.begin(115200);

  // initialize the chassis (which also initializes the motors)
  chassis.init();
  idle();

  //these can be undone for the student to adjust
  chassis.setMotorPIDcoeffs(5, 0.5);

  // OPT: move servo to intermediate position
  servo.attach();
  servo.writeMicroseconds(1500); //move to neutral position to show it's alive

  // TODO: Initialize rangefinder
  rangefinder.init();

  // initialize the IR decoder
  decoder.init();

  Serial.println("/setup()");
}

/*
 * The main loop for the program. The loop function is repeatedly called
 * after setup() is complete.
 */
void loop()
{
  // Check for a key press on the remote
  int16_t keyPress = decoder.getKeyCode();
  if(keyPress >= 0) handleKeyPress(keyPress);

  // A basic state machine
  switch(robotState)
  {
    case ROBOT_DRIVE_FOR: 
       if(chassis.checkMotionComplete()) handleMotionComplete(); 
       break;

    case ROBOT_TURNING: 
       if(chassis.checkMotionComplete()) beginLineFollowing(); 
       break;

    case ROBOT_LINE_FOLLOWING:
      handleLineFollowing(speed); //argument is base speed
      if(checkIntersectionEvent(darkThreshold)) handleIntersection();
      break;

    // TODO: Handle bagging state
    case ROBOT_BAGGING:
      handleLineFollowing(speed); //crawl towards bag
      if(checkBagEvent(8)) {pickupBag();}
      break;

    // TODO: Handle bagging state
    case ROBOT_DROPPING:
      handleLineFollowing(speed); //crawl towards bag
      if(checkForPlatform(8)) {dropoffBag(delivery.deliveryDest);}
      break;

    default:
      break;
  }
}




/**
 * handleIntersection() is called when the robot reaches an intersection. 
 * It returns a value for which way the robot should go.
 * */
void handleIntersection(void)
{
    Serial.println("Intersection!");

    // Drive forward by dead reckoning to center the robot
    chassis.driveFor(8, 5);

    // We'll block for this one to reduce the complexity
    while(!chassis.checkMotionComplete()) {}

    Serial.println("Cleared");

    Serial.print("intersection: ");
    Serial.print(delivery.currLocation);
    Serial.print('\t');
    Serial.print(delivery.currDest);
    Serial.print('\t');
 
    switch(delivery.currLocation)
    {
        case ROAD_MAIN:
            if(delivery.currDest == PICKUP)
            {
                delivery.currLocation = ROAD_PICKUP;
                beginBagging();
            }

            else
            {
                delivery.currLocation = ROAD_START;
                beginLineFollowing();
            }

            break;

        case ROAD_PICKUP:
            // regardless of destination
            {
                delivery.currLocation = ROAD_MAIN;
                beginLineFollowing();
            }

            break;

        case ROAD_START:
            if(delivery.currDest == HOUSE_A || delivery.currDest == HOUSE_B)
            {
                delivery.currLocation = ROAD_ABC;
                beginLineFollowing();
            }
            else if(delivery.currDest == START)
            {
                delivery.currLocation = ROAD_MAIN;
                delivery.currDest = NONE;
                delivery.deliveryDest = NONE;
                idle();
            }

            break;

        case ROAD_ABC:
            if(delivery.currDest == HOUSE_A)
            {
                delivery.currLocation = ROAD_A;
                turn(90, 45);
            }
            else if(delivery.currDest == HOUSE_B)
            {
                delivery.currLocation = ROAD_B;
                driveToDrop();
            }
            else if(delivery.currDest == START)
            {
                delivery.currLocation = ROAD_START;
                beginLineFollowing();
            }

            break;

        case ROAD_A:
            if(delivery.currDest == HOUSE_A)
            {
                dropoffBag(delivery.deliveryDest);
            }
            else if(delivery.currDest == START)
            {
                delivery.currLocation = ROAD_ABC;
                turn(-90, 45);
            }
            break;

        case ROAD_B:
            if(delivery.currDest == START)
            {
                delivery.currLocation = ROAD_ABC;
                beginLineFollowing();
            }
            break;

        default: 
          Serial.println("Unhandled case!");
          idle();
    }
}

