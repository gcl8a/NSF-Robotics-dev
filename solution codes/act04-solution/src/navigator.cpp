#include <Arduino.h>
#include "navigator.h"

/**
 * handleIntersection() is called when the robot reaches an intersection. 
 * It returns a value for which way the robot should go.
 * */
/*Action Navigator::handleIntersection(void)
{
    Serial.print("intersection: ");
    Serial.print(currLocation);
    Serial.print('\t');
    Serial.print(currDest);
    Serial.print('\t');
 
    switch(currLocation)
    {
        case ROAD_MAIN:
            if(currDest == PICKUP)
            {
                currLocation = ROAD_PICKUP;
                return TASK_PICKUP;
            }

            else
            {
                currLocation = ROAD_START;
                return TURN_STRAIGHT;
            }

            break;

        case ROAD_PICKUP:
            // regardless of destination
            {
                currLocation = ROAD_MAIN;
                return TURN_STRAIGHT;
            }

            break;

        case ROAD_START:
            if(currDest == HOUSE_A || currDest == HOUSE_B)
            {
                currLocation = ROAD_ABC;
                return TURN_STRAIGHT;
            }
            else if(currDest == START)
            {
                currLocation = ROAD_MAIN;
                currDest = NONE;
                deliveryDest = NONE;
                return TASK_IDLE;
            }

            break;

        case ROAD_ABC:
            if(currDest == HOUSE_A)
            {
                currLocation = ROAD_A;
                return TURN_LEFT;
            }
            else if(currDest == HOUSE_B)
            {
                currLocation = ROAD_B;
                return TASK_DROPOFF4;
            }
            else if(currDest == START)
            {
                currLocation = ROAD_START;
                return TURN_STRAIGHT;
            }

            break;

        case ROAD_A:
            if(currDest == HOUSE_A)
            {
                return TASK_DROPOFF0;
            }
            else if(currDest == START)
            {
                currLocation = ROAD_ABC;
                return TURN_RIGHT;
            }
            break;

        case ROAD_B:
            if(currDest == START)
            {
                currLocation = ROAD_ABC;
                return TURN_STRAIGHT;
            }
            break;

        default: return TASK_IDLE;
    }

    return TASK_IDLE;
}


*/
// Action Navigator::handleMotionComplete(void)
// {
//     Serial.print("motion complete: ");
//     Serial.print(currLocation);
//     Serial.print('\t');
//     Serial.print(currDest);
//     Serial.print('\t');

//     switch(currLocation)
//     {
//         default: 
//             return TURN_STRAIGHT;
//     }

//     return TURN_STRAIGHT;
// }


// void handleTaskComplete(void)
// {
//   Action nextAction = navigator.handleMotionComplete();

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
//       dropoffBag(SERVO_0);
//       break;
//     case TASK_DROPOFF4:
//     case TASK_DROPOFF8:
//       dropoffBag(SERVO_4);
//       break;
//     default:
//       break;
//   }
// }