#include <Arduino.h>
#include "navigator.h"

/**
 * calculateTurn() is called when the robot reaches an intersection. 
 * It returns a value for which way the robot should go.
 * */
Action Navigator::nextAction(void)
{
    Serial.print(currLocation);
    Serial.print('\t');
    Serial.print(currDest);
    Serial.print('\t');
    Serial.print(deliveryDest);
    Serial.print('\t');
    switch(currLocation)
    {
        case ROAD_MAIN:
            if(currDest == PICKUP)
            {
                currLocation = ROAD_PICKUP;
                currDest = deliveryDest;
                return TASK_PICKUP;
            }

            else
            {
                currLocation = ROAD_START;
                return TURN_STRAIGHT;
            }

            break;

        case ROAD_PICKUP:
            if(currDest == PICKUP)
            {
                Serial.println("Whoa. Should be here. Line 37.");
                currLocation = ROAD_PICKUP;
                currDest = deliveryDest;
                return TURN_STRAIGHT;
            }
            else
            {
                currLocation = ROAD_MAIN;
                return TURN_STRAIGHT;
            }

            break;

        case ROAD_START:
            if(currDest == HOUSE_A)
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
            else if(currDest == START)
            {
                currLocation = ROAD_START;
                return TURN_STRAIGHT;
            }

            break;

        case ROAD_A:
            if(currDest == HOUSE_A)
            {
                currLocation = ROAD_A;
                currDest = START;
                return TASK_DROPOFF;
            }
            else if(currDest == START)
            {
                currLocation = ROAD_ABC;
                return TURN_RIGHT;
            }
            break;

        case ROAD_A_DROP:
                Serial.println("Whoa. Shoulnd't get here. Line 95.");
                currLocation = ROAD_A;
                currDest = START;
                return TURN_STRAIGHT;

            break;

        default: return TASK_IDLE;
    }

    return TASK_IDLE;
}
