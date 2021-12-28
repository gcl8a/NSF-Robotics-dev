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
                currLocation = ROAD_MAIN;
                return TURN_STRAIGHT;

            break;

        case ROAD_START:
                currLocation = ROAD_ABC;
                return TURN_STRAIGHT;

            break;

        case ROAD_ABC:
                currLocation = ROAD_A;
                return TURN_LEFT;

            break;

        case ROAD_A:
                currLocation = ROAD_A_DROP;
                return TURN_STRAIGHT;

            break;

        case ROAD_A_DROP:
                currLocation = ROAD_A_DROP;
                return TASK_IDLE;

            break;

        default: return TASK_IDLE;
    }

    return TASK_IDLE;
}
