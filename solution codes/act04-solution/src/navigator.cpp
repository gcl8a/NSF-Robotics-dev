#include "navigator.h"

/**
 * calculateTurn() is called when the robot reaches an intersection. 
 * It returns a value for which way the robot should go.
 * */
int Navigator::calculateTurn(void)
{
    switch(currLocation)
    {
        case ROAD_MAIN:
            if(currDest == START) 
            {
                currLocation = ROAD_A;
                return TURN_LEFT;
            }

            else if(currDest == HOUSE_A) 
            {
                currLocation = ROAD_A;
                return TURN_LEFT;
            }

            break;

        case ROAD_A:
            if(currDest == START) 
            {
                currLocation = ROAD_MAIN;
                return TURN_RIGHT;
            }

            break;

        default: return TURN_STRAIGHT;
    }

    return TURN_STRAIGHT;
}
