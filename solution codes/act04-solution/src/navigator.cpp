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
                return -1;
            }

            else if(currDest == HOUSE_A) 
            {
                currLocation = ROAD_A;
                return -1;
            }

            else if(currDest == HOUSE_B) return 0;
            else if(currDest == HOUSE_C) return 1;
            break;

        case ROAD_A:
            if(currDest == START) 
            {
                currLocation = ROAD_MAIN;
                return 1;
            }

            break;

        default: return 0;
    }

    return 0;
}
