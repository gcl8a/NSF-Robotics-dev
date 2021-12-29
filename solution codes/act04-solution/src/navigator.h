#pragma once

enum Road {ROAD_MAIN, ROAD_PICKUP, ROAD_START, ROAD_ABC, ROAD_A, ROAD_A_DROP, ROAD_B};
enum Destination {NONE, START, PICKUP, HOUSE_A, HOUSE_B, HOUSE_C};
enum Action {TURN_LEFT, TURN_STRAIGHT, TURN_RIGHT, TURN_UTURN, TASK_IDLE, 
                TASK_PICKUP, TASK_DROPOFF0, TASK_DROPOFF4, TASK_DROPOFF8, TASK_DRIVE};

class Navigator
{
    Road currLocation = ROAD_MAIN;
    Destination currDest = NONE;
    Destination deliveryDest = NONE;

public:
    //Action nextAction(void);
    
    Action handleIntersection(void);
    Action handleMotionComplete(void);

    Destination handlePickup(void) {return currDest = deliveryDest;}
    Destination handleDropoff(void) {return currDest = START;}

    void setDestination(Destination dest) {currDest = PICKUP; deliveryDest = dest;}
    void setTest(Destination dest)
    {
        deliveryDest = dest;
        currDest = dest;
        if(dest == HOUSE_A) currLocation = ROAD_A;
    }
};

