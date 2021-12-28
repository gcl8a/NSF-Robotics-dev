#pragma once

enum Road {ROAD_MAIN, ROAD_PICKUP, ROAD_ABC, ROAD_A, ROAD_B};
enum Destination {NONE, START, PICKUP, HOUSE_A, HOUSE_B, HOUSE_C};
enum Action {TURN_LEFT, TURN_STRAIGHT, TURN_RIGHT, TURN_UTURN, TASK_IDLE, 
                TASK_PICKUP, TASK_DROPOFF, TASK_DRIVE};
//enum Bag {PICKING_UP, OUT_FOR_DELIVERY, RETURN_TO_START};

class Navigator
{
    //bool hasBag = false;
    Road currLocation = ROAD_MAIN;
    Destination currDest = START;
    Destination deliveryDest = NONE;
    //Task currTask = TASK_IDLE;

public:
    Action nextAction(void);
    void setDestination(Destination dest) {currDest = PICKUP; deliveryDest = dest;}
};

