#pragma once

enum Road {ROAD_MAIN, ROAD_PICKUP, ROAD_START, ROAD_ABC, ROAD_A, ROAD_A_DROP, ROAD_B};
enum Destination {NONE, START, PICKUP, HOUSE_A, HOUSE_B, HOUSE_C};
enum Action {TURN_LEFT, TURN_STRAIGHT, TURN_RIGHT, TURN_UTURN, TASK_IDLE, 
                TASK_PICKUP, TASK_DROPOFF, TASK_DRIVE};

class Navigator
{
    Road currLocation = ROAD_MAIN;
    Destination currDest = NONE;
    Destination deliveryDest = NONE;

public:
    Action nextAction(void);
    void setDestination(Destination dest) {currDest = PICKUP; deliveryDest = dest;}
};

