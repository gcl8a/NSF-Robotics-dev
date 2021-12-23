#pragma once

enum Road {ROAD_MAIN, ROAD_A, ROAD_B};
enum Destination {START, PICKUP, HOUSE_A, HOUSE_B, HOUSE_C};
enum Turn {TURN_LEFT = -1, TURN_STRAIGHT, TURN_RIGHT, TURN_UTURN};

class Navigator
{
    Road currLocation = ROAD_MAIN;
    Destination currDest = START;

public:
    int calculateTurn(void);
};

