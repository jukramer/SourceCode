#pragma once

#include "common.h"

enum class Motor_Choice {
    LEFT = 0,
    RIGHT = 1
};

class Motor
{
public:
    int PWM = 0;
    int currentRPM = 0;

    const volatile uint *totalTicks;
    uint prevTicks;

    int pinForward;
    int pinBackward;
    int pinPWM;
    int pinENC;
    int dir = FORWARD;

    float prevRPM = 0.0;
    uint64_t tPrev;

    Motor_Choice choice;

    Motor(Motor_Choice choice);

    void setPWM(int PWM);
    float readRPM();
};

struct TOF_Reading
{
    float distance = 0.0f;
    bool valid = false;
};

enum class TOF_Direction
{
    FRONT = 0,
    LEFT = 1,
    RIGHT = 2,
    FRONT_LEFT_45 = 3,
    FRONT_RIGHT_45 = 4
};

namespace API
{
    void init();
    
    /*
     * Returns the reading of the chosen TOF sensor in millimeters.
     * If the reading is invalid (underflow or overflow), the
     * "valid" flag will be set to false.
     */
    TOF_Reading readTOF(TOF_Direction direction);

    /*
     * Updates the UI in the simulation to show 
     * that the mouse thinks there is a wall there.
     */
    void setWall_UI(int x, int y, Direction direction);
};

extern "C" {
uint64_t time_us_64();
bool stdio_usb_connected();
void sleep_ms(uint32_t ms);
}

inline volatile Cell MAZE_MATRIX[MAZE_SIZE][MAZE_SIZE] = {}; // Init to empty with = {};
inline volatile byte FLOOD_MATRIX[MAZE_SIZE][MAZE_SIZE] = {};
inline volatile byte FLOOD_GEN_MATRIX[MAZE_SIZE][MAZE_SIZE] = {};
inline volatile byte MOVE_MATRIX[MAZE_SIZE][MAZE_SIZE];
inline volatile byte CURRENT_FLOOD_GEN = 0;

inline void setWall(Location location, Direction dir, WallState state)
{
    MAZE_MATRIX[location.y][location.x].walls |= (state << dir);
    MOVE_MATRIX[location.y][location.x] &= ~(state << dir);

    Location n = location + OFFSET_LOCATIONS[dir];
    if (n.is_in_maze())
    {
        MAZE_MATRIX[n.y][n.x].walls |= (1 << OPPOSITE[dir]);
        MOVE_MATRIX[n.y][n.x] &= ~(1 << OPPOSITE[dir]);
    }

    API::setWall_UI(location.x, location.y, dir);
}
