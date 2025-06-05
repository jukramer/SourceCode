#pragma once

#include "common.h"

enum class Motor_Choice {
    LEFT = 0,
    RIGHT = 1
};

class Motor
{
public:
    float PWM = 0;

    // These two are updated every frame
    float RPM = 0;
    float DELTA_POS = 0;

    // This is changed when setPWM is called
    int DIR = FORWARD;

    const volatile uint *totalTicks;
    uint prevTicksRPM;
    uint prevTicksPOS;

    int pinForward;
    int pinBackward;
    int pinPWM;
    int pinENC;

    uint64_t tPrev;
    float prevDist = 0.0f;

    Motor_Choice choice;

    Motor(Motor_Choice choice);

    void setPWM(float PWM);
    void update();
    void updateSmooth();
};

enum class TOF_Direction
{
    FRONT = 0,
    LEFT = 1,
    RIGHT = 2,
    FRONT_LEFT_45 = 3,
    FRONT_RIGHT_45 = 4,
    COUNT
};

#define NUM_TOF_SENSORS ((int) TOF_Direction::COUNT)

void global_init();
void global_read_tofs();
void global_read_imu();

// This is only for simulation...
void setWall_UI(int x, int y, Direction direction);

extern "C" {
uint64_t time_us_64();
bool stdio_usb_connected();
void sleep_ms(uint32_t ms);
}

inline Cell MAZE_MATRIX[MAZE_SIZE][MAZE_SIZE] = {}; // Init to empty with = {};
inline volatile byte FLOOD_MATRIX[MAZE_SIZE][MAZE_SIZE] = {};
inline volatile byte FLOOD_GEN_MATRIX[MAZE_SIZE][MAZE_SIZE] = {};
inline volatile byte MOVE_MATRIX[MAZE_SIZE][MAZE_SIZE];
inline volatile byte CURRENT_FLOOD_GEN = 0;

//
// Sensor readings, long range one is updated around 40 Hz, short range ones around 20 Hz
//
inline int MM[5] = {0};
inline bool MM_VALID[5] = {false, false, false, false, false};

inline float AX = 0.0f;
inline float AY = 0.0f;
inline float GYRO_Z = 0.0f;

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

    setWall_UI(location.x, location.y, dir);
}
