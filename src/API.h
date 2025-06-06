#pragma once

#include "common.h"

void floodFill();

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

    const volatile int64_t *totalTicks;
    int64_t prevTicks;

    int pinForward;
    int pinBackward;
    int pinPWM;
    int pinENC;

    uint64_t tPrev;
    float prevDist = 0.0f;

    Motor_Choice choice;

    Motor(Motor_Choice choice);

    void setPWM(float PWM);
    void setPWMRaw(int pwm);

    void update();
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

void floodFill();

void global_init();
void global_read_tofs();
void global_read_imu();

void setWall_UI(int x, int y, Direction direction);

extern "C" {
uint64_t time_us_64();
bool stdio_usb_connected();
void sleep_ms(uint32_t ms);
}

inline Cell MAZE_MATRIX[MAZE_SIZE][MAZE_SIZE] = {}; // Init to empty with = {};
inline byte FLOOD_MATRIX[MAZE_SIZE][MAZE_SIZE] = {};
inline byte FLOOD_GEN_MATRIX[MAZE_SIZE][MAZE_SIZE] = {};
inline byte MOVE_MATRIX[MAZE_SIZE][MAZE_SIZE];
inline byte CURRENT_FLOOD_GEN = 0;

//
// Sensor readings, long range one is updated around 40 Hz, short range ones around 20 Hz
//
inline int MM[5] = {0};
inline bool MM_VALID[5] = {false, false, false, false, false};

inline float AX = 0.0f;
inline float AY = 0.0f;
inline float GYRO_Z = 0.0f;

inline void setWall(int x, int y, Direction dir)
{
    MAZE_MATRIX[y][x].walls |= (1 << dir);
    MOVE_MATRIX[y][x] &= ~(1 << dir);

    Location n = Location{x, y} + OFFSET_LOCATIONS[dir];
    if (n.is_in_maze())
    {
        MAZE_MATRIX[n.y][n.x].walls |= (1 << OPPOSITE[dir]);
        MOVE_MATRIX[n.y][n.x] &= ~(1 << OPPOSITE[dir]);
    }

    setWall_UI(x, y, dir);
}
