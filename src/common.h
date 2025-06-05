#pragma once

#include <stdint.h>
#include <vector>
#include <string>

typedef unsigned int uint;
typedef void (*gpio_irq_callback_t)(uint gpio, uint32_t event_mask);

using byte = uint8_t;

#if defined _MSC_VER
#define force_inline inline
#else
#define force_inline __attribute__((always_inline)) inline
#endif

#define STATE_IDLE 0
#define STATE_MAP_EXPLORE 2
#define STATE_MAP_EXPLORE_BACK 3
#define STATE_FAST_RUN 5


inline volatile byte STATE = STATE_IDLE; // Global state of the mouse

enum Direction
{
    NORTH,
    EAST,
    SOUTH,
    WEST,
    HEADING_COUNT,
    BLOCKED = 99
};

struct Point {
    float x;
    float y;
};

struct Command {
    std::string action; // "FWD", "TRN", "STOP"
    float value;        // cells for FWD, radians for TRN (positive for left)
};

enum MovementType { FWD, TURN_L, TURN_R, STOP_CMD, IDLE };

constexpr Direction OPPOSITE[4] = {SOUTH, WEST, NORTH, EAST};
constexpr Direction LEFT_FROM[4] = {WEST, NORTH, EAST, SOUTH};
constexpr Direction ROTATE_RIGHT[4] = {EAST, SOUTH, WEST, NORTH};

#define MAZE_SIZE 16
#define MAZE_CELL_COUNT (MAZE_SIZE * MAZE_SIZE)
#define MAX_COST (MAZE_CELL_COUNT - 1)

class Location
{
public:
    int x;
    int y;

    constexpr Location() : x(0), y(0) {};
    constexpr Location(int ix, int iy) : x(ix), y(iy) {};

    force_inline bool is_in_maze()
    {
        return x >= 0 && y >= 0 && x < MAZE_SIZE && y < MAZE_SIZE;
    }

    force_inline bool operator==(const Location &obj) const
    {
        return x == obj.x && y == obj.y;
    }

    force_inline bool operator!=(const Location &obj) const
    {
        return x != obj.x || y != obj.y;
    }

    force_inline Location operator+(Location p2)
    {
        return {x + p2.x, y + p2.y};
    }

    force_inline Location &operator+=(Location p2)
    {
        x += p2.x;
        y += p2.y;
        return *this;
    }
};

std::string findFastestPath();

// Ring buffer queue for flood fill
template <typename T>
struct Queue
{
    T buffer[256];
    byte head = 0, tail = 0;

    force_inline void reset() { head = tail = 0; }
    force_inline bool empty() const { return head == tail; }
    force_inline void push(T p) { buffer[head++ & 255] = p; }
    force_inline T pop() { return buffer[tail++ & 255]; }
};

enum WallState
{
    EXIT = 0,    // a wall that has been seen and confirmed absent
    WALL = 1,    // a wall that has been seen and confirmed present
};

struct StatePrediction
{
    double x;
    double y;
    double theta;
};

struct Cell
{
    union
    {
        struct
        {
            WallState north : 1;  // 1 bit for the north wall
            WallState east : 1;   // 1 bit for the east wall
            WallState south : 1;  // 1 bit for the south wall
            WallState west : 1;   // 1 bit for the west wall
            bool visited : 1;     // 1 bit for visited status
        };
        byte walls;
    };
};

// Note: Must be in the same order as the bit fields in Cell!
constexpr Location OFFSET_LOCATIONS[4] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}};

#define FORWARD 1
#define BACKWARD -1

#define WHEEL_RADIUS_MM 22
#define WHEEL_BASE_MM 80