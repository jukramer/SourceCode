#pragma once

#include <stdint.h>
#include <vector>
#include <string>

#define EPSILON 1e-6f

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
    TOP = 0,
    RIGHT = 1,
    BOTTOM = 2,
    LEFT = 3,
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

constexpr Direction OPPOSITE[4] = {BOTTOM, LEFT, TOP, RIGHT};
constexpr Direction ROTATE_LEFT[4] = {LEFT, TOP, RIGHT, BOTTOM};
constexpr Direction ROTATE_RIGHT[4] = {RIGHT, BOTTOM, LEFT, TOP};

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

std::string fastestPath(int floodMatrix);

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
            byte north : 1;  // 1 bit for the north wall
            byte east : 1;   // 1 bit for the east wall
            byte south : 1;  // 1 bit for the south wall
            byte west : 1;   // 1 bit for the west wall
            bool visited : 1;     // 1 bit for visited status
        };
        byte walls;
    };
};


struct Pose
{
    float x;     // mm
    float y;     // mm
    float theta; // radians, [-PI, PI), 0 along +X, PI/2 along +Y (CCW positive)
    float v;     // linear velocity, mm/s
    float w;     // angular velocity, rad/s (CCW positive)

    Pose(float _x = 0.f, float _y = 0.f, float _theta = 0.f, float _v = 0.f, float _w = 0.f)
        : x(_x), y(_y), theta(_theta), v(_v), w(_w) {}
};

struct Vec2f
{
    float x, y;

    constexpr Vec2f(float x_ = 0.0f, float y_ = 0.0f) : x(x_), y(y_) {}

    constexpr float norm() const
    {
        return sqrtf(x * x + y * y);
    }

    constexpr Vec2f normalized() const
    {
        float n = norm();
        if (n < EPSILON)
        {
            return Vec2f(0.0f, 0.0f);
        }
        return Vec2f(x / n, y / n);
    }

    Vec2f operator+(const Vec2f &other) const
    {
        return Vec2f(x + other.x, y + other.y);
    }

    Vec2f operator-(const Vec2f &other) const
    {
        return Vec2f(x - other.x, y - other.y);
    }

    Vec2f operator*(float scalar) const
    {
        return Vec2f(x * scalar, y * scalar);
    }

    friend Vec2f operator*(float scalar, const Vec2f &vec)
    {
        return vec * scalar;
    }
};

// Note: Must be in the same order as the bit fields in Cell!
constexpr Location OFFSET_LOCATIONS[4] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}};
constexpr Vec2f OFFSET_VECTORS[4] = {
    {0.0f, -1.0f}, // NORTH
    {1.0f, 0.0f},  // EAST
    {0.0f, 1.0f},  // SOUTH
    {-1.0f, 0.0f}  // WEST
};

#define FORWARD 1
#define BACKWARD -1

#define WHEEL_RADIUS_MM 22
#define WHEEL_BASE_MM 90
#define CELL_SIZE_MM 180.0f

#define SENSOR_NOISE_STDDEV 0.1f
#define SENSOR_NOISE_VAR (SENSOR_NOISE_STDDEV * SENSOR_NOISE_STDDEV)

#define DEFAULT_MAX_SENSOR_RANGE_MM 255.0f
#define FRONT_MAX_SENSOR_RANGE_MM (10.0f * 255.0f)

#define LOCAL_TOF_BASE_OFFSET_X_MM 70.0f // Offset along robot's local X-axis
#define LOCAL_TOF_RADIAL_OFFSET_MM 25.0f // Additional radial offset for each sensor

#define WALL_THICKNESS_MM 12.0f // Wall thickness in mm

#define PI 3.14159265358979323846f // Define PI constant

