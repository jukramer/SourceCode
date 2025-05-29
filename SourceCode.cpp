#include <stdio.h>
#include <iostream>
#include <list>
#include <algorithm>
#include <vector>
#include <queue>
#include <stack>
#include <chrono>
#include <thread>
#include <iomanip>
#include <string>

#include "functions.h"

#include "API.h"

// #include "pico/stdlib.h"
// #include "pico/cyw43_arch.h"
// #include "pins.hpp"

/////////////// DEFINITIONS /////////////////////
// Global States
#define INIT 0
#define IDLE 1
#define MAP_FLOODFILL 2

#define MAP_ASTAR 3
#define MAP_CUSTOM 4

#define MAP_FLOODFILL_BACK 5

// Mapping States

///////////////// CLASSES ///////////////////////

#define MAZE_SIZE 16
struct Point
{
    int x = 0;
    int y = 0;
};

Point operator+(Point p1, Point p2)
{
    return {p1.x + p2.x, p1.y + p2.y};
}

Point &operator+=(Point &p1, Point p2)
{
    p1.x += p2.x;
    p1.y += p2.y;
    return p1;
}

bool operator==(Point p1, Point p2)
{
    return p1.x == p2.x && p1.y == p2.y;
}
bool operator!=(Point p1, Point p2) { return !(p1 == p2); }

struct Cell
{
    union
    {
        struct
        {
            uint8_t topWall : 1;    // 1 bit for the top wall
            uint8_t rightWall : 1;  // 1 bit for the right wall
            uint8_t bottomWall : 1; // 1 bit for the bottom wall
            uint8_t leftWall : 1;   // 1 bit for the left wall
            uint8_t visited : 1;    // 1 bit for visited status
        };
        uint8_t walls; // 8 bits, for easy manipulation of all walls together
    };
};

#define force_inline __attribute__((always_inline)) inline

// Ring buffer queue for flood fill
struct Queue
{
    Point buffer[256];
    uint8_t head = 0, tail = 0;

    force_inline void reset() { head = tail = 0; }
    force_inline bool empty() const { return head == tail; }
    force_inline void push(Point p) { buffer[head++ & 255] = p; }
    force_inline Point pop() { return buffer[tail++ & 255]; }
};

enum Direction
{
    TOP = 0,
    RIGHT = 1,
    BOTTOM = 2,
    LEFT = 3
};

constexpr Direction OPPOSITE[4] = {BOTTOM, LEFT, TOP, RIGHT};
constexpr Direction ROTATE_LEFT[4] = {LEFT, TOP, RIGHT, BOTTOM};
constexpr Direction ROTATE_RIGHT[4] = {RIGHT, BOTTOM, LEFT, TOP};
constexpr char DIRECTION_CHAR[4] = {'n', 'e', 's', 'w'};

// Note: Must be in the same order as the bit fields in Cell!
constexpr Point DIRECTIONS[4] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}};

static volatile Cell mazeMatrix[MAZE_SIZE][MAZE_SIZE] = {}; // Init to empty with = {};

static volatile uint8_t floodMatrix[MAZE_SIZE][MAZE_SIZE] = {};
static volatile uint8_t floodGenMatrix[MAZE_SIZE][MAZE_SIZE] = {};
static volatile uint8_t currentFloodGen = 0;

force_inline bool isUnset(int x, int y)
{
    return floodGenMatrix[y][x] != currentFloodGen;
}

force_inline void setDist(int x, int y, uint8_t dist)
{
    floodMatrix[y][x] = dist;
    floodGenMatrix[y][x] = currentFloodGen;
}

static uint8_t moveMask[MAZE_SIZE][MAZE_SIZE];

void printMaze()
{
    const int N = 16;

    for (int y = 0; y < N; y++)
    {
        // Top walls
        for (int x = 0; x < N; x++)
        {
            std::cout << " ";
            if (mazeMatrix[y][x].topWall)
                std::cout << "---";
            else
                std::cout << "   ";
        }
        std::cout << " \n";

        // Side walls + spaces
        for (int x = 0; x < N; x++)
        {
            if (mazeMatrix[y][x].leftWall)
                std::cout << "|";
            else
                std::cout << " ";

            std::cout << std::setw(3) << (int) floodMatrix[y][x];
        }

        // Right wall of the last cell
        if (mazeMatrix[y][N - 1].rightWall)
            std::cout << "|\n";
        else
            std::cout << " \n";
    }

    // Bottom walls of the last row
    for (int x = 0; x < N; x++)
    {
        std::cout << " ";
        if (mazeMatrix[N - 1][x].bottomWall)
            std::cout << "---";
        else
            std::cout << "   ";
    }
    std::cout << " \n";
}

void setWall(int x, int y, Direction dir)
{
    mazeMatrix[y][x].walls |= (1 << dir);
    moveMask[y][x] &= ~(1 << dir);

    int nx = x + DIRECTIONS[dir].x;
    int ny = y + DIRECTIONS[dir].y;

    if (nx >= 0 && nx < 16 && ny >= 0 && ny < 16)
    {
        mazeMatrix[ny][nx].walls |= (1 << OPPOSITE[dir]);
        moveMask[ny][nx] &= ~(1 << OPPOSITE[dir]);
    }

    API::setWall(x, 15 - y, DIRECTION_CHAR[dir]);
}

// Global States
#define INIT 0
#define IDLE 1
#define MAP_FLOODFILL 2

#define MAP_ASTAR 3
#define MAP_CUSTOM 4

#define MAP_FLOODFILL_BACK 5

class Mouse
{
public:
    Point cellPos = {0, MAZE_SIZE - 1};
    Direction orientation = Direction::TOP;

    int state = IDLE;
    int solvingType = 0; // 0 = floodfill, 1 = a*, 2 = custom  -  replace with states?

    int phase = 0; // 0: mapping phase   1: pathfinding phase   2: solving phase

    Mouse()
    {
        mazeMatrix[cellPos.y][cellPos.x].visited = true; // Mark starting cell as visited
    }

    void turnLeft()
    {
        API::turnLeft();
        orientation = ROTATE_LEFT[orientation];
    }

    void turnRight()
    {
        API::turnRight();
        orientation = ROTATE_RIGHT[orientation];
    }

    void move(Direction targetDir)
    {
        while (targetDir != orientation)
        {
            turnLeft();
        }

        API::moveForward();
        cellPos += DIRECTIONS[targetDir];
        mazeMatrix[cellPos.y][cellPos.x].visited = true;
    }
};

Queue floodQueue;


force_inline void tryPropagate(int dir, int x, int y, int dist, uint8_t mask) {
    if (!(mask & (1 << dir))) return;

    int nx = x + DIRECTIONS[dir].x;
    int ny = y + DIRECTIONS[dir].y;
    uint8_t ndist = floodMatrix[ny][nx];
    if (isUnset(nx, ny) || ndist > dist + 1) {
        setDist(nx, ny, dist + 1);
        floodQueue.push({nx, ny});
    }
}

force_inline void initBackFill(int dir, int x, int y, uint8_t mask) {
    if (!(mask & (1 << dir))) return;

    int nx = x + DIRECTIONS[dir].x;
    int ny = y + DIRECTIONS[dir].y;

    auto &neighbor = mazeMatrix[ny][nx];
    if (!neighbor.visited)
    {
        setDist(nx, ny, 10); // Set initial distance for unknown neighbors
        floodQueue.push({nx, ny});
    }
}


void floodFill(Mouse &mouse)
{
    auto begin = std::chrono::high_resolution_clock::now();

    currentFloodGen++;
    floodQueue.reset();

    if (mouse.state == MAP_FLOODFILL)
    {
        setDist(7, 7, 0); // Cost to start is 0
        setDist(7, 8, 0);
        setDist(8, 7, 0);
        setDist(8, 8, 0);

        // Add goal cells to queue
        floodQueue.push({7, 7});
        floodQueue.push({7, 8});
        floodQueue.push({8, 7});
        floodQueue.push({8, 8});
    }
    else if (mouse.state == MAP_FLOODFILL_BACK)
    {
        setDist(0, MAZE_SIZE - 1, 0); // Start from bottom left corner
        floodQueue.push({0, MAZE_SIZE - 1});

        for (int x = 0; x < MAZE_SIZE; x++)
        {
            for (int y = 0; y < MAZE_SIZE; y++)
            {
                auto &cell = mazeMatrix[y][x];
                if (!cell.visited)
                    continue; // Only consider known cells

                uint8_t mask = moveMask[y][x];
                initBackFill(0, x, y, mask);
                initBackFill(1, x, y, mask);
                initBackFill(2, x, y, mask);
                initBackFill(3, x, y, mask);
            }
        }
    }

    int c = 0; // Cell counter
    while (!floodQueue.empty())
    {
        Point current = floodQueue.pop();
        c++;

        int x = current.x, y = current.y;
        int dist = floodMatrix[y][x];

        uint8_t mask = moveMask[y][x];
        tryPropagate(0, x, y, dist, mask);
        tryPropagate(1, x, y, dist, mask);
        tryPropagate(2, x, y, dist, mask);
        tryPropagate(3, x, y, dist, mask);
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
    std::cout << "Flood fill completed in " << duration.count() / 1e6 << " ms, processed " << c << " cells.\n";

    printMaze();
}

////////////////// FUNCTIONS ////////////////////

void scanWalls(Mouse &mouse)
{
    if (API::wallFront())
    {
        setWall(mouse.cellPos.x, mouse.cellPos.y, mouse.orientation);
    }
    if (API::wallLeft())
    {
        setWall(mouse.cellPos.x, mouse.cellPos.y, ROTATE_LEFT[mouse.orientation]);
    }
    if (API::wallRight())
    {
        setWall(mouse.cellPos.x, mouse.cellPos.y, ROTATE_RIGHT[mouse.orientation]);
    }
}

void aStar()
{
    ;
}

int main()
{
    std::ios::sync_with_stdio(false);
    std::cin.tie(nullptr);

    for (int y = 0; y < MAZE_SIZE; y++)
    {
        for (int x = 0; x < MAZE_SIZE; x++)
        {
            uint8_t mask = 0b1111;

            if (y == 0)               mask &= ~(1 << TOP);
            if (x == MAZE_SIZE - 1)   mask &= ~(1 << RIGHT);
            if (y == MAZE_SIZE - 1)   mask &= ~(1 << BOTTOM);
            if (x == 0)               mask &= ~(1 << LEFT);

            moveMask[y][x] = mask;
        }
    }

    // Initialize mouse
    Mouse mouse;

    // Global State Machine
    while (true)
    {
        if (mouse.state == INIT)
        {
            // stdio_init_all();
            // for (int i = 0; i < 27; i++){
            //     gpio_init(allPins[i]);
            //     gpio_set_dir(allPins[i], GPIO_OUT);
            // }
            mouse.state = IDLE;
        }
        else if (mouse.state == IDLE)
        {
            mouse.state = MAP_FLOODFILL;
        }
        else if (mouse.state == MAP_FLOODFILL || mouse.state == MAP_FLOODFILL_BACK)
        {
            scanWalls(mouse);
            floodFill(mouse);

            Direction minFloodFillDirection;
            int minFloodFill = 255;
            
            int x = mouse.cellPos.x;
            int y = mouse.cellPos.y;

            uint8_t mask = moveMask[y][x];
            for (int i = 0; i < 4; ++i) {
                if (!(mask & (1 << i))) continue;

                int nx = x + DIRECTIONS[i].x;
                int ny = y + DIRECTIONS[i].y;

                if (floodMatrix[ny][nx] <= minFloodFill)
                {
                    minFloodFill = floodMatrix[ny][nx];
                    minFloodFillDirection = (Direction)i;
                }
            }

            // std::cout << "Flood fill direction: " << minFloodFillDirection << std::endl;
            // std::cout << "Flood fill value: " << minFloodFill << std::endl;

            mouse.move(minFloodFillDirection);

            if (floodMatrix[mouse.cellPos.y][mouse.cellPos.x] == 0)
            {
                if (mouse.state == MAP_FLOODFILL)
                {
                    std::cout << "Reached goal! Backing..." << std::endl;
                    mouse.state = MAP_FLOODFILL_BACK;
                } else if (mouse.state == MAP_FLOODFILL_BACK)
                {
                    if (mouse.cellPos.x == 0 && mouse.cellPos.y == MAZE_SIZE - 1)
                    {
                        std::cout << "Reached start! Stopping..." << std::endl;
                        break;
                    }
                }
            }
        }
    }

    // main loop
    // gpio_put(ledPin, 1);

    // floodfill
    // if (mouse.solvingType == 0) {

    //     int x0 = mouse.cellPos[0];
    //     int y0 = mouse.cellPos[1];
    // mazeMatrix[x0][y0] = cellConfig(mouse);

    // mazeMatrix[13][4] = {0,1,1,0,1};
    // for (int x = 5; x <= 10; ++x) {
    //     setWall(mouse, x, 6, TOP);
    // }

    // floodFill(mouse);
    // std::cout<<"hellohello"<<std::endl;
    // }
    // std::cin.get(); //uncomment if running exe

    // std::vector<std::string> path = stateMachine("FRFFLFRLS");
    // for (std::string item : path) {
    //     std::cout<<item<<" ";
    // }
    // std::cout<<std::endl;

    //     std::vector<std::string> path = stateMachineGoated("FRFRLFLLRFRLS");
    //     for (std::string item : path) {
    //         std::cout<<item<<" ";
    // }
    //     std::cout<<std::endl;
}