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
#include "range.h"

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
    return {p1.x + p2.x, p2.x + p2.y};
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
        };
        uint8_t walls; // 8 bits, for easy manipulation of all walls together
    };
    bool visited = false;
};

enum Direction
{
    TOP = 0,
    RIGHT = 1,
    BOTTOM = 2,
    LEFT = 3
};

char directionToChar(Direction dir)
{
    switch (dir)
    {
    case TOP:
        return 'n';
    case RIGHT:
        return 'e';
    case BOTTOM:
        return 's';
    case LEFT:
        return 'w';
    default:
        return '?';
    }
}

Direction opposite(Direction dir)
{
    return (Direction)(((int)dir + 2) % 4);
}

Direction rotate_direction_left(Direction dir)
{
    if (dir == TOP)
    {
        return LEFT;
    }
    else if (dir == LEFT)
    {
        return BOTTOM;
    }
    else if (dir == BOTTOM)
    {
        return RIGHT;
    }
    else
    {
        return TOP;
    }
}

Direction rotate_direction_right(Direction dir)
{
    if (dir == TOP)
    {
        return RIGHT;
    }
    else if (dir == RIGHT)
    {
        return BOTTOM;
    }
    else if (dir == BOTTOM)
    {
        return LEFT;
    }
    else
    {
        return TOP;
    }
}

// Note: Must be in the same order as the bit fields in Cell!
const Point DIRECTIONS[4] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}};

class Node
{
public:
    double g, h;            // A* values
    std::vector<float> pos; // position vector: <x,y>

    Node(std::vector<float> position, double g, double h)
    {
        std::vector<float> pos = position;
        g = g;
        h = h;
    }
};

class Mouse
{
public:
    Point cellPos = {0, MAZE_SIZE - 1};
    Direction orientation = Direction::TOP;

    int state = IDLE;
    int solvingType = 0; // 0 = floodfill, 1 = a*, 2 = custom  -  replace with states?

    std::vector<Point> adjacentCells;
    Point targetCell = {0, 0};

    bool posChanged = false; // true if the cell position of the mouse has changed
    std::stack<Point> cellPath;

    int phase = 0; // 0: mapping phase   1: pathfinding phase   2: solving phase

    // Maze map in matrix
    Cell mazeMatrix[MAZE_SIZE][MAZE_SIZE] = {}; // Init to empty with = {};

    // Flood matrix
    int floodMatrix[MAZE_SIZE][MAZE_SIZE] = {};

    Mouse()
    {
        mazeMatrix[cellPos.y][cellPos.x].visited = true; // Mark starting cell as visited
    }

    void turnLeft()
    {
        API::turnLeft();
        orientation = rotate_direction_left(orientation);
    }

    void turnRight()
    {
        API::turnRight();
        orientation = rotate_direction_right(orientation);
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

////////////////// FUNCTIONS ////////////////////

std::vector<int> cellConfig(Mouse mouse)
{
    // assign dl, dr, and df via tof sensors later!

    double dl = 0, dr = 0, df = 0; // distance values for front, left, and right
    double tl = 0, tr = 0, tf = 0; // threshold values for front, left, and right
    std::vector<double> dVec = {dl, df, dr};
    std::vector<double> tVec = {tl, tf, tr};
    std::vector<int> cellVec = {0, 0, 0, 0}; // left, top, right, bottom, checked (y/n). 1=wall, 0=no wall

    // places wall around cells if detected
    for (int i = 0; i <= dVec.size(); i++)
    {
        if (dVec[i] < tVec[i])
        {
            cellVec[i] = 1;
        }
    }

    // shifts cellVec depending on mouse orientation
    std::rotate(cellVec.begin(), cellVec.end() - mouse.orientation, cellVec.end());

    cellVec.insert(cellVec.end(), 1); // adds a 1 to end of cell to signify it has been checked

    return cellVec;
}

bool contains(std::stack<Point> stack, const Point &target)
{
    // Convert stack to temporary vector while searching
    while (!stack.empty())
    {
        if (stack.top() == target)
        {
            return true;
        }
        stack.pop();
    }
    return false;
}

void turnPossible(Mouse mouse)
{
    ;
    /*if (mouse.mazeMatrix[mouse.cellX][mouse.cellY][0] == 0)
    {
        mouse.turnLeft = true;
    }
    else if (mouse.mazeMatrix[mouse.cellX][mouse.cellY][0] == 1)
    {
        mouse.turnLeft = false;
    }

    if (mouse.mazeMatrix[mouse.cellX][mouse.cellY][0] == 0)
    {
        mouse.turnLeft = true;
    }
    else if (mouse.mazeMatrix[mouse.cellX][mouse.cellY][0] == 1)
    {
        mouse.turnLeft = false;
    }

    if (mouse.mazeMatrix[mouse.cellX][mouse.cellY][0] == 0)
    {
        mouse.turnLeft = true;
    }
    else if (mouse.mazeMatrix[mouse.cellX][mouse.cellY][0] == 1)
    {
        mouse.turnLeft = false;
    }*/
}

void mapping(Mouse mouse) // handle overall movement of the mouse
{
    // movement handling here
    /*
        if (mouse.posChanged)
        {
            mouse.posChanged = false;
            if (mouse.cellPath.top() != mouse.cellPos)
            {
                mouse.cellPath.push({mouse.cellX, mouse.cellY});
            }

            // mouse.mazeMatrix[mouse.cellX-1][mouse.cellY-1] = cellConfig(mouse); // assign cell vector to each matrix cell

            // Find adjacent cells to mouse
            std::vector<Point> adjacentCells;
            adjacentCells[0] = {mouse.cellX - 1, mouse.cellY}; // cell to left of mouse
            adjacentCells[1] = {mouse.cellX, mouse.cellY - 1}; // cell in front of mouse
            adjacentCells[2] = {mouse.cellX + 1, mouse.cellY}; // cell to right of mouse

            std::rotate(adjacentCells.begin(), adjacentCells.end() - mouse.orientation, adjacentCells.end()); // account for mouse orientation

            // set mouse adjacentCells
            mouse.adjacentCells = adjacentCells;

            // Find if adjacent cell has been visited or not
            if (mouse.possMovements[0] && contains(mouse.cellPath, adjacentCells[0]))
            {
                mouse.targetCell = adjacentCells[0];
            }
            else if (mouse.possMovements[1] && contains(mouse.cellPath, adjacentCells[1]))
            {
                mouse.targetCell = adjacentCells[1];
            }
            else if (mouse.possMovements[2] && contains(mouse.cellPath, adjacentCells[2]))
            {
                mouse.targetCell = adjacentCells[2];
            }
            else
            {
                mouse.targetCell = mouse.cellPath.top();
                mouse.cellPath.pop();
            }
        }

        // Check if all cells have been mapped
        int uncheckedCells = 0;

        while (int checkedCells = 0)
        {
            for (int i = 0; i < MAZE_SIZE; i++)
            {
                for (int j = 0; i < MAZE_SIZE; j++)
                {
                    //if (mouse.mazeMatrix[i][j][4] == 0) //TODO XXXXXXXXXX
                    {
                        uncheckedCells += 1;
                    }
                }
            }
        }

        if (uncheckedCells == 0)
        {
            mouse.phase = 1;
        }*/
}

std::vector<Point> getNeighbors(const Point &cell)
{
    std::vector<Point> neighbors;

    // Order: Up, Right, Down, Left
    int dx[4] = {0, 1, 0, -1};
    int dy[4] = {-1, 0, 1, 0};

    for (int i = 0; i < 4; ++i)
    {
        int nx = cell.x + dx[i];
        int ny = cell.y + dy[i];
        if (nx >= 0 && nx < 16 && ny >= 0 && ny < 16)
        {
            neighbors.push_back({nx, ny});
        }
        else
        {
            // Push an invalid point or a placeholder
            neighbors.push_back({-1, -1});
        }
    }

    return neighbors;
}

void printMaze(Mouse &mouse)
{
    const int N = 16;

    auto &maze = mouse.mazeMatrix;

    for (int y = 0; y < N; y++)
    {
        // Top walls
        for (int x = 0; x < N; x++)
        {
            std::cout << " ";
            if (maze[y][x].topWall)
                std::cout << "---";
            else
                std::cout << "   ";
        }
        std::cout << " \n";

        // Side walls + spaces
        for (int x = 0; x < N; x++)
        {
            if (maze[y][x].leftWall)
                std::cout << "|";
            else
                std::cout << " ";

            std::cout << std::setw(3) << mouse.floodMatrix[y][x];
        }

        // Right wall of the last cell
        if (maze[y][N - 1].rightWall)
            std::cout << "|\n";
        else
            std::cout << " \n";
    }

    // Bottom walls of the last row
    for (int x = 0; x < N; x++)
    {
        std::cout << " ";
        if (maze[N - 1][x].bottomWall)
            std::cout << "---";
        else
            std::cout << "   ";
    }
    std::cout << " \n";
}

void printFloodfill(Mouse &mouse)
{
    // std::cout << "hello!!" << std::endl;
    for (int i : range(MAZE_SIZE))
    {
        for (int j : range(MAZE_SIZE))
        {
            std::cout << std::setw(4) << mouse.floodMatrix[i][j] << " ";
        }
        std::cout << std::endl;
    }
}

void setWall(Mouse &mouse, int x, int y, Direction dir)
{
    auto &maze = mouse.mazeMatrix;

    maze[y][x].walls |= (1 << dir);

    int dx = 0, dy = 0;
    switch (dir)
    {
    case TOP:
        dy = -1;
        break;
    case RIGHT:
        dx = +1;
        break;
    case BOTTOM:
        dy = +1;
        break;
    case LEFT:
        dx = -1;
        break;
    }

    int nx = x + dx;
    int ny = y + dy;

    if (nx >= 0 && nx < 16 && ny >= 0 && ny < 16)
    {
        maze[ny][nx].walls |= (1 << opposite(dir));
    }

    API::setWall(x, MAZE_SIZE - y - 1, directionToChar(dir));
}

void scanWalls(Mouse &mouse)
{
    if (API::wallFront())
    {
        setWall(mouse, mouse.cellPos.x, mouse.cellPos.y, mouse.orientation);
    }
    if (API::wallLeft())
    {
        setWall(mouse, mouse.cellPos.x, mouse.cellPos.y, rotate_direction_left(mouse.orientation));
    }
    if (API::wallRight())
    {
        setWall(mouse, mouse.cellPos.x, mouse.cellPos.y, rotate_direction_right(mouse.orientation));
    }
}

void floodFill(Mouse &mouse)
{
    // timer to check computation time
    auto start = std::chrono::high_resolution_clock::now();

    for (int i : range(MAZE_SIZE))
    {
        for (int j : range(MAZE_SIZE))
        {
            mouse.floodMatrix[i][j] = 255; // High initial value
        }
    }

    std::vector<Point> floodQueue;
    floodQueue.reserve(256);

    if (mouse.state == MAP_FLOODFILL)
    {
        mouse.floodMatrix[7][7] = 0; // Distance to center is 0
        mouse.floodMatrix[7][8] = 0;
        mouse.floodMatrix[8][7] = 0;
        mouse.floodMatrix[8][8] = 0;

        // Add goal cells to queue
        floodQueue.insert(floodQueue.begin(), {7, 7});
        floodQueue.insert(floodQueue.begin(), {7, 8});
        floodQueue.insert(floodQueue.begin(), {8, 7});
        floodQueue.insert(floodQueue.begin(), {8, 8});
    }
    else if (mouse.state == MAP_FLOODFILL_BACK)
    {
        mouse.floodMatrix[MAZE_SIZE - 1][0] = 0; // Cost to start is 0
        floodQueue.insert(floodQueue.begin(), {0, MAZE_SIZE - 1});

        for (int x : range(MAZE_SIZE))
        {
            for (int y : range(MAZE_SIZE))
            {
                Cell &cell = mouse.mazeMatrix[y][x];
                if (!cell.visited) continue; // Only consider known cells
                
                for (int i : range(len(DIRECTIONS))) {
                    int nx = x + DIRECTIONS[i].x;
                    int ny = y + DIRECTIONS[i].y;

                    if (nx < 0 || nx >= 16 || ny < 0 || ny >= 16)
                        continue;

                    bool wallHere = (mouse.mazeMatrix[y][x].walls & (1 << i));
                    bool wallThere = (mouse.mazeMatrix[ny][nx].walls & (1 << opposite((Direction)i)));

                    if (wallHere || wallThere)
                        continue;

                    Cell &neighbor = mouse.mazeMatrix[ny][nx];
                    if (!neighbor.visited)
                    {
                        mouse.floodMatrix[ny][nx] = 10;
                        floodQueue.insert(floodQueue.begin(), {nx, ny});
                    }
                }
            }
        }
    }

    int c = 0; // Cell counter
    while (!floodQueue.empty())
    {
        Point current = floodQueue.back();
        floodQueue.pop_back();
        c++;

        int x = current.x, y = current.y;
        int dist = mouse.floodMatrix[y][x];

        for (int i : range(len(DIRECTIONS)))
        {
            int nx = x + DIRECTIONS[i].x;
            int ny = y + DIRECTIONS[i].y;

            if (nx < 0 || nx >= 16 || ny < 0 || ny >= 16)
                continue;

            bool wallHere = (mouse.mazeMatrix[y][x].walls & (1 << i));
            bool wallThere = (mouse.mazeMatrix[ny][nx].walls & (1 << opposite((Direction)i)));

            if (wallHere || wallThere)
            {
                // std::cout << "Skipping wall dir " << i << " from (" << x << "," << y << ") to (" << nx << "," << ny << ")\n";
                continue;
            }

            if (mouse.floodMatrix[ny][nx] > dist + 1)
            {
                mouse.floodMatrix[ny][nx] = dist + 1;
                floodQueue.insert(floodQueue.begin(), {nx, ny});
            }
        }
    }

    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> duration = end - start;
    printMaze(mouse);
    printf("Checked %d cells in %.2f ms.\n", c, duration.count());
}

void aStar()
{
    ;
}

int main()
{
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

            for (int i : range(len(DIRECTIONS)))
            {
                int nx = mouse.cellPos.x + DIRECTIONS[i].x;
                int ny = mouse.cellPos.y + DIRECTIONS[i].y;

                if (nx < 0 || nx >= 16 || ny < 0 || ny >= 16)
                    continue;

                bool wallHere = (mouse.mazeMatrix[mouse.cellPos.y][mouse.cellPos.x].walls & (1 << i));
                bool wallThere = (mouse.mazeMatrix[ny][nx].walls & (1 << opposite((Direction)i)));

                if (wallHere)
                {
                    // std::cout << "Skipping wall here dir " << i << " from (" << mouse.cellPos.x << "," << mouse.cellPos.y << ") to (" << nx << "," << ny << ")\n";
                    continue;
                }

                if (wallThere)
                {
                    // std::cout << "Skipping wall there dir " << i << " from (" << mouse.cellPos.x << "," << mouse.cellPos.y << ") to (" << nx << "," << ny << ")\n";
                    continue;
                }

                if (mouse.floodMatrix[ny][nx] <= minFloodFill)
                {
                    minFloodFill = mouse.floodMatrix[ny][nx];
                    minFloodFillDirection = (Direction)i;
                }
            }

            // std::cout << "Flood fill direction: " << minFloodFillDirection << std::endl;
            // std::cout << "Flood fill value: " << minFloodFill << std::endl;

            mouse.move(minFloodFillDirection);

            if (mouse.floodMatrix[mouse.cellPos.y][mouse.cellPos.x] == 0)
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
    // mouse.mazeMatrix[x0][y0] = cellConfig(mouse);

    // mouse.mazeMatrix[13][4] = {0,1,1,0,1};
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