#include <stdio.h>
#include <iostream>
#include <list>
#include <algorithm>
#include <vector>
#include <queue>
#include <stack>
#include "functions.h"
#include <chrono>
#include <thread>
#include <iomanip>
#include <string>

#include "range.h"

// #include "pico/stdlib.h"
// #include "pico/cyw43_arch.h"
// #include "pins.hpp"

/////////////// DEFINITIONS /////////////////////
// States
#define START 0
#define ORTHO 1
#define ORTHO_L 2
#define ORTHO_R 3
#define ORTHO_LL 4
#define ORTHO_RR 5
#define DIAG_LR 6
#define DIAG_RR 7
#define DIAG_RL 8
#define DIAG_LL 9
#define STOP 10

///////////////// CLASSES ///////////////////////

#define MAZE_SIZE 16

struct Point
{
    int x = 0;
    int y = 0;
};

bool operator==(Point p1, Point p2)
{
    return p1.x == p2.x && p1.y == p2.y;
}
bool operator!=(Point p1, Point p2) { return !(p1 == p2); }

struct Cell {
    union {
        struct {
            uint8_t topWall : 1;    // 1 bit for the top wall
            uint8_t rightWall : 1;  // 1 bit for the right wall
            uint8_t bottomWall : 1; // 1 bit for the bottom wall
            uint8_t leftWall : 1;   // 1 bit for the left wall
        };
        uint8_t walls; // 8 bits, for easy manipulation of all walls together
    };
};

enum Direction { TOP = 0, RIGHT = 1, BOTTOM = 2, LEFT = 3 };

int opposite(int dir) {
    return (dir + 2) % 4;
}

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
    int cellX = 0, cellY = 0; // position in maze grid
    Point cellPos = {cellX, cellY};
    int solvingType = 0; // 0 = floodfill, 1 = a*, 2 = custom

    std::vector<Point> adjacentCells;

    int targetCellX, targetCellY; // cell that the mouse wants to travel to next
    Point targetCell = {targetCellX, targetCellY};

    bool posChanged = false; // true if the cell position of the mouse has changed
    int orientation = 0;     // heading/orientation of mouse: 0=up, 1=right, 2=down, 3=left
    std::stack<Point> cellPath;
    std::vector<Point> visitedCells;

    int phase = 0; // 0: mapping phase   1: pathfinding phase   2: solving phase

    // Maze map in matrix
    Cell mazeMatrix[MAZE_SIZE][MAZE_SIZE] = {}; // Init to empty with = {};

    bool turnLeft = false, turnRight = false, goStraight = false; // variables that determine if specific movements are possible
    std::vector<bool> possMovements = {turnLeft, turnRight, goStraight};

    // Flood matrix
    int floodMatrix[MAZE_SIZE][MAZE_SIZE] = {};

    Mouse()
    {
    }
};

////////////////// FUNCTIONS ////////////////////

std::vector<int> cellConfig(Mouse mouse)
{
    // assign dl, dr, and df via tof sensors later!

    double dl =0, dr =0, df =0; // distance values for front, left, and right
    double tl =0, tr =0, tf =0; // threshold values for front, left, and right
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

    if (uncheckedCells = 0)
    {
        mouse.phase = 1;
    }
}

std::vector<Point> getNeighbors(const Point& cell)
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


void printMaze(Mouse &mouse) {
    const int N = 16;

    auto &maze = mouse.mazeMatrix;

    for (int y = 0; y < N; y++) {
        // Top walls
        for (int x = 0; x < N; x++) {
            std::cout << " ";
            if (maze[y][x].topWall)
                std::cout << "---";
            else
                std::cout << "   ";
        }
        std::cout << " \n";

        // Side walls + spaces
        for (int x = 0; x < N; x++) {
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
    for (int x = 0; x < N; x++) {
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

void setWall(Mouse &mouse, int x, int y, Direction dir) {
    auto &maze = mouse.mazeMatrix;

    maze[y][x].walls |= (1 << dir);

    int dx = 0, dy = 0;
    switch (dir) {
        case TOP:    dy = -1; break;
        case RIGHT:  dx = +1; break;
        case BOTTOM: dy = +1; break;
        case LEFT:   dx = -1; break;
    }

    int nx = x + dx;
    int ny = y + dy;

    if (nx >= 0 && nx < 16 && ny >= 0 && ny < 16) {
        maze[ny][nx].walls |= (1 << opposite(dir));
    }
}

void floodFill(Mouse &mouse)
{
    // timer to check computation time
    auto start = std::chrono::high_resolution_clock::now();

    for (int i : range(MAZE_SIZE)) {
        for (int j : range(MAZE_SIZE)) {
            mouse.floodMatrix[i][j] = 255;  // High initial value
        }
    }

    std::vector<Point> floodQueue;
    floodQueue.reserve(256);

    mouse.floodMatrix[7][7] = 0;  // Distance to center is 0
    mouse.floodMatrix[7][8] = 0;
    mouse.floodMatrix[8][7] = 0;
    mouse.floodMatrix[8][8] = 0;

    // Add goal cells to queue
    floodQueue.insert(floodQueue.begin(), {7, 7});
    floodQueue.insert(floodQueue.begin(), {7, 8});
    floodQueue.insert(floodQueue.begin(), {8, 7});
    floodQueue.insert(floodQueue.begin(), {8, 8});

    // Note: Must be in the same order as the bit fields in Cell!
    const Point directions[4] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}};
    
    int c = 0; // Cell counter
    while (!floodQueue.empty()) {
        Point current = floodQueue.back();
        floodQueue.pop_back();
        c++;

        int x = current.x, y = current.y;
        int dist = mouse.floodMatrix[y][x];

        for (int i : range(len(directions))) {
            int nx = x + directions[i].x;
            int ny = y + directions[i].y;

            if (nx < 0 || nx >= 16 || ny < 0 || ny >= 16)
                continue;

            bool wallHere = (mouse.mazeMatrix[y][x].walls & (1 << i));
            bool wallThere = (mouse.mazeMatrix[ny][nx].walls & (1 << opposite(i)));
        
            if (wallHere || wallThere) {
                // std::cout << "Skipping wall dir " << i << " from (" << x << "," << y << ") to (" << nx << "," << ny << ")\n";
                continue;
            }

            if (mouse.floodMatrix[ny][nx] > dist + 1) {
                mouse.floodMatrix[ny][nx] = dist + 1;
                floodQueue.insert(floodQueue.begin(), {nx, ny});
            }
        }
    }

    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> duration = end - start;
    printMaze(mouse);
    std::cout << "Checked " << c << " cells in " << duration.count() << " ms." << std::endl;
}

void aStar()
{
    ;
}

std::vector<std::string> stateMachine(const std::string &path)
{
    int n;
    int state = START;
    std::vector<std::string> output;

    for (char c : path)
    {
        std::cout << c << " State: " << state << std::endl;
        switch (state)
        {
        case START:
            if (c == 'F')
            {
                n = 1;
                state = ORTHO;
            }
            break;

        case ORTHO:
            if (c == 'F')
            {
                n++;
            }
            else if (c == 'L')
            {
                output.push_back("FWD" + std::to_string(n));
                state = ORTHO_L;
            }
            else if (c == 'R')
            {
                output.push_back("FWD" + std::to_string(n));
                state = ORTHO_R;
            }
            else if (c == 'S')
            {
                output.push_back("FWD" + std::to_string(n));
                output.push_back("STOP");
            }
            break;

        case ORTHO_L:
            if (c == 'F')
            {
                n = 2;
                output.push_back("SS90EL");
                state = ORTHO;
            }
            else if (c == 'L')
            {
                output.push_back("SS90EL");
                output.push_back("FWD1");
            }
            else if (c == 'R')
            {
                output.push_back("SS90EL");
                output.push_back("FWD1");
                state = ORTHO_R;
            }
            else if (c == 'S')
            {
                output.push_back("SS90EL");
                output.push_back("FWD1");
                output.push_back("STOP");
            }
            break;

        case ORTHO_R:
            if (c == 'F')
            {
                n = 2;
                output.push_back("SS90ER");
                state = ORTHO;
            }
            else if (c == 'L')
            {
                output.push_back("SS90ER");
                output.push_back("FWD1");
                state = ORTHO_L;
            }
            else if (c == 'R')
            {
                output.push_back("SS90ER");
                output.push_back("FWD1");
            }
            else if (c == 'S')
            {
                output.push_back("SS90ER");
                output.push_back("FWD1");
                output.push_back("STOP");
            }
            break;
        }
    }

    return output;
}

#define START 0
#define ORTHO 1
#define ORTHO_L 2
#define ORTHO_R 3
#define ORTHO_LL 4
#define ORTHO_RR 5
#define DIAG_LR 6
#define DIAG_RR 7
#define DIAG_RL 8
#define DIAG_LL 9
#define STOP 10

std::vector<std::string> stateMachineGoated(const std::string &path)
{
    int n;
    int state = START;
    std::vector<std::string> output;

    for (char c : path)
    {
        std::cout << c << " State: " << state << std::endl;
        switch (state)
        {
        case START:
            if (c == 'F')
            {
                n = 1;
                state = ORTHO;
            }
            else if (c == 'S')
            {
                state = STOP;
                output.push_back("STOP");
            }
            break;

        case ORTHO:
            if (c == 'F')
            {
                n++;
            }
            else if (c == 'L')
            {
                output.push_back("FWD" + std::to_string(n));
                state = ORTHO_L;
            }
            else if (c == 'R')
            {
                output.push_back("FWD" + std::to_string(n));
                state = ORTHO_R;
            }
            else if (c == 'S')
            {
                output.push_back("FWD" + std::to_string(n));
                output.push_back("STOP");
                state = STOP;
            }
            break;

        case ORTHO_L:
            if (c == 'F')
            {
                output.push_back("SS90L");
                n = 2;
                state = ORTHO;
            }
            else if (c == 'L')
            {
                state = ORTHO_LL;
            }
            else if (c == 'R')
            {
                output.push_back("SD45L");
                n = 2;
                state = DIAG_LR;
            }
            else if (c == 'S')
            {
                output.push_back("SS90EL");
                output.push_back("FWD1");
                output.push_back("STOP");
                state = STOP;
            }
            break;

        case ORTHO_R:
            if (c == 'F')
            {
                output.push_back("SS90R");
                n = 2;
                state = ORTHO;
            }
            else if (c == 'L')
            {
                output.push_back("SD45R");
                n = 2;
                state = DIAG_RL;
            }
            else if (c == 'R')
            {
                state = ORTHO_RR;
            }
            else if (c == 'S')
            {
                output.push_back("SS90ER");
                output.push_back("FWD1");
                output.push_back("STOP");
                state = STOP;
            }
            break;

        case ORTHO_LL:
            if (c == 'F')
            {
                output.push_back("SS180L");
                n = 2;
                state = ORTHO;
            }
            else if (c == 'R')
            {
                output.push_back("SD135L");
                n = 2;
                state = DIAG_LR;
            }
            else if (c == 'S')
            {
                output.push_back("SS180L");
                output.push_back("FWD1");
                output.push_back("STOP");
                state = STOP;
            }
            break;

        case ORTHO_RR:
            if (c == 'F')
            {
                output.push_back("SS180R");
                n = 2;
                state = ORTHO;
            }
            else if (c == 'L')
            {
                output.push_back("SD135R");
                n = 2;
                state = DIAG_RL;
            }
            else if (c == 'S')
            {
                output.push_back("SS180R");
                output.push_back("FWD1");
                output.push_back("STOP");
                state = STOP;
            }
            break;

        case DIAG_LR:
            if (c == 'F')
            {
                output.push_back("DIA" + std::to_string(n));
                output.push_back("DS45R");
                n = 2;
                state = ORTHO;
            }
            else if (c == 'L')
            {
                n++;
            }
            else if (c == 'R')
            {
                state = DIAG_RR;
            }
            else if (c == 'S')
            {
                output.push_back("DIA" + std::to_string(n));
                output.push_back("DS45R");
                output.push_back("STOP");
                state = STOP;
            }
            break;

        case DIAG_RR:
            if (c == 'F')
            {
                output.push_back("DIA" + std::to_string(n));
                output.push_back("DD90R");
                n = 2;
                state = ORTHO;
            }
            else if (c == 'L')
            {
                output.push_back("DIA" + std::to_string(n));
                output.push_back("DD90R");
                n = 2;
                state = DIAG_RL;
            }
            else if (c == 'S')
            {
                output.push_back("DIA" + std::to_string(n));
                output.push_back("DS135R");
                output.push_back("FWD1");
                output.push_back("STOP");
                state = STOP;
            }
            break;

        case DIAG_RL:
            if (c == 'F')
            {
                output.push_back("DIA" + std::to_string(n));
                output.push_back("DS45L");
                n = 2;
                state = ORTHO;
            }
            else if (c == 'L')
            {
                state = DIAG_LL;
            }
            else if (c == 'R')
            {
                n++;
                state = DIAG_LR;
            }
            else if (c == 'S')
            {
                output.push_back("DIA" + std::to_string(n));
                output.push_back("DS45L");
                output.push_back("FWD1");
                output.push_back("STOP");
                state = STOP;
            }
            break;

        case DIAG_LL:
            if (c == 'F')
            {
                output.push_back("DIA" + std::to_string(n));
                output.push_back("DS135L");
                n = 2;
                state = ORTHO;
            }
            else if (c == 'R')
            {
                output.push_back("DIA" + std::to_string(n));
                output.push_back("DD90L");
                n = 2;
                state = DIAG_LR;
            }
            else if (c == 'S')
            {
                output.push_back("DIA" + std::to_string(n));
                output.push_back("DS135L");
                output.push_back("FWD1");
                output.push_back("STOP");
                state = STOP;
            }
            break;

        case STOP:
            break;
        }
    }

    return output;
}

int main()
{
    // Initialize mouse
    Mouse mouse;
    // for (int i = 0; i < 27; i++){
    //     // gpio_init(allPins[i]);
    //     // gpio_set_dir(allPins[i], GPIO_OUT);
    //     ;
    // }

    // stdio_init_all();

    // main loop
    // gpio_put(ledPin, 1);

    // floodfill
    // if (mouse.solvingType == 0) {

    //     int x0 = mouse.cellPos[0];
    //     int y0 = mouse.cellPos[1];
    // mouse.mazeMatrix[x0][y0] = cellConfig(mouse);

    // mouse.mazeMatrix[13][4] = {0,1,1,0,1};
    for (int x = 5; x <= 10; ++x) {
        setWall(mouse, x, 6, TOP);
    }
    
    floodFill(mouse);
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