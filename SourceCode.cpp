#include <stdio.h>
#include <iostream>
#include <list>
#include <algorithm>
#include <vector>
#include <stack>
#include "functions.h"
// #include "pico/stdlib.h"
// #include "pico/cyw43_arch.h"
// #include "pins.hpp"


///////////////// CLASSES ///////////////////////

class Node
{
    public:
        double g, h; // A* values
        std::vector<float> pos; //position vector: <x,y>

        Node(std::vector<float> position, double g, double h) {
            std::vector<float> pos = position;
            g = g;
            h = h;
        }
};


class Mouse
{
    public:
        int cellX=0, cellY=0; // position in maze grid
        std::vector<int> cellPos = {cellX, cellY};

        std::vector<std::vector<int>> adjacentCells;

        int targetCellX, targetCellY; // cell that the mouse wants to travel to next
        std::vector<int> targetCell = {targetCellX, targetCellY};

        bool posChanged = false; // true if the cell position of the mouse has changed
        int orientation = 0; // heading/orientation of mouse: 0=up, 1=right, 2=down, 3=left
        std::stack<std::vector<int>> cellPath;
        std::vector<std::vector<int>> visitedCells;

        int phase = 0; //0: mapping phase   1: pathfinding phase   2: solving phase

        // Maze map in matrix
        int n = 16;
        std::vector<int> mazeMatrix[16][16]; 
        
        bool turnLeft = false, turnRight = false, goStraight = false; // variables that determine if specific movements are possible
        std::vector<bool> possMovements = {turnLeft, turnRight, goStraight};

        // Flood matrix
        int floodMatrix[16][16];

        Mouse() {
            for (int i=0; i<n; i++) {
                for (int j=0; j<n; j++) {
                    floodMatrix[i][j] = manhattanDist({i, j}, {{7,7}, {7,8}, {8,7}, {8,8}});
                    std::cout << floodMatrix[i][j] << " ";
                }
                std::cout << std::endl;
            }
            std::cout << floodMatrix[8][8];
        }
};


////////////////// FUNCTIONS ////////////////////

std::vector<int> cellConfig(Mouse mouse)
{
    // assign dl, dr, and df via tof sensors later!

    double dl, dr, df; // distance values for front, left, and right
    double tl, tr, tf; // threshold values for front, left, and right
    std::vector<double> dVec = {dl, df, dr};
    std::vector<double> tVec = {tl, tf, tr};
    std::vector<int> cellVec = {0, 0, 0, 0}; // contains value for wall in each cell, and whether it has been checked: left, front, right, bottom, checked (y/n). 1=wall, 0=no wall

    // places wall around cells if detected 
    for (int i=0; i <= dVec.size(); i++) {
        if (dVec[i] < tVec[i]) {
            cellVec[i] = 1;
        }
    }

    // shifts cellVec depending on mouse orientation
    std::rotate(cellVec.begin(), cellVec.end()-mouse.orientation, cellVec.end());

    cellVec.insert(cellVec.end(), 1); // adds a 1 to end of cell to signify it has been checked

    return cellVec;    
}


bool contains(std::stack<std::vector<int>> stack, const std::vector<int>& target)
{
    //Convert stack to temporary vector while searching
    while (!stack.empty()) {
        if (stack.top() == target) {
            return true;
        }
        stack.pop();
    }
    return false;
}


void turnPossible(Mouse mouse) 
{
    if (mouse.mazeMatrix[mouse.cellX][mouse.cellY][0] == 0) {
        mouse.turnLeft = true;
    } else if (mouse.mazeMatrix[mouse.cellX][mouse.cellY][0] == 1) {
        mouse.turnLeft = false;
    }

    if (mouse.mazeMatrix[mouse.cellX][mouse.cellY][0] == 0) {
        mouse.turnLeft = true;
    } else if (mouse.mazeMatrix[mouse.cellX][mouse.cellY][0] == 1) {
        mouse.turnLeft = false;
    }

    if (mouse.mazeMatrix[mouse.cellX][mouse.cellY][0] == 0) {
        mouse.turnLeft = true;
    } else if (mouse.mazeMatrix[mouse.cellX][mouse.cellY][0] == 1) {
        mouse.turnLeft = false;
    }
}


void mapping(Mouse mouse) // handle overall movement of the mouse
{
    // movement handling here

    if (mouse.posChanged) {
        mouse.posChanged = false;
        if (mouse.cellPath.top() != mouse.cellPos) {
            mouse.cellPath.push({mouse.cellX, mouse.cellY});
        }

        // mouse.mazeMatrix[mouse.cellX-1][mouse.cellY-1] = cellConfig(mouse); // assign cell vector to each matrix cell
        
        // Find adjacent cells to mouse
        std::vector<std::vector<int>> adjacentCells;
        adjacentCells[0] = {mouse.cellX-1, mouse.cellY}; // cell to left of mouse
        adjacentCells[1] = {mouse.cellX, mouse.cellY-1}; // cell in front of mouse
        adjacentCells[2] = {mouse.cellX+1, mouse.cellY}; // cell to right of mouse

        std::rotate(adjacentCells.begin(), adjacentCells.end()-mouse.orientation, adjacentCells.end()); // account for mouse orientation

        // set mouse adjacentCells
        mouse.adjacentCells = adjacentCells;

        // Find if adjacent cell has been visited or not
        if (mouse.possMovements[0] && contains(mouse.cellPath, adjacentCells[0])) {
            mouse.targetCell = adjacentCells[0];
        } else if (mouse.possMovements[1] && contains(mouse.cellPath, adjacentCells[1])) {
            mouse.targetCell = adjacentCells[1];
        } else if (mouse.possMovements[2] && contains(mouse.cellPath, adjacentCells[2])) {
            mouse.targetCell = adjacentCells[2];
        } else {
            mouse.targetCell = mouse.cellPath.top();
            mouse.cellPath.pop();
        }
    }

    // Check if all cells have been mapped
    int uncheckedCells = 0;
    
    while (int checkedCells = 0) {
        for (int i=0; i < mouse.n; i++) {
            for (int j=0; i<mouse.n; j++) {
                if (mouse.mazeMatrix[i][j][4]==0) {
                    uncheckedCells += 1;
                }
            }
        }  
    } 

    if (uncheckedCells = 0) {
        mouse.phase = 1;
    }
}


void floodFill()
{
    ;
}


void aStar()
{
    ;
}


int main()
{
    // Initialize mouse 
    Mouse mouse;
    for (int i = 0; i < 27; i++){
        // gpio_init(allPins[i]);
        // gpio_set_dir(allPins[i], GPIO_OUT);
        ;
    }

    
    
    // stdio_init_all();

    // main loop
    // while (true) {
    //     printf("LED on!");
    //     // gpio_put(ledPin, 1);
    //     // sleep_ms(500);
    //     printf("LED off!");
    //     // gpio_put(ledPin, 0);
    //     // sleep_ms(500);
    // }


    // while (true) {

    //     // Mapping phase
    //     if (mouse.phase == 0) {
    //         mapping(mouse);
    //     }
    //     // Pathfinding phase (super short)
    //     else if (mouse.phase == 1) {
    //         ;
    //     }
    //     // Solving phase (mouse zoom)
    //     else if (mouse.phase == 2) {
    //         ;
    //     }
    // }
    std::cin.get();
}

