#include <stdio.h>
#include <iostream>
#include <list>
#include <algorithm>
#include <vector>
#include <stack>
// #include "pico/stdlib.h"
// #include "pico/cyw43_arch.h"

///////////////// CLASSES ///////////////////////
class Node
{
    public:
        double g, h; // A* values
        std::vector<float> pos; // position vector: <x,y>

        Node(std::vector<float> pos, double g, double h) {
            std::vector<float> pos = pos;
            g = g;
            h = h;
        }
};


class Mouse
{
    public:
        int cellX=1, cellY=1; // position in maze grid
        bool posChanged = false; // true if the cell position of the mouse has changed
        int orientation = 0; // heading/orientation of mouse: 0=up, 1=right, 2=down, 3=left
        std::stack<std::vector<int>> cellPath;

        int phase = 0; //0: mapping phase   1: pathfinding phase   2: solving phase

        // Maze map in matrix
        int n = 16;
        std::vector<std::vector<std::vector<int>>> mazeMatrix{n, std::vector<std::vector<int>>(n, std::vector<int>(5, 0))};  
        
        bool turnLeft = false, turnRight = false, goStraight = false; // variables that determine if specific movements are possible
        std::vector<bool> possMovements = {turnLeft, turnRight, goStraight};

        Mouse() {
            ;
        }
        
};


////////////////// FUNCTIONS ////////////////////

void mapping(Mouse mouse) // handle overall movement of the mouse
{
    // movement handling here

    if (mouse.posChanged) {
        mouse.cellPath.push({mouse.cellX, mouse.cellY});
        dfs(mouse); 
    }

    if (std::none_of(mouse.possMovements.begin(), mouse.possMovements.end(), [](bool b) { return b; })) {

    };
}

std::vector<int> cellMapper(Mouse mouse)
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


void dfs(Mouse mouse)
{
    mouse.mazeMatrix[mouse.cellX-1][mouse.cellY-1] = cellMapper(mouse); // assign cell vector to each matrix cell
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
    // while (true) {
    //     printf("Hello, world!\n");
    //     sleep_ms(1000);
    // }

    // Initialize mouse
    // stdio_init_all();
    Mouse mouse;

    // main loop
    while (true) {

        // Mapping phase
        if (mouse.phase == 0) {
            mapping(mouse);
        }
        // Pathfinding phase (super short)
        else if (mouse.phase == 1) {
            ;
        }
        // Solving phase (mouse zoom)
        else if (mouse.phase == 2) {
            ;
        }
    }
    
    


}

