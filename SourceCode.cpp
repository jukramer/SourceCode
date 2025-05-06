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
        int solvingType = 0; // 0 = floodfill, 1 = a*, 2 = custom

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
            // Initialize mazeMatrix
            for (int i=0; i<16; i++) {
                for (int j=0; j<16; j++) {
                    mazeMatrix[i][j]={0,0,0,0,0};
                }
            }

            // Initialize floodMatrix
            for (int i=0; i<n; i++) {
                for (int j=0; j<n; j++) {
                    floodMatrix[i][j] = 255;
                    // std::cout << floodMatrix[i][j] << " ";
                }
                // std::cout << std::endl;
            }
            // std::cout << floodMatrix[8][8];
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
    std::vector<int> cellVec = {0, 0, 0, 0}; // left, top, right, bottom, checked (y/n). 1=wall, 0=no wall

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


std::vector<std::vector<int>> getNeighbors(std::vector<int> cell) 
{
    std::vector<std::vector<int>> neighbors = {};
    int x0 = cell[0], y0 = cell[1];
    int x1 = x0-1, y1 = y0; // left cell
    if (x1 >= 0 && x1 <= 16 && y1 >= 0 && y1 <= 16) {
        neighbors.push_back({x1, y1});
    }

    int x2 = x0, y2 = y0 + 1; // top cell
    if (x2 >= 0 && x2 <= 16 && y2 >= 0 && y2 <= 16) {
        neighbors.push_back({x2, y2});
    }

    int x3 = x0+1, y3 = y0; // right cell
    if (x3 >= 0 && x3 <= 16 && y3 >= 0 && y3 <= 16) {
        neighbors.push_back({x3, y3});
    }

    int x4 = x0, y4 = y0-1; // bottom cell
    if (x4 >= 0 && x4 <= 16 && y4 >= 0 && y4 <= 16) {
        neighbors.push_back({x4, y4});
    }

    return neighbors;
}


void printFloodfill(Mouse mouse)
{
    // std::cout << "hello!!" << std::endl; 
    for (int i=0; i<16; i++) {
        for (int j=0; j<16; j++) {
            std::cout << std::setw(4)<< mouse.floodMatrix[i][j] << " ";
        }
        std::cout << std::endl;
    }
    // std::cout << floodMatrix[8][8];
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
    // gpio_put(ledPin, 1);

    
    // floodfill
    // std::cout << mouse.solvingType << std::endl; 
    if (mouse.solvingType == 0) {
        auto start = std::chrono::high_resolution_clock::now();
        // reset floodfill map
        for (int i=0; i<16; i++) {
            for (int j=0; j<16; j++) {
                mouse.floodMatrix[i][j] = 255;
                // std::cout << floodMatrix[i][j] << " ";
            }
            // std::cout << std::endl;
        }
        mouse.floodMatrix[7][7] = 0;
        mouse.floodMatrix[7][8] = 0;
        mouse.floodMatrix[8][7] = 0;
        mouse.floodMatrix[8][8] = 0;

        // std::cout << "hello" << std::endl;
        int x0 = mouse.cellPos[0];
        int y0 = mouse.cellPos[1];
        // mouse.mazeMatrix[x0][y0] = cellConfig(mouse);
        // mouse.mazeMatrix[3][3] = {1,1,0,0,1};
        mouse.mazeMatrix[13][4] = {0,1,1,0,1};
        mouse.mazeMatrix[6][5] = {0,0,0,1,1};
        mouse.mazeMatrix[6][6] = {0,0,0,1,1};
        mouse.mazeMatrix[6][7] = {0,0,0,1,1};
        mouse.mazeMatrix[6][8] = {0,0,0,1,1};
        mouse.mazeMatrix[6][9] = {0,0,0,1,1};
        mouse.mazeMatrix[6][10] = {0,0,0,1,1};
        // mouse.mazeMatrix[7][7] = {1,0,0,1,1};
        // mouse.mazeMatrix[8][7] = {1,1,0,0,1};
        // mouse.mazeMatrix[7][8] = {0,0,1,1,1};
        // mouse.mazeMatrix[8][8] = {0,1,1,0,1};

        std::queue<std::vector<int>> floodQueue;
        
        // Add goal cells to queue
        floodQueue.push({7,7});
        floodQueue.push({7,8});
        floodQueue.push({8,7});
        floodQueue.push({8,8});
        int c = 0;

        while (floodQueue.size() != 0) {
            std::vector<int> currentCell = floodQueue.front();
            floodQueue.pop();
            c++;
            int xc = currentCell[0], yc = currentCell[1];
            std::vector<std::vector<int>> neighbors = getNeighbors(currentCell);
            // std::cout << floodQueue.size()<<" - "<<xc<<" "<<yc<<std::endl;
            for (int i=0; i<4; i++) {
                    try {
                        // std::cout << neighbor[0] << " " << neighbor[1] << " " << std::endl;
                        int xn = neighbors[i][0], yn = neighbors[i][1];
                        // std::cout<<"ello"<<std::endl;
                        // std::cout<<"-------------------"<<std::endl;
                        // std::cout<<i<<" ";
                        // std::cout<<mouse.mazeMatrix[yc][xc][0]<<" ";
                        // std::cout<<mouse.floodMatrix[yn][xn]<<" ";
                        // std::cout<<mouse.floodMatrix[yc][xc]<<" ";
                        // std::cout<<"Checked: "<<xn<<" "<<yn<<" with value "<<mouse.floodMatrix[yn][xn]<<std::endl;
                        if (i==0 && mouse.mazeMatrix[yc][xc][0]==0 && mouse.floodMatrix[yn][xn] > (mouse.floodMatrix[yc][xc]+1)) {
                            mouse.floodMatrix[yn][xn] = mouse.floodMatrix[yc][xc]+1;
                            floodQueue.push(neighbors[i]);
                            // std::cout<<"Pushed "<<xn<<" "<<yn<<std::endl;
                        } if (i==1 && mouse.mazeMatrix[yc][xc][1]==0 && mouse.floodMatrix[yn][xn] > (mouse.floodMatrix[yc][xc]+1)) {
                            mouse.floodMatrix[yn][xn] = mouse.floodMatrix[yc][xc]+1;
                            floodQueue.push(neighbors[i]);
                            // std::cout<<"Pushed "<<xn<<" "<<yn<<std::endl;
                        } if (i==2 && mouse.mazeMatrix[yc][xc][2]==0 && mouse.floodMatrix[yn][xn] > (mouse.floodMatrix[yc][xc]+1)) {
                            mouse.floodMatrix[yn][xn] = mouse.floodMatrix[yc][xc]+1;
                            floodQueue.push(neighbors[i]);
                            // std::cout<<"Pushed "<<xn<<" "<<yn<<std::endl;

                        } if (i==3 && mouse.mazeMatrix[yc][xc][3]==0 && mouse.floodMatrix[yn][xn] > (mouse.floodMatrix[yc][xc]+1)) {
                            mouse.floodMatrix[yn][xn] = mouse.floodMatrix[yc][xc]+1;
                            floodQueue.push(neighbors[i]);
                            // std::cout<<"Pushed "<<xn<<" "<<yn<<std::endl;
                        } 
                        // std::cout<<"ello"<<std::endl;
                    }
                    catch (...) {
                        ;
                    }
                }
            }
            // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> duration = end-start;
            printFloodfill(mouse);
            std::cout<<"Checked "<<c<<" cells in "<<duration.count()<<" ms."<<std::endl;
        }
        // std::cout<<"hellohello"<<std::endl;
        
    
    // std::cin.get();
}

