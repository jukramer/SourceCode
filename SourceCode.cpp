#include <stdio.h>
#include <iostream>
#include <list>
#include <algorithm>
#include <vector>
// #include "pico/stdlib.h"
// #include "pico/cyw43_arch.h"

///////////////// CLASSES ///////////////////////
// class Node
// {
//     public:
//         Node(const std::list<int>& pos, double g, double h) {
//             const std::list<int> pos = pos;
//             double g = g;
//             double h = h;
//         }
// };


class Mouse
{
    public:
        int cellX=1, cellY=1; // position in maze grid
        int orientation = 0; // heading/orientation of mouse: 0=up, 1=right, 2=down, 3=left

        Mouse() {
            
        }
};


////////////////// FUNCTIONS ////////////////////
std::vector<int> cellMapper(Mouse mouse)
{
    // assign dl, dr, and df via tof sensors later!

    double dl, dr, df; // distance values for front, left, and right
    double tl, tr, tf; // threshold values for front, left, and right
    std::vector<double> dVec = {dl, df, dr};
    std::vector<double> tVec = {tl, tf, tr};
    std::vector<int> cellVec = {0, 0, 0, 0}; // contains value for wall in each cell: left, front, right, bottom. 1=wall, 0=no wall

    // places wall around cells if detected 
    for (int i=0; i <= dVec.size(); i++) {
        if (dVec[i] < tVec[i]) {
            cellVec[i] = 1;
        }
    }

    // shifts cellVec depending on mouse orientation
    std::rotate(cellVec.begin(), cellVec.end()-mouse.orientation, cellVec.end());

    return cellVec;    
}


void dfs()
{
    int n = 16; // n for nxn matrix
    
    std::vector<std::vector<int>> matrix(n, std::vector<int>(n,0)); // nxn matrix
}

void calcHeuristic()
{
    ;
}

void aStar()
{
    ;
}


int main()
{
    // stdio_init_all();

    // // Initialise the Wi-Fi chip
    // if (cyw43_arch_init()) {
    //     printf("Wi-Fi init failed\n");
    //     return -1;
    // }

    // // Example to turn on the Pico W LED
    // cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

    // while (true) {
    //     printf("Hello, world!\n");
    //     sleep_ms(1000);
    // }

    Mouse mouse;
    mouse.orientation = 2;

    std::vector<int> vec = cellMapper(mouse);
    
    for (int i: vec) {
        std::cout << i;
    }
}

