#include <stdio.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <iomanip>


int manhattanDist(std::vector<int> cell1, std::vector<std::vector<int>> cells)
{
    std::vector<int> distances = {};

    for (int i=0; i<cells.size(); i++) {
        int d = abs(cell1[0]-cells[i][0]) + abs(cell1[1]-cells[i][1]);
        distances.push_back(d);
    }

    int minVal = *std::min_element(distances.begin(), distances.end());

    return minVal;
}


