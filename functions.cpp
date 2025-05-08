#include <stdio.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <iomanip>
#include <string>

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


//////////////// FUNCTIONS //////////////////////
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


std::vector<std::string> stateMachineSimple(const std::string &path)
{
    int n;
    int state = START;
    std::vector<std::string> output;

    for (char c : path) {
        std::cout << c << " State: " << state << std::endl;
        switch (state) {
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