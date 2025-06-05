#include "API.h"
#include <tuple>

force_inline bool isUnvisited(int x, int y)
{
    return FLOOD_GEN_MATRIX[y][x] != CURRENT_FLOOD_GEN;
}

force_inline void setCost(int x, int y, byte dist)
{
    FLOOD_MATRIX[y][x] = dist;
    FLOOD_GEN_MATRIX[y][x] = CURRENT_FLOOD_GEN;
}

Queue<Location> floodQueue;

force_inline void tryPropagate(int dir, int x, int y, int dist, uint8_t mask)
{
    if (!(mask & (1 << dir)))
        return;

    int nx = x + OFFSET_LOCATIONS[dir].x;
    int ny = y + OFFSET_LOCATIONS[dir].y;
    uint8_t ndist = FLOOD_MATRIX[ny][nx];
    if (isUnvisited(nx, ny) || ndist > dist + 1)
    {
        setCost(nx, ny, dist + 1);
        floodQueue.push({nx, ny});
    }
}

force_inline void initBackFill(int dir, int x, int y, uint8_t mask)
{
    if (!(mask & (1 << dir)))
        return;

    int nx = x + OFFSET_LOCATIONS[dir].x;
    int ny = y + OFFSET_LOCATIONS[dir].y;

    auto &neighbor = MAZE_MATRIX[ny][nx];
    if (!neighbor.visited)
    {
        setCost(nx, ny, 10); // Set initial distance for unknown neighbors
        floodQueue.push({nx, ny});
    }
}
void printMaze()
{
    const int N = 16;

    for (int y = 0; y < N; y++)
    {
        // Top walls
        for (int x = 0; x < N; x++)
        {
            printf(" ");
            if (MAZE_MATRIX[y][x].north)
                printf("---");
            else
                printf("   ");
        }
        printf(" \n");

        // Side walls + spaces
        for (int x = 0; x < N; x++)
        {
            if (MAZE_MATRIX[y][x].west)
                printf("|");
            else
                printf(" ");

            printf("%3d", (int) FLOOD_MATRIX[y][x]);
        }

        // Right wall of the last cell
        if (MAZE_MATRIX[y][N - 1].east)
            printf("|\n");
        else
            printf(" \n");
    }

    // Bottom walls of the last row
    for (int x = 0; x < N; x++)
    {
        printf(" ");
        if (MAZE_MATRIX[N - 1][x].south)
            printf("---");
        else
            printf("   ");
    }
    printf(" \n");
}


void floodFill()
{
    auto begin = time_us_64();

    STATE = STATE_MAP_EXPLORE;

    CURRENT_FLOOD_GEN++;
    floodQueue.reset();

    // Set high value
    for (int i; i<15; i++) {
        for (int j; j<15; j++) {
            if (!(i==7 || i==8 || j==7 || j==8)) {
                FLOOD_MATRIX[j][i] = 255;
            }
        }
    }

    // printMaze();

    if (STATE == STATE_MAP_EXPLORE)
    {
        setCost(7, 7, 0); // Cost to start is 0
        setCost(7, 8, 0);
        setCost(8, 7, 0);
        setCost(8, 8, 0);

        // Add goal cells to queue
        floodQueue.push({7, 7});
        floodQueue.push({7, 8});
        floodQueue.push({8, 7});
        floodQueue.push({8, 8});
    }
    else if (STATE == STATE_MAP_EXPLORE_BACK)
    {
        setCost(0, MAZE_SIZE - 1, 0); // Start from bottom left corner
        floodQueue.push({0, MAZE_SIZE - 1});

        for (int x = 0; x < MAZE_SIZE; x++)
        {
            for (int y = 0; y < MAZE_SIZE; y++)
            {
                auto &cell = MAZE_MATRIX[y][x];
                if (!cell.visited)
                    continue; // Only consider known cells

                uint8_t mask = MOVE_MATRIX[y][x];
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
        Location current = floodQueue.pop();
        c++;

        int x = current.x, y = current.y;
        int dist = FLOOD_MATRIX[y][x];

        uint8_t mask = MOVE_MATRIX[y][x];
        tryPropagate(0, x, y, dist, mask);
        tryPropagate(1, x, y, dist, mask);
        tryPropagate(2, x, y, dist, mask);
        tryPropagate(3, x, y, dist, mask);
    }

    auto end = time_us_64();
    auto duration = (end - begin) / 1000.0f;
    printf("Flood fill completed in %.2f ms, processed %d cells.\n", duration, c);

    printMaze();
}

struct CellCoord {
    int C;
    int R;

    CellCoord operator+(const CellCoord c2) const {
        CellCoord result;
        result.C = C + c2.C;
        result.R = R + c2.R;
        
        return result;
    }

    CellCoord operator-(const CellCoord c2) const {
        CellCoord result;
        result.C = C - c2.C;
        result.R = R - c2.R;
        
        return result;
    }
};

#include <string>
#include <algorithm>

// Enum for mouse heading
enum class Heading { NORTH, EAST, SOUTH, WEST };

// Function to check if a cell is a goal cell
bool isGoalCell(int x, int y) {
    return (x == 7 && y == 7) || (x == 8 && y == 7) ||
           (x == 7 && y == 8) || (x == 8 && y == 8);
}

// Function to find the fastest path from {0,15} to a goal cell
std::string findFastestPath() {
    std::string path; // Dynamic string for path commands

    // Starting position and heading
    int x = 0, y = 15; // Starting at {0,15} (FLOOD_MATRIX[15][0])
    Heading heading = Heading::NORTH; // Assume initial heading is North

    while (!isGoalCell(x, y)) {
        // Define movements relative to current heading
        int forwardX = x, forwardY = y, leftX = x, leftY = y, rightX = x, rightY = y;
        char moveCommand = '\0';

        // Determine coordinates of forward, left, and right cells based on heading
        switch (heading) {
            case Heading::NORTH:
                forwardX = x; forwardY = y - 1; // Up
                leftX = x - 1; leftY = y;       // West
                rightX = x + 1; rightY = y;     // East
                break;
            case Heading::EAST:
                forwardX = x + 1; forwardY = y; // Right
                leftX = x; leftY = y - 1;       // North
                rightX = x; rightY = y + 1;     // South
                break;
            case Heading::SOUTH:
                forwardX = x; forwardY = y + 1; // Down
                leftX = x + 1; leftY = y;       // East
                rightX = x - 1; rightY = y;     // West
                break;
            case Heading::WEST:
                forwardX = x - 1; forwardY = y; // Left
                leftX = x; leftY = y + 1;       // South
                rightX = x; rightY = y - 1;     // North
                break;
        }

        // Evaluate floodfill values for forward, left, and right cells
        int minValue = 999; // Large number for unreachable cells
        int nextX = x, nextY = y;

        // Check forward cell
        if (forwardY >= 0 && forwardY < 16 && forwardX >= 0 && forwardX < 16 &&
            FLOOD_MATRIX[forwardY][forwardX] < minValue) {
            minValue = FLOOD_MATRIX[forwardY][forwardX];
            nextX = forwardX;
            nextY = forwardY;
            moveCommand = 'F';
        }

        // Check left cell
        if (leftY >= 0 && leftY < 16 && leftX >= 0 && leftX < 16 &&
            FLOOD_MATRIX[leftY][leftX] < minValue) {
            minValue = FLOOD_MATRIX[leftY][leftX];
            nextX = leftX;
            nextY = leftY;
            moveCommand = 'L';
        }

        // Check right cell
        if (rightY >= 0 && rightY < 16 && rightX >= 0 && rightX < 16 &&
            FLOOD_MATRIX[rightY][rightX] < minValue) {
            minValue = FLOOD_MATRIX[rightY][rightX];
            nextX = rightX;
            nextY = rightY;
            moveCommand = 'R';
        }

        // If no valid move is found, stop (shouldn't happen with correct FLOOD_MATRIX)
        if (moveCommand == '\0') {
            path += 'S';
            break;
        }

        // Update heading based on the move
        if (moveCommand == 'L') {
            switch (heading) {
                case Heading::NORTH: heading = Heading::WEST; break;
                case Heading::EAST: heading = Heading::NORTH; break;
                case Heading::SOUTH: heading = Heading::EAST; break;
                case Heading::WEST: heading = Heading::SOUTH; break;
            }
        } else if (moveCommand == 'R') {
            switch (heading) {
                case Heading::NORTH: heading = Heading::EAST; break;
                case Heading::EAST: heading = Heading::SOUTH; break;
                case Heading::SOUTH: heading = Heading::WEST; break;
                case Heading::WEST: heading = Heading::NORTH; break;
            }
        } // No heading change for 'F'

        // Append the move command and update position
        path += moveCommand;
        x = nextX;
        y = nextY;
    }

    // Append stop command when goal is reached
    path += 'S';

    return path;
}



// std::vector<CellCoord> getNeighbors(CellCoord coord) {
//     std::vector<CellCoord> neighbors;

//     CellCoord diff[4] = {{-1,0}, {0,1}, {1,0}, {0,-1}};

//     for (CellCoord d : diff) {
//         CellCoord neighbor = coord - d;
//         if (!(neighbor.C < 0 || neighbor.C > 15 || neighbor.R < 0 || neighbor.R > 15)) {
//             neighbors.push_back(neighbor)    ;    
//         }
//     }
    
//     return neighbors;
// }

// std::string fastestPath() {
//     #define forward 100
//     #define right 101
//     #define left 102
//     #define backward 103

//     CellCoord cell = {0, 15};

//     std::vector<CellCoord> neighbors = getNeighbors(cell);
    
// }


std::string fastestPath()
{
    // TODO: encourage to take diagonals

    // Fastest path solely based on floodfill distance
    std::string path = "";

    Location currentCell = {0, 15};
    Direction currentDir = Direction::TOP;
    // Initial point
    for (int i = 0; i < 100; i++)
    {
        Direction minFloodFillDir;
        int minFloodFill = 255;
        int x1, y1;

        uint8_t mask = MOVE_MATRIX[currentCell.y][currentCell.x];
        for (int i = 0; i < 4; ++i)
        {
            if (!(mask & (1 << i)))
                continue;

            int nx = currentCell.x + DIRECTIONS[i].x;
            int ny = currentCell.y + DIRECTIONS[i].y;

            if (FLOOD_MATRIX[ny][nx] <= minFloodFill)
            {
                minFloodFill = FLOOD_MATRIX[ny][nx];
                minFloodFillDir = (Direction)i;
                x1 = nx;
                y1 = ny;
            }
        }

        std::cout << currentDir << " " << minFloodFillDir << std::endl;
        // Append path depending on relative dir of next cell and mouse
        if (currentDir == minFloodFillDir)
        {
            path += 'F';
            std::cout << "Pushing F..." << std::endl;
        }
        else if (minFloodFillDir - currentDir == -1 || minFloodFillDir - currentDir == 3)
        {
            path += 'L';
            std::cout << "Pushing L..." << std::endl;
        }
        else if (minFloodFillDir - currentDir == 1 || minFloodFillDir - currentDir == -3)
        {
            path += 'R';
            std::cout << "Pushing R..." << std::endl;
        }

        std::cout << currentCell.x << " " << currentCell.y << " --> " << x1 << " " << y1 << " " << FLOOD_MATRIX[y1][x1] << std::endl;

        currentCell = {x1, y1};
        currentDir = minFloodFillDir;
        std::cout << currentDir << " " << minFloodFillDir << std::endl;
        if (FLOOD_MATRIX[y1][x1] == 0)
        {
            std::cout << "Breaking" << std::endl;
            break;
        }
    }
    path += 'S';

    return path;
} 