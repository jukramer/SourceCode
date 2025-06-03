#include "API.h"

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

    CURRENT_FLOOD_GEN++;
    floodQueue.reset();

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

/*std::string fastestPath(Mouse &mouse)
{
    // TODO: encourage to take diagonals

    // Fastest path solely based on floodfill distance
    std::string path = "";

    Point currentCell = {0, 15};
    Direction currentDir = TOP;
    // Initial point
    for (int i = 0; i < 100; i++)
    {
        Direction minFloodFillDir;
        int minFloodFill = 255;
        int x1, y1;

        uint8_t mask = moveMask[currentCell.y][currentCell.x];
        for (int i = 0; i < 4; ++i)
        {
            if (!(mask & (1 << i)))
                continue;

            int nx = currentCell.x + DIRECTIONS[i].x;
            int ny = currentCell.y + DIRECTIONS[i].y;

            if (floodMatrix[ny][nx] <= minFloodFill)
            {
                minFloodFill = floodMatrix[ny][nx];
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

        std::cout << currentCell.x << " " << currentCell.y << " --> " << x1 << " " << y1 << " " << floodMatrix[y1][x1] << std::endl;

        currentCell = {x1, y1};
        currentDir = minFloodFillDir;
        std::cout << currentDir << " " << minFloodFillDir << std::endl;
        if (floodMatrix[y1][x1] == 0)
        {
            std::cout << "Breaking" << std::endl;
            break;
        }
    }
    path += 'S';

    return path;
}*/