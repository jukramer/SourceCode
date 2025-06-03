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

#include "API.h"

#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "pico/cyw43_arch.h"
#include "hardware/pwm.h"
#include "pins.hpp"
#include "pico/time.h"

/////////////// DEFINITIONS /////////////////////
// Global States
#define INIT 0
#define IDLE 1
#define MAP_FLOODFILL 2
#define MAP_FLOODFILL_BACK 5
#define FIND_ROUTE 6
#define FAST_RUN 7

// Mapping States

// Controller Constants
#define KP_V 200.0 // Velocity controller
#define KI_V 40.0
#define KD_V 5.32089706902006
#define KP_W 50.0 // Angular velocity controller
#define KI_W 5.0
#define KD_W 0.238664893739785

#define V_MAX 2.0
#define W_MAX 3.0

// Motor Constants/Variables
#define FORWARD 1
#define BACKWARD -1
#define TICKS_PER_REV 7.0
#define GEAR_RATIO 30.0
#define WHEEL_RADIUS 21.5
#define WHEEL_BASE 0.076

volatile uint totalTicksL = 0;
volatile uint totalTicksR = 0;


///////////////// CLASSES ///////////////////////

#define MAZE_SIZE 16
struct Point
{
    int x = 0;
    int y = 0;
};

Point operator+(Point p1, Point p2)
{
    return {p1.x + p2.x, p1.y + p2.y};
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
            uint8_t visited : 1;    // 1 bit for visited status
        };
        uint8_t walls; // 8 bits, for easy manipulation of all walls together
    };
};

#if defined _MSC_VER
#define force_inline inline
#else
#define force_inline __attribute__((always_inline)) inline
#endif

// Ring buffer queue for flood fill
struct Queue
{
    Point buffer[256];
    uint8_t head = 0, tail = 0;

    force_inline void reset() { head = tail = 0; }
    force_inline bool empty() const { return head == tail; }
    force_inline void push(Point p) { buffer[head++ & 255] = p; }
    force_inline Point pop() { return buffer[tail++ & 255]; }
};

enum Direction
{
    TOP = 0,
    RIGHT = 1,
    BOTTOM = 2,
    LEFT = 3
};

constexpr Direction OPPOSITE[4] = {BOTTOM, LEFT, TOP, RIGHT};
constexpr Direction ROTATE_LEFT[4] = {LEFT, TOP, RIGHT, BOTTOM};
constexpr Direction ROTATE_RIGHT[4] = {RIGHT, BOTTOM, LEFT, TOP};
constexpr char DIRECTION_CHAR[4] = {'n', 'e', 's', 'w'};

// Note: Must be in the same order as the bit fields in Cell!
constexpr Point DIRECTIONS[4] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}};

static volatile Cell mazeMatrix[MAZE_SIZE][MAZE_SIZE] = {}; // Init to empty with = {};

static volatile uint8_t floodMatrix[MAZE_SIZE][MAZE_SIZE] = {};
static volatile uint8_t floodGenMatrix[MAZE_SIZE][MAZE_SIZE] = {};
static volatile uint8_t currentFloodGen = 0;

force_inline bool isUnset(int x, int y)
{
    return floodGenMatrix[y][x] != currentFloodGen;
}

force_inline void setDist(int x, int y, uint8_t dist)
{
    floodMatrix[y][x] = dist;
    floodGenMatrix[y][x] = currentFloodGen;
}

static uint8_t moveMask[MAZE_SIZE][MAZE_SIZE];

void analogWrite(uint gpio, uint level)
{
    uint slice = pwm_gpio_to_slice_num(gpio);
    pwm_set_chan_level(slice, pwm_gpio_to_channel(gpio), level);
}

void c1_callback(uint gpio, uint32_t events)
{
    if (events & GPIO_IRQ_EDGE_RISE)
    {
        totalTicksL++; // Or -- if in reverse
    }
}

void c2_callback(uint gpio, uint32_t events)
{
    if (events & GPIO_IRQ_EDGE_RISE)
    {
        totalTicksR++; // Or -- if in reverse
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
            std::cout << " ";
            if (mazeMatrix[y][x].topWall)
                std::cout << "---";
            else
                std::cout << "   ";
        }
        std::cout << " \n";

        // Side walls + spaces
        for (int x = 0; x < N; x++)
        {
            if (mazeMatrix[y][x].leftWall)
                std::cout << "|";
            else
                std::cout << " ";

            std::cout << std::setw(3) << (int) floodMatrix[y][x];
        }

        // Right wall of the last cell
        if (mazeMatrix[y][N - 1].rightWall)
            std::cout << "|\n";
        else
            std::cout << " \n";
    }

    // Bottom walls of the last row
    for (int x = 0; x < N; x++)
    {
        std::cout << " ";
        if (mazeMatrix[N - 1][x].bottomWall)
            std::cout << "---";
        else
            std::cout << "   ";
    }
    std::cout << " \n";
}

void setWall(int x, int y, Direction dir)
{
    mazeMatrix[y][x].walls |= (1 << dir);
    moveMask[y][x] &= ~(1 << dir);

    int nx = x + DIRECTIONS[dir].x;
    int ny = y + DIRECTIONS[dir].y;

    if (nx >= 0 && nx < 16 && ny >= 0 && ny < 16)
    {
        mazeMatrix[ny][nx].walls |= (1 << OPPOSITE[dir]);
        moveMask[ny][nx] &= ~(1 << OPPOSITE[dir]);
    }

    API::setWall(x, 15 - y, DIRECTION_CHAR[dir]);
}

// Global States
#define INIT 0
#define IDLE 1
#define MAP_FLOODFILL 2

#define MAP_ASTAR 3
#define MAP_CUSTOM 4

#define MAP_FLOODFILL_BACK 5


class VController
{
public:
    double ePrev = 0;
    double eTot = 0;

    double Kp;
    double Ki;
    double Kd;

    // int tPrev;
    int64_t tPrev;
    // int64_t tPrev;

    VController(double Kp, double Ki, double Kd)
    {
        this->Kp = Kp;
        this->Ki = Ki;
        this->Kd = Kd;

        tPrev = time_us_64();
        // tPrev = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
    }

    double output(double vRef, double vCurrent)
    {
        double e = vRef - vCurrent;
        int t = time_us_64();
        // int64_t t = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();

        double dt = (t - tPrev)/1000000.0;

        // std::cout<<"dt: "<<dt<<"Current time: "<<t<<" Previous time: "<<tPrev<<std::endl;
        // std::cout<<"eTot: "<<eTot<<std::endl; 
        eTot += e * dt;
        
        double output = Kp * e + Ki * eTot + Kd * (e - ePrev)/dt;
        tPrev = t;


        ePrev = e;

        return output;
    }

    void reset() {
        ePrev = 0;
        eTot = 0;
        tPrev = 0;
    }
};


class WController
{
public:
    double ePrev = 0;
    double eTot = 0;

    double Kp;
    double Ki;
    double Kd;

    int64_t tPrev;

    WController(double Kp, double Ki, double Kd)
    {
        this->Kp = Kp;
        this->Ki = Ki;
        this->Kd = Kd;

        tPrev = time_us_64();
        // tPrev = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
    }

    double output(double wRef, double wCurrent)
    {
        double e = wRef - wCurrent;
        int t = time_us_64();
        // int64_t t = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();

        double dt = (t - tPrev) / 1000000;

        eTot += e * dt;
        
        double output = Kp * e + Ki * eTot;

        ePrev = e;

        return output;
    }

    void reset() {
        ePrev = 0;
        eTot = 0;
        tPrev = 0;
    }
};


class Motor
{
    public:
        int PWM = 0;
        int currentRPM = 0;
        
        const volatile uint* totalTicks;
        uint prevTicks;

        int dir = FORWARD; // forward = 1, backward = -1
        int pinForward;
        int pinBackward;
        int pinPWM;
        int pinENC;
        double prevRPM;
        uint64_t tPrev;
        
    Motor(int pinForward, int pinBackward, int pinPWM, int pinENC, const volatile uint *totalTicks, gpio_irq_callback_t callback) {
        tPrev = time_us_64();

        this->pinForward = pinForward;
        this->pinBackward = pinBackward;
        this->pinPWM = pinPWM;
        this->pinENC = pinENC;
        
        this->totalTicks = totalTicks;
        this->prevTicks = prevTicks;

        initEncoder(callback);
    }

    int setPWM(int PWM, int dir) {
        printf("Setting PWM...\n");
        this->dir = dir;
        if (dir == FORWARD) {
            gpio_put(pinForward, 1);
            gpio_put(pinBackward, 0);
        } else if (dir == BACKWARD) {
            gpio_put(pinForward, 0);
            gpio_put(pinBackward, 1);
        }

        if (PWM > 255 || PWM < 0) {
            printf("Invalid PWM\n");
            return -1;
        }

        analogWrite(pinPWM, PWM);
        return 1;
    }


    int initEncoder(gpio_irq_callback_t callback) {
        printf("Starting encoder...\n");
        gpio_init(pinENC);
        gpio_set_dir(pinENC, GPIO_IN);
        gpio_pull_up(pinENC);

        gpio_set_irq_enabled_with_callback(pinENC, GPIO_IRQ_EDGE_RISE, true, callback);
        irq_set_enabled(IO_IRQ_BANK0, true);
        printf("Encoder finished!\n");

        return 0;
    }

    
    float readRPM() {
        uint64_t now = time_us_64();

        double pulses = double(*totalTicks - prevTicks);
        printf("Pulses: %d, ticks: %d, prev ticks: %d ", pulses, *totalTicks, prevTicks);
        prevTicks = *totalTicks;

        float dt_us = (now - tPrev);
        printf("tNow: %lld tPrev: %lld dt: %f\n", now, tPrev, dt_us);

        tPrev = now;

        if (dt_us > 0)
        {
            // float rps = (pulses / TICKS_PER_REV) * (1000000.0f / dt_us) / GEAR_RATIO;
            float rpm = (pulses / TICKS_PER_REV) * (60000000.0f / dt_us) / GEAR_RATIO; // Convert to RPM
            printf("RPM reading is: %f\n", rpm);
            prevRPM = rpm;
            return rpm;
        }
        
        return 0.0;
    }
};


class Mouse
{
public:
    Point cellPos = {0, MAZE_SIZE - 1};
    Direction orientation = Direction::TOP;

    int state = IDLE;
    int solvingType = 0; // 0 = floodfill, 1 = a*, 2 = custom  -  replace with states?

    int phase = 0; // 0: mapping phase   1: pathfinding phase   2: solving phase

    Mouse()
    {
        mazeMatrix[cellPos.y][cellPos.x].visited = true; // Mark starting cell as visited
    }

    void turnLeft()
    {
        API::turnLeft();
        orientation = ROTATE_LEFT[orientation];
    }

    void turnRight()
    {
        API::turnRight();
        orientation = ROTATE_RIGHT[orientation];
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

Queue floodQueue;


force_inline void tryPropagate(int dir, int x, int y, int dist, uint8_t mask) {
    if (!(mask & (1 << dir))) return;

    int nx = x + DIRECTIONS[dir].x;
    int ny = y + DIRECTIONS[dir].y;
    uint8_t ndist = floodMatrix[ny][nx];
    if (isUnset(nx, ny) || ndist > dist + 1) {
        setDist(nx, ny, dist + 1);
        floodQueue.push({nx, ny});
    }
}

force_inline void initBackFill(int dir, int x, int y, uint8_t mask) {
    if (!(mask & (1 << dir))) return;

    int nx = x + DIRECTIONS[dir].x;
    int ny = y + DIRECTIONS[dir].y;

    auto &neighbor = mazeMatrix[ny][nx];
    if (!neighbor.visited)
    {
        setDist(nx, ny, 10); // Set initial distance for unknown neighbors
        floodQueue.push({nx, ny});
    }
}


double readTOF() {
    return 200.0;
}


uint pwm_setup(uint gpio)
{
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(gpio);
    pwm_set_enabled(slice, true);
    pwm_set_wrap(slice, 255); // 8-bit PWM
    return slice;
}

void floodFill(Mouse &mouse)
{
    auto begin = std::chrono::high_resolution_clock::now();

    currentFloodGen++;
    floodQueue.reset();

    if (mouse.state == MAP_FLOODFILL)
    {
        setDist(7, 7, 0); // Cost to start is 0
        setDist(7, 8, 0);
        setDist(8, 7, 0);
        setDist(8, 8, 0);

        // Add goal cells to queue
        floodQueue.push({7, 7});
        floodQueue.push({7, 8});
        floodQueue.push({8, 7});
        floodQueue.push({8, 8});
    }
    else if (mouse.state == MAP_FLOODFILL_BACK)
    {
        setDist(0, MAZE_SIZE - 1, 0); // Start from bottom left corner
        floodQueue.push({0, MAZE_SIZE - 1});

        for (int x = 0; x < MAZE_SIZE; x++)
        {
            for (int y = 0; y < MAZE_SIZE; y++)
            {
                auto &cell = mazeMatrix[y][x];
                if (!cell.visited)
                    continue; // Only consider known cells

                uint8_t mask = moveMask[y][x];
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
        Point current = floodQueue.pop();
        c++;

        int x = current.x, y = current.y;
        int dist = floodMatrix[y][x];

        uint8_t mask = moveMask[y][x];
        tryPropagate(0, x, y, dist, mask);
        tryPropagate(1, x, y, dist, mask);
        tryPropagate(2, x, y, dist, mask);
        tryPropagate(3, x, y, dist, mask);
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
    std::cout << "Flood fill completed in " << duration.count() / 1e6 << " ms, processed " << c << " cells.\n";

    printMaze();
}

////////////////// FUNCTIONS ////////////////////

void scanWalls(Mouse &mouse)
{
    if (API::wallFront())
    {
        setWall(mouse.cellPos.x, mouse.cellPos.y, mouse.orientation);
    }
    if (API::wallLeft())
    {
        setWall(mouse.cellPos.x, mouse.cellPos.y, ROTATE_LEFT[mouse.orientation]);
    }
    if (API::wallRight())
    {
        setWall(mouse.cellPos.x, mouse.cellPos.y, ROTATE_RIGHT[mouse.orientation]);
    }
}


std::string fastestPath(Mouse &mouse)
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
}


void motorTest(Motor &Motor) {
    if (stdio_usb_connected()) {
            printf("Forward...\n");
            Motor.setPWM(255, FORWARD);
            sleep_ms(100);
            int rpm1 = Motor.readRPM();
            sleep_ms(50);
            int rpm2 = Motor.readRPM();
            sleep_ms(500);
            Motor.readRPM();
            sleep_ms(10);
            int rpm3 = Motor.readRPM(); 
            printf("RPM1: %d RPM2: %d RPM3: %d\n", rpm1, rpm2, rpm3);
            sleep_ms(300);
            printf("Stopping...\n");
            Motor.setPWM(0, BACKWARD);
            sleep_ms(1000);

            printf("Backward...\n");
            Motor.setPWM(255, BACKWARD);
            sleep_ms(300);
            
            printf("Stopping...\n");
            Motor.setPWM(0, BACKWARD);
            sleep_ms(1000);
        } else {
            Motor.setPWM(0, BACKWARD);
    }
}


std::pair<int, int> controlLoop(VController &VContr, WController &WContr, Mouse &mouse, Motor &MotorL, Motor &MotorR, float vTarget, float wTarget) {
    float rpmCurrentL = MotorL.readRPM();
    float rpmCurrentR = MotorR.readRPM();
    float vCurrentL = rpmCurrentL*WHEEL_RADIUS/60;
    float vCurrentR = rpmCurrentR*WHEEL_RADIUS/60;
    float vCurrentAVG = (vCurrentL + vCurrentR)/2;
    float wCurrent = (vCurrentR - vCurrentL)/WHEEL_BASE;

    double vOut = VContr.output(vTarget, vCurrentAVG);
    double wOut = WContr.output(wTarget, wCurrent);

    printf("VOut: %f WOut: %f rpmL: %f rpmR: %f vL: %f vR: %f W: %f\n", vOut, wOut, rpmCurrentL, rpmCurrentR, vCurrentL, vCurrentR, wCurrent);

    int dutyL = int(std::max(std::min((vOut - wOut), 220.0), 0.0));
    int dutyR = int(std::max(std::min((vOut + wOut), 220.0), 0.0));

    return {dutyL, dutyR};
}


int main() 
{        
    std::ios::sync_with_stdio(false);
    std::cin.tie(nullptr);

    stdio_init_all();

    while (!stdio_usb_connected())
    {
        sleep_ms(1000);
    }

    // Uncomment this line if you want to run the micromouse code, otherwise the MMS simulator.
    // extern int actual_main();
    // return actual_main();

    printf("Initializing GPIOs...\n");
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);
    gpio_put(25, 1);

    gpio_init(dirA1Pin);
    gpio_set_dir(dirA1Pin, GPIO_OUT);
    gpio_init(dirA2Pin);
    gpio_set_dir(dirA2Pin, GPIO_OUT);
    pwm_setup(spdAPin);

    gpio_init(dirB1Pin);
    gpio_set_dir(dirB1Pin, GPIO_OUT);
    gpio_init(dirB2Pin);
    gpio_set_dir(dirB2Pin, GPIO_OUT);
    pwm_setup(spdBPin);

    printf("GPIOs Initialized!\n");
    printf("Initalizing Classes...\n");

    // Initialize mouse
    Mouse mouse;

    VController VContr(KP_V, KI_V, KD_V);
    WController WContr(KP_W, KI_W, KD_W);

    Motor MotorL(dirA1Pin, dirA2Pin, spdAPin, mrencPin, &totalTicksL, &c2_callback);
    Motor MotorR(dirB1Pin, dirB2Pin, spdBPin, mlencPin, &totalTicksR, &c1_callback);

    printf("Classes initialized!\n");

    double vtarg = 5;
    double vout1 = VContr.output(vtarg, 0);
    double vout2 = VContr.output(vtarg, 2);
    double vout3 = VContr.output(vtarg, 3);

    double wout1 = WContr.output(vtarg, 0);
    double wout2 = WContr.output(vtarg, 2);
    double wout3 = WContr.output(vtarg, 3);

    while (true) {
        if (stdio_usb_connected()) {
            auto [pwmL, pwmR] = controlLoop(VContr, WContr, mouse, MotorL, MotorR, 2, 0);
            MotorL.setPWM(pwmL, FORWARD);
            MotorR.setPWM(pwmR, FORWARD);
            printf("Duty Left: %d Duty Right: %d\n", pwmL, pwmR);
            
        } else {
            MotorL.setPWM(0, FORWARD);
            MotorR.setPWM(0, FORWARD);
        }
    }

    
    // printf("VOut1: %f VOut2: %f VOut3: %f\nWOut1: %f WOut2: %f WOut3: %f", vout1, vout2, vout3, wout1, wout2, wout3);



    // while (true) {
    //     // motorTest(MotorL);
    //     // motorTest(MotorR);
    //     if(stdio_usb_connected()) {
    //         // motorTest(MotorR);
    //         // motorTest(MotorL);
    //         MotorL.setPWM(255, FORWARD);
    //         MotorR.setPWM(255, FORWARD);
    //         sleep_ms(100);
    //         printf("RPM: %f, %f", MotorL.readRPM(), MotorR.readRPM());
    //     } else {
    //         MotorR.setPWM(0, FORWARD);
    //         MotorL.setPWM(0, FORWARD);
    //     }
        
        
        // double dutyL, dutyR = controlLoop(VContr, WContr, mouse, MotorL, MotorR, 1000, 0);
        // double rpmL = MotorL.readRPM();
        // double rpmR = MotorR.readRPM();
        // printf("Duty L: %f Duty R: %f\n", dutyL, dutyR);
        // printf("RPM Left: %f RPM Right: %f\n", rpmL, rpmR);
        // MotorL.setPWM(dutyL, FORWARD);
        // MotorR.setPWM(dutyR, FORWARD);
    

    // gpio_put(dirA1Pin, 1);
    // gpio_put(dirA2Pin, 0);
    // analogWrite(spdAPin, 200);

    // double out = VContr.output(1000, 0);
    
    

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
            
            int x = mouse.cellPos.x;
            int y = mouse.cellPos.y;

            uint8_t mask = moveMask[y][x];
            for (int i = 0; i < 4; ++i) {
                if (!(mask & (1 << i))) continue;

                int nx = x + DIRECTIONS[i].x;
                int ny = y + DIRECTIONS[i].y;

                if (floodMatrix[ny][nx] <= minFloodFill)
                {
                    minFloodFill = floodMatrix[ny][nx];
                    minFloodFillDirection = (Direction)i;
                }
            }

            // std::cout << "Flood fill direction: " << minFloodFillDirection << std::endl;
            // std::cout << "Flood fill value: " << minFloodFill << std::endl;

            mouse.move(minFloodFillDirection);

            if (floodMatrix[mouse.cellPos.y][mouse.cellPos.x] == 0)
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
        } else if (mouse.state == FAST_RUN) {
            ;
            
        }
    }
}

