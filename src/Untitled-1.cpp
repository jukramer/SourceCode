#include <stdio.h>
#include <iostream>
#include <iomanip>

#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "hardware/pwm.h"
#include "pico/time.h"

// Pico W devices use a GPIO on the WIFI chip for the LED,
// so when building for Pico W, CYW43_WL_GPIO_LED_PIN will be defined
#ifdef CYW43_WL_GPIO_LED_PIN
#include "pico/cyw43_arch.h"
#endif

// Perform initialisation
int pico_led_init(void)
{
#if defined(PICO_DEFAULT_LED_PIN)
    // A device like Pico that uses a GPIO for the LED will define PICO_DEFAULT_LED_PIN
    // so we can use normal GPIO functionality to turn the led on and off
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    return PICO_OK;
#elif defined(CYW43_WL_GPIO_LED_PIN)
    // For Pico W devices we need to initialise the driver etc
    return cyw43_arch_init();
#endif
}

// Turn the led on or off
void pico_set_led(bool led_on)
{
#if defined(PICO_DEFAULT_LED_PIN)
    // Just set the GPIO on or off
    gpio_put(PICO_DEFAULT_LED_PIN, led_on);
#elif defined(CYW43_WL_GPIO_LED_PIN)
    // Ask the wifi "driver" to set the GPIO on or off
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);
#endif
}

#define ENA 0
#define IN1 2
#define IN2 1

#define ENC_A 4

absolute_time_t start_time;
volatile long encoder_count = 0;

void c1_callback(uint gpio, uint32_t events)
{
    if (events & GPIO_IRQ_EDGE_RISE)
    {
        encoder_count++; // Or -- if in reverse
    }
}

void setup_encoder_single()
{
    gpio_init(ENC_A);
    gpio_set_dir(ENC_A, GPIO_IN);
    gpio_pull_up(ENC_A);

    gpio_set_irq_enabled_with_callback(ENC_A, GPIO_IRQ_EDGE_RISE, true, &c1_callback);
    irq_set_enabled(IO_IRQ_BANK0, true);
}

uint pwm_setup(uint gpio)
{
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(gpio);
    pwm_set_enabled(slice, true);
    pwm_set_wrap(slice, 255); // 8-bit PWM
    return slice;
}

void analogWrite(uint gpio, uint level)
{
    uint slice = pwm_gpio_to_slice_num(gpio);
    pwm_set_chan_level(slice, pwm_gpio_to_channel(gpio), level);
}

void direction_test()
{
    analogWrite(ENA, 255); // full speed

    gpio_put(IN1, 1);
    gpio_put(IN2, 0);
    sleep_ms(2000);

    gpio_put(IN1, 0);
    gpio_put(IN2, 1);
    sleep_ms(2000);

    gpio_put(IN1, 0);
    gpio_put(IN2, 0);
}

volatile long last_encoder_count = 0;
volatile int64_t last_sample_time = 0;

#define TICKS_PER_REV 7.0
#define GEAR_RATIO (1.0 / 30.0)

void sample_rpm(uint duty_cycle)
{
    int64_t now = time_us_64();
    if (last_sample_time == 0)
    {
        last_sample_time = now;
    }

    long count_now = encoder_count;
    long pulses = count_now - last_encoder_count;
    last_encoder_count = count_now;

    float dt_us = now - last_sample_time;
    if (dt_us > 0)
    {
        float rpm = (pulses / TICKS_PER_REV) * (60000000.0f / dt_us) * GEAR_RATIO; // Convert to RPM

        int64_t timestamp = now - start_time;
        printf("%lld,%u,%.2f\n", timestamp, duty_cycle, rpm);
    }
    last_sample_time = now;
}

void dynamic_sweep_test(uint start_pwm, uint end_pwm, int step_pwm, uint stabilization_ms, uint sample_interval_ms)
{
    printf("Starting dynamic sweep test...\n");
    printf("Parameters: Start=%u, End=%u, Step=%d, Stabilize=%ums, Sample=%ums\n",
           start_pwm, end_pwm, step_pwm, stabilization_ms, sample_interval_ms);

    // Set motor direction
    gpio_put(IN1, 0);
    gpio_put(IN2, 1);

    // Determine if we're increasing or decreasing PWM
    int direction = (end_pwm > start_pwm) ? 1 : -1;
    if (step_pwm < 0)
        step_pwm = -step_pwm; // Ensure step is positive

    // Initialize
    uint current_pwm = start_pwm;
    analogWrite(ENA, current_pwm);
    last_encoder_count = encoder_count; // Reset encoder count

    printf("timestamp_ms,pwm,rpm\n"); // CSV header

    while (true)
    {
        // Sample at regular intervals regardless of PWM changes
        absolute_time_t start_time = get_absolute_time();
        absolute_time_t next_sample_time = delayed_by_ms(start_time, sample_interval_ms);

        // Run until we reach the next PWM step or end condition
        while (true)
        {
            // Take sample
            absolute_time_t now = get_absolute_time();
            int64_t elapsed_ms = absolute_time_diff_us(start_time, now) / 1000;
            sample_rpm(current_pwm);

            // Check if we should change PWM
            if (elapsed_ms >= stabilization_ms)
            {
                // Move to next PWM value
                current_pwm += direction * step_pwm;

                // Check if we've reached the end
                if ((direction == 1 && current_pwm > end_pwm) ||
                    (direction == -1 && current_pwm < end_pwm))
                {
                    current_pwm = end_pwm; // Clamp to final value
                }

                analogWrite(ENA, current_pwm);
                last_encoder_count = encoder_count; // Reset count for new PWM
                break;
            }

            // Sleep until next sample time
            sleep_until(next_sample_time);
            next_sample_time = delayed_by_ms(next_sample_time, sample_interval_ms);

            // Early exit if we've reached the end PWM
            if (current_pwm == end_pwm && elapsed_ms >= stabilization_ms)
            {
                gpio_put(IN1, 0);
                gpio_put(IN2, 0);
                printf("Test completed.\n");
                return;
            }
        }
    }
}

void step_response_test()
{
    printf("Starting step response test...\n");

    // Initialization
    analogWrite(ENA, 0);
    gpio_put(IN1, 0);
    gpio_put(IN2, 1);

    // Apply full power
    analogWrite(ENA, 255);

    for (int i = 0; i < 600; i++)
    { // 3 seconds total
        sample_rpm(255);
        sleep_ms(5);
    }

    printf("Full power applied. Now cutting power...\n");

    // Cut power
    analogWrite(ENA, 0);

    last_sample_time = 0;
    sample_rpm(0);

    // Capture decay
    for (int i = 0; i < 600; i++)
    {
        sample_rpm(0);
        sleep_ms(5);
    }

    printf("Step response test completed.\n");
}

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

#define force_inline __attribute__((always_inline)) inline

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

#define IDX(x, y) ((y) * MAZE_SIZE + (x))

static volatile Cell mazeMatrix[MAZE_SIZE][MAZE_SIZE] = {}; // Init to empty with = {};

static volatile uint8_t floodMatrix[MAZE_SIZE * MAZE_SIZE] = {};
static volatile uint8_t floodGenMatrix[MAZE_SIZE * MAZE_SIZE] = {};
static volatile uint8_t currentFloodGen = 0;

force_inline bool isUnset(int x, int y)
{
    return floodGenMatrix[IDX(x, y)] != currentFloodGen;
}

force_inline void setDist(int x, int y, uint8_t dist)
{
    floodMatrix[IDX(x, y)] = dist;
    floodGenMatrix[IDX(x, y)] = currentFloodGen;
}

static uint8_t moveMask[MAZE_SIZE * MAZE_SIZE];

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

            std::cout << std::setw(3) << floodMatrix[IDX(x, y)];
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
    moveMask[IDX(x, y)] &= ~(1 << dir);

    int nx = x + DIRECTIONS[dir].x;
    int ny = y + DIRECTIONS[dir].y;

    if (nx >= 0 && nx < 16 && ny >= 0 && ny < 16)
    {
        mazeMatrix[ny][nx].walls |= (1 << OPPOSITE[dir]);
        moveMask[IDX(nx, ny)] &= ~(1 << OPPOSITE[dir]);
    }
}

// Global States
#define INIT 0
#define IDLE 1
#define MAP_FLOODFILL 2

#define MAP_ASTAR 3
#define MAP_CUSTOM 4

#define MAP_FLOODFILL_BACK 5

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
        // API::turnLeft();
        orientation = ROTATE_LEFT[orientation];
    }

    void turnRight()
    {
        // API::turnRight();
        orientation = ROTATE_RIGHT[orientation];
    }

    void move(Direction targetDir)
    {
        while (targetDir != orientation)
        {
            turnLeft();
        }

        // API::moveForward();
        cellPos += DIRECTIONS[targetDir];
        mazeMatrix[cellPos.y][cellPos.x].visited = true;
    }
};

Queue floodQueue;


force_inline void tryPropagate(int dir, int x, int y, int dist, uint8_t mask) {
    if (!(mask & (1 << dir))) return;

    int nx = x + DIRECTIONS[dir].x;
    int ny = y + DIRECTIONS[dir].y;
    uint8_t ndist = floodMatrix[IDX(nx, ny)];
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


void floodFill(Mouse &mouse)
{
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

                uint8_t mask = moveMask[IDX(x, y)];
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
        int dist = floodMatrix[IDX(x, y)];

        uint8_t mask = moveMask[IDX(x, y)];
        tryPropagate(0, x, y, dist, mask);
        tryPropagate(1, x, y, dist, mask);
        tryPropagate(2, x, y, dist, mask);
        tryPropagate(3, x, y, dist, mask);
    }

    // printMaze();
}

int main()
{
    stdio_init_all();

    while (!stdio_usb_connected())
    {
        sleep_ms(1000);
    }

    int rc = pico_led_init();
    hard_assert(rc == PICO_OK);

    std::ios::sync_with_stdio(false);
    std::cin.tie(nullptr);

    Mouse mouse;
    mouse.state = MAP_FLOODFILL;

    for (int y = 0; y < MAZE_SIZE; y++)
    {
        for (int x = 0; x < MAZE_SIZE; x++)
        {
            uint8_t mask = 0b1111;

            if (y == 0)
                mask &= ~(1 << TOP);
            if (x == MAZE_SIZE - 1)
                mask &= ~(1 << RIGHT);
            if (y == MAZE_SIZE - 1)
                mask &= ~(1 << BOTTOM);
            if (x == 0)
                mask &= ~(1 << LEFT);

            moveMask[IDX(x, y)] = mask;
        }
    }

    // Set 5 walls on 8th row
    for (int i = 0; i < 5; i++)
    {
        setWall(i + 1, 5, Direction::TOP);
    }

    uint32_t dummy = 0;

#define TIMES_TO_TEST 1000

    uint64_t start = time_us_64();
    for (int i = 0; i < TIMES_TO_TEST; i++)
    {
        floodFill(mouse);
        dummy += floodMatrix[IDX(0, i % 16)];
    }
    uint64_t duration = (time_us_64() - start);
    float durationMs = (float)duration / 1000.0f;
    float durationPerTest = durationMs / TIMES_TO_TEST;

    std::cout << ">> Flood fill completed in average " << durationPerTest << " ms, dummy: " << dummy << std::endl;
}

void motor_test()
{
    gpio_init(IN1);
    gpio_set_dir(IN1, GPIO_OUT);
    gpio_init(IN2);
    gpio_set_dir(IN2, GPIO_OUT);

    pwm_setup(ENA); // Configure PWM for ENA pin

    gpio_put(IN1, 0);
    gpio_put(IN2, 0);

    setup_encoder_single();

    pico_set_led(true);

    printf("timestamp_us,pwm,rpm\n"); // CSV header

    start_time = time_us_64();
    step_response_test();

    sleep_ms(5000); // Wait before next run

    // Upward sweep: 0-255 PWM in steps of 5, 1s stabilization, 100ms samples
    dynamic_sweep_test(45, 255, 5, 1000, 100);

    // Downward sweep: 255-0 PWM in steps of 5, 1s stabilization, 100ms samples
    dynamic_sweep_test(255, 45, -5, 1000, 100);

    pico_set_led(false);
    sleep_ms(2000); // Wait before next run
}