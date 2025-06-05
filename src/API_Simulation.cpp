#include "API.h"

#include <stdlib.h>
#include <chrono>
#include <thread>

uint64_t time_us_64()
{
    return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

bool stdio_usb_connected()
{
    return true;
}

void sleep_ms(uint32_t ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

Motor::Motor(Motor_Choice choice)
{
    this->choice = choice;
}

void Motor::setPWM(float PWM)
{
    printf(">>> setPWM %s %f\n", choice == Motor_Choice::LEFT ? "LEFT" : "RIGHT", PWM);
    fflush(stdout);
}

float Motor::readRPM()
{
    printf(">>> readRPM %s\n", choice == Motor_Choice::LEFT ? "LEFT" : "RIGHT");
    fflush(stdout);

    float rpm = 0.0f;
    scanf("%f", &rpm);

    return rpm;
}

float Motor::readPOS() {
    printf(">>> readPOS %s\n", choice == Motor_Choice::LEFT ? "LEFT" : "RIGHT");
    fflush(stdout);

    float dist = 0.0f;
    scanf("%f", &dist);

    if (prevDist == 0.0f) {
        prevDist = dist;
    }

    float result = dist - prevDist;
    prevDist = dist;

    return result;
}

void global_init()
{
}

void global_read_tofs()
{
    for (int direction = 0; direction < NUM_TOF_SENSORS; direction++)
    {
        printf(">>> readTOF %d\n", direction);
        fflush(stdout);

        char valid_char;
        scanf("%d %c", &MM[direction], &valid_char);

        MM_VALID[direction] = (valid_char == 't' || valid_char == 'T');
    }
}

void global_read_imu()
{
    printf(">>> readIMU\n");
    fflush(stdout);

    float ax, ay, gyro_z;
    scanf("%f %f %f", &ax, &ay, &gyro_z);

    AX = ax;
    AY = ay;
    GYRO_Z = gyro_z;
}