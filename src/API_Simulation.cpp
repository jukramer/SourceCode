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

    float dist = 0.0f;
    scanf("%f", &dist);
    
    if (prevDist == 0.0f) {
        prevDist = dist;
    }

    return dist - prevDist;
}


void global_init()
{
}

void global_read_tofs()
{
}

TOF_Reading global_get_tof(TOF_Direction direction)
{
    printf(">>> readTOF %d\n", direction);
    fflush(stdout);

    TOF_Reading reading;
    reading.distance = 0;
    
    char valid_char;
    scanf("%f %c", &reading.distance, &valid_char);

    reading.valid = (valid_char == 't' || valid_char == 'T');

    return reading;
}