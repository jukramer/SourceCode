#include "API.h"

#include <stdlib.h>
#include <chrono>
#include <thread>

uint64_t time_us_64() {
    return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

bool stdio_usb_connected() {
    return true;
}

void sleep_ms(uint32_t ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

Motor::Motor(Motor_Choice choice)
{
    this->choice = choice;
}

void Motor::setPWM(int PWM)
{
    printf(">>> setPWM %s %d\n", choice == Motor_Choice::LEFT ? "LEFT" : "RIGHT", PWM);
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

namespace API {
    void init() {
    }
}