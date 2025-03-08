#include <stdio.h>
#include <list>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

///////////////// CLASSES ///////////////////////
class Node
{
    public:
        Node(const std::list<int>& pos, double g, double h) {
            const std::list<int> pos = pos;
            double g = g;
            double h = h;
        }
};


////////////////// FUNCTIONS ////////////////////
void floodFill()
{
    ;
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
    stdio_init_all();

    // Initialise the Wi-Fi chip
    if (cyw43_arch_init()) {
        printf("Wi-Fi init failed\n");
        return -1;
    }

    // Example to turn on the Pico W LED
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

    while (true) {
        printf("Hello, world!\n");
        sleep_ms(1000);
    }
}

