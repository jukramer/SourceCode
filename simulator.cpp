#include <iostream>
#include "API.h"

int main() {
    // Disable stdout and stderr buffering for real-time I/O
    setvbuf(stdout, nullptr, _IONBF, 0);
    setvbuf(stderr, nullptr, _IONBF, 0);

    // Handshake: read maze size
    int W = API::mazeWidth();
    int H = API::mazeHeight();

    // Simple wall-following loop
    while (true) {
        if (!API::wallFront()) {
            API::moveForward();
        } else {
            API::turnRight();
        }
    }

    return 0;
}