#include "API.h"

#include <stdlib.h>
#include <chrono>

uint64_t time_us_64() {
    return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

bool stdio_usb_connected() {
    return true;
}

namespace API {
    bool init() {
        return true;
    }
}