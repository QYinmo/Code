#include "signal_stop.hpp"

#include <csignal>
#include <iostream>
#include <string>

int main() {
    gimbal::clearSignalStopForTest();
    std::string error;
    if (!gimbal::installSignalStopHandlers(error)) {
        std::cerr << error << '\n';
        return 1;
    }
    if (std::raise(SIGTERM) != 0 || !gimbal::signalStopRequested()) {
        std::cerr << "SIGTERM 未通过 sig_atomic_t 标志传递\n";
        return 1;
    }
    return 0;
}
