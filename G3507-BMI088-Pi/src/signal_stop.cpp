#include "signal_stop.hpp"

#include <cerrno>
#include <csignal>
#include <cstring>

namespace gimbal {
namespace {

volatile std::sig_atomic_t g_signal_stop = 0;

extern "C" void signalHandler(int) {
    g_signal_stop = 1;
}

} // namespace

bool installSignalStopHandlers(std::string& error) noexcept {
    struct sigaction action {};
    action.sa_handler = signalHandler;
    sigemptyset(&action.sa_mask);
    action.sa_flags = 0;
    if (::sigaction(SIGINT, &action, nullptr) != 0 ||
        ::sigaction(SIGTERM, &action, nullptr) != 0) {
        error = "安装 SIGINT/SIGTERM 处理器失败：" + std::string(std::strerror(errno));
        return false;
    }
    return true;
}

bool signalStopRequested() noexcept { return g_signal_stop != 0; }

void clearSignalStopForTest() noexcept { g_signal_stop = 0; }

} // namespace gimbal
