#pragma once

#include <string>

namespace gimbal {

bool installSignalStopHandlers(std::string& error) noexcept;
bool signalStopRequested() noexcept;
void clearSignalStopForTest() noexcept;

} // namespace gimbal
