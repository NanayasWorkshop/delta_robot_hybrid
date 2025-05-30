#include "utils/timing.hpp"
#include "delta_constants.hpp"

namespace delta_robot {
namespace utils {

Timer::Timer() {
    start();
}

void Timer::start() {
    start_time_ = Clock::now();
}

double Timer::elapsed_ms() const {
    TimePoint end_time = Clock::now();
    return std::chrono::duration_cast<Duration>(end_time - start_time_).count() * constants::MS_CONVERSION_FACTOR;
}

} // namespace utils
} // namespace delta_robot