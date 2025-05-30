#pragma once

#include <chrono>

namespace delta_robot {
namespace utils {

using Clock = std::chrono::high_resolution_clock;
using TimePoint = std::chrono::time_point<Clock>;
using Duration = std::chrono::duration<double>;

/**
 * @brief Performance timing statistics for delta robot calculations
 */
struct TimingStats {
    double verify_and_correct_ms;
    double calculate_top_positions_ms;
    double calculate_fermat_ms;
    double optimization_ms;
    double total_ms;
    
    TimingStats() : verify_and_correct_ms(0.0), 
                   calculate_top_positions_ms(0.0),
                   calculate_fermat_ms(0.0),
                   optimization_ms(0.0),
                   total_ms(0.0) {}
};

/**
 * @brief Simple timer class for measuring performance
 */
class Timer {
public:
    Timer();
    void start();
    double elapsed_ms() const;
    
private:
    TimePoint start_time_;
};

} // namespace utils
} // namespace delta_robot
