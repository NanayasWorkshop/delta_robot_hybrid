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
 * Core functions implemented in .cpp for proper linking
 */
class Timer {
public:
    Timer();
    
    /**
     * @brief Start/restart the timer
     */
    void start();
    
    /**
     * @brief Get elapsed time in milliseconds
     * @return Elapsed time since start() was called
     */
    double elapsed_ms() const;
    
    /**
     * @brief Get elapsed time in microseconds for high precision
     * @return Elapsed time in microseconds
     */
    inline double elapsed_us() const {
        TimePoint end_time = Clock::now();
        return std::chrono::duration_cast<std::chrono::duration<double, std::micro>>(end_time - start_time_).count();
    }
    
    /**
     * @brief Get elapsed time in nanoseconds for maximum precision
     * @return Elapsed time in nanoseconds
     */
    inline double elapsed_ns() const {
        TimePoint end_time = Clock::now();
        return std::chrono::duration_cast<std::chrono::duration<double, std::nano>>(end_time - start_time_).count();
    }
    
private:
    TimePoint start_time_;
};

/**
 * @brief RAII timer for automatic timing of scopes
 */
class ScopedTimer {
public:
    explicit ScopedTimer(double& result_ms) : result_(result_ms) {
        timer_.start();
    }
    
    ~ScopedTimer() {
        result_ = timer_.elapsed_ms();
    }
    
private:
    Timer timer_;
    double& result_;
};

/**
 * @brief Macro for easy scoped timing
 */
#define SCOPED_TIMER(var) ScopedTimer _scoped_timer(var)

} // namespace utils
} // namespace delta_robot