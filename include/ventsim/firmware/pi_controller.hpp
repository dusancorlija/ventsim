// pi_controller.hpp - PI controller with feed-forward and conditional
// integration (anti-windup).
#pragma once

#include <algorithm>

namespace ventsim::fw {

struct PiConfig {
    double kp = 0.0;
    double ki = 0.0;           // 1/s
    double out_min = 0.0;
    double out_max = 0.0;
    bool anti_windup = true;   // false only to demonstrate the failure mode
};

class PiController {
public:
    explicit PiController(const PiConfig& cfg) noexcept : cfg_(cfg) {}

    // u = ff + kp*e + integral. With anti-windup the integral only moves
    // while the output is unsaturated, or when the error would pull it back
    // out of saturation. Without it, a long saturation (a disconnected
    // patient, say) winds the integral up and the first breath after
    // reconnection overshoots.
    double update(double setpoint, double measured, double feedforward, double dt) noexcept
    {
        const double e = setpoint - measured;
        const double step = cfg_.ki * e * dt;
        const double trial = feedforward + cfg_.kp * e + integral_ + step;
        const bool high = trial > cfg_.out_max, low = trial < cfg_.out_min;
        if (!cfg_.anti_windup || (!high && !low) || (high && e < 0.0) || (low && e > 0.0))
            integral_ += step;
        return std::clamp(feedforward + cfg_.kp * e + integral_, cfg_.out_min, cfg_.out_max);
    }

    void reset() noexcept { integral_ = 0.0; }
    double integral() const noexcept { return integral_; }

private:
    PiConfig cfg_;
    double integral_ = 0.0;
};

}  // namespace ventsim::fw
