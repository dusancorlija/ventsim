// plant.hpp - the "physical" side of the simulation: patient lung, blower,
// patient effort and noisy sensors. Nothing here is visible to the firmware
// except through the Sensors readings.
//
// Units: pressure cmH2O, flow L/s, volume L, resistance cmH2O/(L/s),
// compliance L/cmH2O, time s.
#pragma once

#include <algorithm>
#include <cmath>
#include <random>

namespace ventsim::plant {

// Single-compartment lung: alveolar pressure follows the elastic recoil of
// the volume above relaxation volume, minus any inspiratory muscle effort.
struct Lung {
    double resistance = 10.0;   // airway resistance R
    double compliance = 0.05;   // C (50 mL/cmH2O)
    double volume = 0.0;        // above functional residual capacity

    double alveolar_pressure(double p_muscle) const { return volume / compliance - p_muscle; }
};

// Blower modelled as a pressure source with first-order dynamics, a slew
// limit and an internal (outlet + circuit) resistance. The internal
// resistance is what limits flow on a disconnection.
struct Blower {
    double tau = 0.015;          // s
    double slew = 600.0;         // cmH2O/s
    double max_pressure = 60.0;
    double internal_resistance = 2.0;
    double pressure = 0.0;       // generated pressure

    void step(double command, double dt)
    {
        command = std::clamp(command, 0.0, max_pressure);
        double dp = (command - pressure) / tau * dt;
        dp = std::clamp(dp, -slew * dt, slew * dt);
        pressure += dp;
    }
};

// Periodic half-sine inspiratory effort (spontaneously breathing patient).
struct PatientEffort {
    double amplitude = 0.0;      // cmH2O, 0 = passive patient
    double rate_bpm = 24.0;
    double duration = 0.8;       // s of each effort
    double phase = 0.0;

    double at(double t) const
    {
        if (amplitude <= 0.0) return 0.0;
        const double period = 60.0 / rate_bpm;
        const double tau = std::fmod(t + phase, period);
        return tau < duration ? amplitude * std::sin(M_PI * tau / duration) : 0.0;
    }
};

struct Readings {
    double airway_pressure;     // cmH2O, at the Y-piece
    double flow;                // L/s, positive into the patient
};

// Gaussian noise plus a 12-bit-ish quantisation step.
class Sensors {
public:
    explicit Sensors(unsigned seed = 1) : rng_(seed) {}
    double pressure_noise = 0.05;  // cmH2O rms
    double flow_noise = 0.002;     // L/s rms

    Readings read(double p, double q)
    {
        auto quant = [](double v, double lsb) { return std::round(v / lsb) * lsb; };
        return { quant(p + pressure_noise * n_(rng_), 0.02), quant(q + flow_noise * n_(rng_), 0.0005) };
    }

private:
    std::mt19937 rng_;
    std::normal_distribution<double> n_{ 0.0, 1.0 };
};

// Connects blower, circuit and lung. A disconnection replaces the lung with
// a near-zero resistance path to ambient.
struct Patient {
    Lung lung;
    Blower blower;
    PatientEffort effort;
    bool disconnected = false;

    double airway_pressure = 0.0;
    double flow = 0.0;

    void step(double t, double dt)
    {
        const double r_int = blower.internal_resistance;
        if (disconnected) {
            constexpr double r_open = 0.5;
            flow = blower.pressure / (r_int + r_open);
            airway_pressure = flow * r_open;
            // lung passively empties through its own airway
            lung.volume -= lung.volume / (lung.resistance * lung.compliance) * dt;
            return;
        }
        const double p_alv = lung.alveolar_pressure(effort.at(t));
        flow = (blower.pressure - p_alv) / (r_int + lung.resistance);
        airway_pressure = blower.pressure - flow * r_int;
        lung.volume += flow * dt;
    }
};

}  // namespace ventsim::plant
