// sim.hpp - closes the loop between the firmware and the plant.
#pragma once

#include "firmware/ventilator.hpp"
#include "plant/plant.hpp"

namespace ventsim {

struct Sample {
    double t, setpoint, paw, flow, volume;
    double r_true, c_true, r_est, c_est;
    std::uint8_t alarms;
};

class Simulation {
public:
    explicit Simulation(const fw::Settings& s, unsigned seed = 1) : sensors(seed), vent(s) {}

    plant::Patient patient;
    plant::Sensors sensors;
    fw::Ventilator vent;
    double t = 0.0;

    // Advances one 1 ms control period: sample, compute, actuate, integrate.
    Sample step()
    {
        constexpr double dt = fw::Ventilator::kDt;
        const plant::Readings m = sensors.read(patient.airway_pressure, patient.flow);
        const double cmd = vent.tick(t, { m.airway_pressure, m.flow });

        // Plant integrates at 10 kHz inside each control period.
        constexpr int sub = 10;
        for (int i = 0; i < sub; ++i) {
            patient.blower.step(cmd, dt / sub);
            patient.step(t + i * dt / sub, dt / sub);
        }
        t += dt;

        const auto& est = vent.mechanics();
        return { t, vent.setpoint(), m.airway_pressure, m.flow, patient.lung.volume,
                 patient.lung.resistance, patient.lung.compliance,
                 est.resistance(), est.compliance(), vent.active_alarms() };
    }

    template <typename F>
    void run_until(double t_end, F&& on_sample)
    {
        while (t < t_end - 1e-9) on_sample(step());
    }
    void run_until(double t_end)
    {
        run_until(t_end, [](const Sample&) {});
    }
};

}  // namespace ventsim
