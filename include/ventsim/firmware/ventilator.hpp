// ventilator.hpp - the "firmware": pressure-controlled assist-control
// ventilation running at 1 kHz.
//
// Responsibilities, each a small class:
//   BreathTimer         mandatory / patient-triggered breath cycling
//   pressure loop       ramped set point + flow feed-forward + PI
//   BreathMonitor       Vti, Vte, peak pressure, measured PEEP per breath
//   MechanicsEstimator  online R and C via recursive least squares
//   AlarmMonitor        high pressure, low tidal volume, disconnection
//
// No heap, no exceptions: every container is a fixed-size std::array.
#pragma once

#include <algorithm>
#include <array>
#include <cstdint>

#include "pi_controller.hpp"
#include "rls.hpp"

namespace ventsim::fw {

struct Readings {
    double airway_pressure; // cmH2O
    double flow;            // L/s
};

struct Settings {
    double rate_bpm = 15.0;
    double t_insp = 1.0;          // s
    double p_insp = 15.0;         // cmH2O above PEEP
    double peep = 5.0;            // cmH2O
    double rise_time = 0.10;      // s, set point ramp
    bool trigger_enabled = true;
    double trigger_flow = 0.05;   // L/s (3 L/min)
    double trigger_refractory = 0.5; // s into expiration before a trigger is accepted

    double alarm_p_high = 40.0;   // cmH2O
    double alarm_vt_low = 0.20;   // L
    double circuit_resistance = 2.0; // known from the pre-use self test
    double blower_tau = 0.015;       // s, from blower characterisation
    bool anti_windup = true;
};

enum class Phase : std::uint8_t { Inspiration, Expiration };

struct BreathRecord {
    double start = 0.0;
    bool triggered = false;
    double vti = 0.0, vte = 0.0;
    double p_peak = 0.0, peep = 0.0;
};

enum Alarm : std::uint8_t { kHighPressure = 1, kLowTidalVolume = 2, kDisconnect = 4 };

struct AlarmEvent {
    double t;
    std::uint8_t alarm;
    bool raised;
};

// Requires a condition to hold for on_delay before raising and to be absent
// for off_delay before clearing, so noise cannot make an alarm chatter.
class Debounce {
public:
    Debounce(double on_delay, double off_delay) noexcept : on_(on_delay), off_(off_delay) {}
    bool update(bool cond, double dt) noexcept
    {
        timer_ = (cond != active_) ? timer_ + dt : 0.0;
        if (timer_ >= (active_ ? off_ : on_)) {
            active_ = !active_;
            timer_ = 0.0;
        }
        return active_;
    }
    bool active() const noexcept { return active_; }

private:
    double on_, off_, timer_ = 0.0;
    bool active_ = false;
};

// Passive single-compartment model  Paw = R*Q + V/C + P0, identified at 100 Hz.
//
// Patient effort is not in the model, and an effort during a mandatory breath
// biases the fit. So the estimator works transactionally: it fits during a
// breath but only commits the new state when the breath turns out clean
// (mandatory, and ended by the timer rather than by a patient trigger).
// Otherwise it rolls back to the last committed state.
class MechanicsEstimator {
public:
    void begin_breath(bool usable) noexcept
    {
        volume_ = 0.0;
        usable_ = usable;
    }

    void end_breath(bool clean) noexcept
    {
        if (clean) {
            committed_ = rls_;
            committed_updates_ = updates_;
        } else {
            rls_ = committed_;
            updates_ = committed_updates_;
        }
    }

    void sample(const Readings& r, double dt) noexcept
    {
        volume_ += r.flow * dt;
        if (!usable_ || ++decim_ < 10) return;
        decim_ = 0;
        rls_.update({ r.flow, volume_, 1.0 }, r.airway_pressure);
        ++updates_;
    }

    void pause() noexcept { usable_ = false; }
    // Committed values: what a clinician would be shown.
    double resistance() const noexcept { return committed_.theta()[0]; }
    double compliance() const noexcept
    {
        const double e = committed_.theta()[1];
        return e > 1e-6 ? 1.0 / e : 0.0;
    }
    bool converged() const noexcept { return committed_updates_ > 600; }

private:
    Rls<3> rls_{ 0.995 };      // ~2 s memory at 100 Hz
    Rls<3> committed_{ 0.995 };
    double volume_ = 0.0;
    bool usable_ = false;
    int decim_ = 0;
    long updates_ = 0, committed_updates_ = 0;
};

class Ventilator {
public:
    static constexpr double kDt = 0.001; // 1 kHz control loop

    explicit Ventilator(const Settings& s) noexcept
        : s_(s), pi_({ 1.2, 60.0, 0.0, 60.0, s.anti_windup })
    {
        start_breath(0.0, false);
    }

    // One control period. Returns the blower pressure command.
    double tick(double t, const Readings& r) noexcept
    {
        if (disconnect_.active())
            return standby(t, r);

        const double since = t - breath_start_;
        const double period = 60.0 / s_.rate_bpm;

        if (phase_ == Phase::Inspiration && since >= s_.t_insp) {
            phase_ = Phase::Expiration;
        } else if (phase_ == Phase::Expiration) {
            const bool due = since >= period;
            const bool trig = s_.trigger_enabled && since - s_.t_insp >= s_.trigger_refractory &&
                              r.flow > s_.trigger_flow;
            if (due || trig) {
                finish_breath(due);
                start_breath(t, trig && !due);
            }
        }
        const double since_now = t - breath_start_;

        // Set point: linear ramp to PEEP + Pinsp, then hold; PEEP in expiration.
        const double prev = setpoint_;
        if (phase_ == Phase::Inspiration)
            setpoint_ = s_.peep + s_.p_insp * std::min(1.0, since_now / s_.rise_time);
        else
            setpoint_ = s_.peep;

        // Feed-forward does most of the work so the PI loop only corrects
        // the residual: the pressure drop across the known circuit
        // resistance, plus a rate term that inverts the blower's first-order
        // lag. Without the rate term the loop trails the ramp, the integral
        // charges up and the plateau overshoots by about 1 cmH2O.
        const double rate = (setpoint_ - prev) / kDt;
        const double ff = setpoint_ + s_.blower_tau * rate + r.flow * s_.circuit_resistance;
        const double cmd = pi_.update(setpoint_, r.airway_pressure, ff, kDt);

        monitor(r);
        estimator_.sample(r, kDt);
        alarms(t, r, since_now);
        if (disconnect_.active()) {
            // Just detected: drop to standby instead of chasing the set
            // point with full blower output into an open circuit.
            pi_.reset();
            estimator_.end_breath(false);
            estimator_.pause();
            setpoint_ = s_.peep;
            return s_.peep;
        }
        return cmd;
    }

    Phase phase() const noexcept { return phase_; }
    double setpoint() const noexcept { return setpoint_; }
    const MechanicsEstimator& mechanics() const noexcept { return estimator_; }
    std::uint8_t active_alarms() const noexcept { return alarm_bits_; }

    int breaths() const noexcept { return breaths_; }
    // Most recent completed breath (valid when breaths() > 0).
    const BreathRecord& last_breath() const noexcept { return history_[slot(breaths_ - 1)]; }
    const BreathRecord& breath(int i) const noexcept { return history_[slot(i)]; }
    static constexpr int kHistory = 64;

    int alarm_events() const noexcept { return n_events_; }
    const AlarmEvent& alarm_event(int i) const noexcept { return events_[static_cast<std::size_t>(i)]; }

private:
    static std::size_t slot(int i) noexcept { return static_cast<std::size_t>(i % kHistory); }

    // Circuit open: hold a low bias pressure (PEEP, open loop) and stop
    // cycling. The bias flow tells us when the circuit is closed again; the
    // interrupted breath is discarded and ventilation restarts cleanly.
    double standby(double t, const Readings& r) noexcept
    {
        setpoint_ = s_.peep;
        set_alarm(t, kDisconnect, disconnect_.update(r.flow > 1.0, kDt));
        if (!disconnect_.active()) {
            low_vt_count_ = 0;
            start_breath(t, false);
        }
        return s_.peep;
    }

    void start_breath(double t, bool triggered) noexcept
    {
        breath_start_ = t;
        phase_ = Phase::Inspiration;
        cur_ = BreathRecord{};
        cur_.start = t;
        cur_.triggered = triggered;
        // Muscle effort is not in the passive model: estimate on mandatory breaths only.
        estimator_.begin_breath(!triggered && !disconnect_.active());
    }

    void finish_breath(bool ended_by_timer) noexcept
    {
        estimator_.end_breath(ended_by_timer && !cur_.triggered);
        double sum = 0.0;
        for (double p : peep_window_) sum += p;
        cur_.peep = sum / static_cast<double>(peep_window_.size());
        history_[slot(breaths_)] = cur_;
        ++breaths_;

        low_vt_count_ = (cur_.vti < s_.alarm_vt_low) ? low_vt_count_ + 1 : 0;
        set_alarm(cur_.start, kLowTidalVolume, low_vt_count_ >= 2 && !disconnect_.active());
    }

    void monitor(const Readings& r) noexcept
    {
        if (phase_ == Phase::Inspiration) {
            cur_.vti += std::max(r.flow, 0.0) * kDt;
            cur_.p_peak = std::max(cur_.p_peak, r.airway_pressure);
        } else {
            cur_.vte += std::max(-r.flow, 0.0) * kDt;
            peep_window_[peep_idx_++ % peep_window_.size()] = r.airway_pressure;
        }
    }

    void alarms(double t, const Readings& r, double since) noexcept
    {
        set_alarm(t, kHighPressure, high_p_.update(r.airway_pressure > s_.alarm_p_high, kDt));

        // Large inspiratory flow while pressure cannot follow the set point
        // means the circuit is open. Skip the ramp at the start of a breath.
        // Sustained inspiratory flow during expiration means the same.
        const bool settled = phase_ == Phase::Expiration || since > s_.rise_time + 0.1;
        const bool open = settled && ((r.flow > 0.5 && r.airway_pressure < setpoint_ - 3.0) ||
                                      (phase_ == Phase::Expiration && r.flow > 1.0));
        set_alarm(t, kDisconnect, disconnect_.update(open, kDt));
    }

    void set_alarm(double t, std::uint8_t bit, bool on) noexcept
    {
        const bool was = alarm_bits_ & bit;
        if (on == was) return;
        alarm_bits_ = static_cast<std::uint8_t>(on ? (alarm_bits_ | bit) : (alarm_bits_ & ~bit));
        if (n_events_ < static_cast<int>(events_.size()))
            events_[static_cast<std::size_t>(n_events_++)] = { t, bit, on };
    }

    Settings s_;
    PiController pi_;
    MechanicsEstimator estimator_;

    Phase phase_ = Phase::Inspiration;
    double breath_start_ = 0.0;
    double setpoint_ = 0.0;

    BreathRecord cur_;
    std::array<BreathRecord, kHistory> history_{};
    int breaths_ = 0;
    std::array<double, 100> peep_window_{}; // last 100 ms of expiration
    std::size_t peep_idx_ = 0;

    Debounce high_p_{ 0.02, 0.5 };
    Debounce disconnect_{ 0.3, 1.0 };
    int low_vt_count_ = 0;
    std::uint8_t alarm_bits_ = 0;
    std::array<AlarmEvent, 64> events_{};
    int n_events_ = 0;
};

}  // namespace ventsim::fw
