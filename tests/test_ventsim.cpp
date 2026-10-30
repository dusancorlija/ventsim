#include <cmath>
#include <cstdio>
#include <random>

#include "ventsim/sim.hpp"

static int failures = 0, checks = 0;
#define CHECK(cond)                                                                 \
    do {                                                                            \
        ++checks;                                                                   \
        if (!(cond)) {                                                              \
            ++failures;                                                             \
            std::printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);           \
        }                                                                           \
    } while (0)
#define CHECK_NEAR(a, b, tol)                                                       \
    do {                                                                            \
        ++checks;                                                                   \
        const double va = (a), vb = (b);                                            \
        if (!(std::fabs(va - vb) <= (tol))) {                                       \
            ++failures;                                                             \
            std::printf("  FAIL %s:%d  |%s - %s| <= %g  (%g vs %g)\n", __FILE__,    \
                        __LINE__, #a, #b, (double)(tol), va, vb);                   \
        }                                                                           \
    } while (0)
#define RUN(fn)                                                                     \
    do {                                                                            \
        const int before = failures;                                                \
        fn();                                                                       \
        std::printf("%s %s\n", failures == before ? "ok  " : "FAIL", #fn);         \
    } while (0)

using namespace ventsim;

// ── plant ──────────────────────────────────────────────────────────────────

static void lung_charges_like_an_rc_circuit()
{
    plant::Patient p;
    p.blower.pressure = 15.0;
    const double R = p.lung.resistance + p.blower.internal_resistance, C = p.lung.compliance;
    double t = 0.0;
    const double dt = 1e-4;
    for (double probe : { 0.25, 0.5, 1.0, 2.0 }) {
        while (t < probe - 1e-12) { p.step(t, dt); t += dt; }
        CHECK_NEAR(p.lung.volume, C * 15.0 * (1.0 - std::exp(-probe / (R * C))), 0.002);
    }
}

// ── firmware building blocks ───────────────────────────────────────────────

static void rls_identifies_a_known_system()
{
    fw::Rls<3> rls(0.999);
    std::mt19937 rng(3);
    std::normal_distribution<double> n(0.0, 1.0);
    for (int i = 0; i < 4000; ++i) {
        const double a = n(rng), b = n(rng);
        rls.update({ a, b, 1.0 }, 2.0 * a - 3.0 * b + 0.5 + 0.01 * n(rng));
    }
    CHECK_NEAR(rls.theta()[0], 2.0, 0.01);
    CHECK_NEAR(rls.theta()[1], -3.0, 0.01);
    CHECK_NEAR(rls.theta()[2], 0.5, 0.01);
}

// The actuator is blocked for 2 s (output has no effect, like an open
// circuit), then works again. Returns the peak overshoot above the set point.
static double overshoot_after_blocked_actuator(bool anti_windup)
{
    fw::PiController pi({ 1.0, 20.0, 0.0, 30.0, anti_windup });
    double y = 0.0, peak = 0.0;
    const double dt = 1e-3, tau = 0.05, sp = 20.0;
    for (int i = 0; i < 6000; ++i) {
        const double u = pi.update(sp, y, 0.0, dt);
        if (i >= 2000) y += (u - y) / tau * dt;
        peak = std::max(peak, y - sp);
    }
    return peak;
}

static void anti_windup_prevents_overshoot()
{
    const double with = overshoot_after_blocked_actuator(true);
    const double without = overshoot_after_blocked_actuator(false);
    std::printf("    overshoot after 2 s blocked actuator: %.2f with anti-windup, %.2f without\n", with, without);
    CHECK(with < 0.25 * without);
    CHECK(without > 5.0);
}

static void debounce_rejects_glitches()
{
    fw::Debounce d(0.1, 0.5);
    for (int i = 0; i < 50; ++i) d.update(true, 1e-3);   // 50 ms glitch
    for (int i = 0; i < 10; ++i) d.update(false, 1e-3);
    CHECK(!d.active());
    for (int i = 0; i < 101; ++i) d.update(true, 1e-3);
    CHECK(d.active());
    for (int i = 0; i < 400; ++i) d.update(false, 1e-3);
    CHECK(d.active());                                    // still within off delay
    for (int i = 0; i < 101; ++i) d.update(false, 1e-3);
    CHECK(!d.active());
}

// ── closed loop ────────────────────────────────────────────────────────────

static void pressure_tracks_the_set_point()
{
    Simulation sim(fw::Settings{});
    double max_over = 0.0, plateau_err = 0.0;
    int n = 0;
    sim.run_until(20.0, [&](const Sample& x) {
        if (x.t < 4.0) return;                           // first breath fills the lung to PEEP
        const double since = std::fmod(x.t, 4.0);
        const double p = sim.patient.airway_pressure;     // true pressure, no sensor noise
        if (since > 0.0 && since < 1.0) max_over = std::max(max_over, p - 20.0);
        if (since > 0.25 && since < 0.95) { plateau_err += std::fabs(p - 20.0); ++n; }
    });
    std::printf("    overshoot %.2f cmH2O, mean plateau error %.3f cmH2O\n", max_over, plateau_err / n);
    CHECK(max_over < 0.5);
    CHECK(plateau_err / n < 0.1);
}

static void tidal_volume_matches_theory()
{
    fw::Settings s;
    Simulation sim(s);
    sim.run_until(16.0);
    const auto& b = sim.vent.last_breath();
    // Passive RC response to a 15 cmH2O step, delayed by half the ramp.
    const double tau = sim.patient.lung.resistance * sim.patient.lung.compliance;
    const double expected = s.p_insp * sim.patient.lung.compliance * (1.0 - std::exp(-(s.t_insp - s.rise_time / 2) / tau));
    CHECK_NEAR(b.vti, expected, 0.03 * expected);
    CHECK_NEAR(b.vte, b.vti, 0.02 * b.vti);
    CHECK_NEAR(b.peep, s.peep, 0.1);
}

static void breath_timing()
{
    Simulation sim(fw::Settings{});
    sim.run_until(60.5);
    CHECK(sim.vent.breaths() == 15);
    for (int i = 1; i < sim.vent.breaths(); ++i)
        CHECK_NEAR(sim.vent.breath(i).start - sim.vent.breath(i - 1).start, 4.0, 0.002);
}

static void mechanics_estimate_converges()
{
    Simulation sim(fw::Settings{});
    sim.patient.lung.resistance = 15.0;
    sim.patient.lung.compliance = 0.035;
    sim.run_until(30.0);
    CHECK(sim.vent.mechanics().converged());
    CHECK_NEAR(sim.vent.mechanics().resistance(), 15.0, 0.75);
    CHECK_NEAR(sim.vent.mechanics().compliance(), 0.035, 0.035 * 0.05);
}

static void patient_effort_does_not_corrupt_estimate()
{
    Simulation sim(fw::Settings{});
    sim.run_until(30.0);
    sim.patient.effort.amplitude = 6.0;
    double worst = 0.0;
    sim.run_until(60.0, [&](const Sample&) {
        worst = std::max(worst, std::fabs(sim.vent.mechanics().resistance() - 10.0));
    });
    CHECK(worst < 0.5);
}

static void trigger_follows_the_patient()
{
    Simulation sim(fw::Settings{});
    sim.patient.effort.amplitude = 5.0;   // 24/min > set rate of 15/min
    sim.run_until(30.0);
    int triggered = 0;
    for (int i = 0; i < sim.vent.breaths(); ++i) triggered += sim.vent.breath(i).triggered;
    CHECK(sim.vent.breaths() >= 11);       // ~12 in 30 s at 24/min
    CHECK(triggered >= sim.vent.breaths() - 2);
}

static void disconnect_alarm_and_standby()
{
    Simulation sim(fw::Settings{});
    sim.run_until(10.0);
    sim.patient.disconnected = true;
    double raised_at = -1.0;
    sim.run_until(14.0, [&](const Sample& x) {
        if (raised_at < 0 && (x.alarms & fw::kDisconnect)) raised_at = x.t;
    });
    CHECK(raised_at > 0 && raised_at - 10.0 < 0.8);
    CHECK(sim.patient.blower.pressure < 5.5);   // standby bias, not full blower output

    sim.patient.disconnected = false;
    sim.run_until(24.0);
    CHECK(!(sim.vent.active_alarms() & fw::kDisconnect));
    CHECK_NEAR(sim.vent.last_breath().vti, 0.636, 0.03);
}

static void low_tidal_volume_alarm()
{
    Simulation sim(fw::Settings{});
    sim.run_until(8.0);
    CHECK(!(sim.vent.active_alarms() & fw::kLowTidalVolume));
    sim.patient.lung.compliance = 0.010;    // Vt ~ 140 mL
    sim.run_until(20.0);
    CHECK(sim.vent.active_alarms() & fw::kLowTidalVolume);
}

static void high_pressure_alarm()
{
    fw::Settings s;
    s.alarm_p_high = 18.0;                   // below the 20 cmH2O plateau
    Simulation sim(s);
    sim.run_until(2.0);
    bool raised = false;
    for (int i = 0; i < sim.vent.alarm_events(); ++i)
        raised |= sim.vent.alarm_event(i).alarm == fw::kHighPressure && sim.vent.alarm_event(i).raised;
    CHECK(raised);
}

int main()
{
    RUN(lung_charges_like_an_rc_circuit);
    RUN(rls_identifies_a_known_system);
    RUN(anti_windup_prevents_overshoot);
    RUN(debounce_rejects_glitches);
    RUN(pressure_tracks_the_set_point);
    RUN(tidal_volume_matches_theory);
    RUN(breath_timing);
    RUN(mechanics_estimate_converges);
    RUN(patient_effort_does_not_corrupt_estimate);
    RUN(trigger_follows_the_patient);
    RUN(disconnect_alarm_and_standby);
    RUN(low_tidal_volume_alarm);
    RUN(high_pressure_alarm);
    std::printf("\n%d checks, %d failed\n", checks, failures);
    return failures ? 1 : 0;
}
