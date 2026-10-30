// ventsim - runs a clinical scenario through the closed loop and writes
// waveforms as CSV (stdout) and a per-breath report (stderr).
//
//   ./ventsim > docs/scenario.csv 2> docs/breaths.md
//   python3 tools/plot.py docs/scenario.csv docs/scenario.svg
//
//   0 s   passive patient, R = 10 cmH2O/(L/s), C = 50 mL/cmH2O
//  30 s   bronchospasm: R -> 20
//  50 s   stiffer lung: C -> 25 mL/cmH2O
//  64 s   patient starts breathing at 24/min (5 cmH2O efforts)
//  76 s   patient stops; circuit disconnected at 80 s for 6 s
//  96 s   end
#include <cstdio>
#include <cmath>

#include "ventsim/sim.hpp"

using namespace ventsim;

int main()
{
    Simulation sim(fw::Settings{});

    std::printf("t,setpoint,paw,flow,volume,r_true,c_true,r_est,c_est,alarms\n");
    int decim = 0;
    auto log = [&](const Sample& x) {
        if (++decim < 10) return; // 100 Hz in the CSV
        decim = 0;
        std::printf("%.3f,%.2f,%.2f,%.4f,%.4f,%.1f,%.4f,%.2f,%.4f,%u\n", x.t, x.setpoint, x.paw, x.flow,
                    x.volume, x.r_true, x.c_true, x.r_est, x.c_est, x.alarms);
    };

    sim.run_until(30, log);
    sim.patient.lung.resistance = 20.0;
    sim.run_until(50, log);
    sim.patient.lung.compliance = 0.025;
    sim.run_until(64, log);
    sim.patient.effort.amplitude = 5.0;
    sim.patient.effort.phase = 60.0 / 24.0 - std::fmod(64.0, 60.0 / 24.0);
    sim.run_until(76, log);
    sim.patient.effort.amplitude = 0.0;
    sim.run_until(80, log);
    sim.patient.disconnected = true;
    sim.run_until(86, log);
    sim.patient.disconnected = false;
    sim.run_until(96, log);

    std::fprintf(stderr, "| # | Start | Type | Vti (mL) | Vte (mL) | Ppeak | PEEP |\n");
    std::fprintf(stderr, "|---:|---:|---|---:|---:|---:|---:|\n");
    for (int i = 0; i < sim.vent.breaths(); ++i) {
        const auto& b = sim.vent.breath(i);
        std::fprintf(stderr, "| %d | %.2f s | %s | %.0f | %.0f | %.1f | %.1f |\n", i + 1, b.start,
                     b.triggered ? "triggered" : "mandatory", b.vti * 1000, b.vte * 1000, b.p_peak, b.peep);
    }
    std::fprintf(stderr, "\nAlarm events:\n");
    const char* names[] = { "", "HIGH PRESSURE", "LOW TIDAL VOLUME", "", "DISCONNECT" };
    for (int i = 0; i < sim.vent.alarm_events(); ++i) {
        const auto& e = sim.vent.alarm_event(i);
        std::fprintf(stderr, "  %6.2f s  %-16s %s\n", e.t, names[e.alarm], e.raised ? "raised" : "cleared");
    }
    return 0;
}
