// Translation unit for the cross-compile check in CI: only the firmware
// headers, built with -fno-exceptions -fno-rtti for a Cortex-M4F.
#include "ventsim/firmware/ventilator.hpp"

namespace {
ventsim::fw::Ventilator vent{ ventsim::fw::Settings{} };
}

extern "C" double control_tick(double t, double paw, double flow)
{
    return vent.tick(t, { paw, flow });
}

extern "C" unsigned ventilator_ram_bytes() { return sizeof(ventsim::fw::Ventilator); }
