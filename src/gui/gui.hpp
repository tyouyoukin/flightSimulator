#pragma once
#include "aircraft/aircraft.hpp"
#include "dynamics/dynamics.hpp"
#include "stability/stability.hpp"
#include <vector>

struct Simulator {
    AircraftParams  ac;
    FlightState     state;
    ControlInputs   ctrl;
    StaticStability stability;

    bool   running      = false;
    bool   crashed      = false;
    double sim_time     = 0.0;
    double dt           = 0.01;     // 10 ms simulation step

    static constexpr int HISTORY = 3000;
    std::vector<float> hist_t, hist_h, hist_V, hist_alpha, hist_theta, hist_q;

    void reset();
    void step(double real_dt);      // call each frame; runs multiple physics steps
    void update_stability();
};

// Top-level GUI call — renders one full frame
void render_gui(Simulator& sim);
