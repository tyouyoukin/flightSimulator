#pragma once
#include "aircraft/aircraft.hpp"
#include "aerodynamics/aerodynamics.hpp"
#include "atmosphere/atmosphere.hpp"

// Longitudinal state (2D, earth frame with z = altitude positive up)
struct FlightState {
    double x     = 0.0;     // horizontal position [m]
    double h     = 500.0;   // altitude [m]
    double V     = 60.0;    // true airspeed [m/s]
    double gamma = 0.0;     // flight-path angle [rad], positive up
    double q     = 0.0;     // pitch rate [rad/s], positive nose up
    double theta = 0.0;     // pitch angle [rad], positive nose up

    double alpha() const { return theta - gamma; }
};

struct ControlInputs {
    double delta_e  = 0.0;  // elevator [rad], positive = nose up
    double throttle = 0.5;  // 0–1
};

FlightState compute_derivatives(const AircraftParams& ac,
                                 const FlightState& s,
                                 const ControlInputs& ctrl);

FlightState rk4_step(const AircraftParams& ac,
                      const FlightState& s,
                      const ControlInputs& ctrl,
                      double dt);
