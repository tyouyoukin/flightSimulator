#pragma once
#include "aircraft/aircraft.hpp"

struct AeroState {
    double alpha;    // angle of attack [rad]
    double delta_e;  // elevator command [rad], positive = nose UP
    double q;        // pitch rate [rad/s]
    double V;        // true airspeed [m/s]
    double rho;      // air density [kg/m³]
};

struct AeroForces {
    double L;     // total lift [N]
    double D;     // total drag [N]
    double M;     // pitching moment about CG [N·m], positive = nose up
    double CL;    // total CL (ref. wing area)
    double CD;    // total CD (ref. wing area)
    double alpha_t; // tail effective AoA [rad]
};

AeroForces compute_aero(const AircraftParams& ac, const AeroState& state);
