#pragma once
#include "aircraft/aircraft.hpp"

struct StaticStability {
    double x_np;      // neutral point from nose [m]
    double SM;        // static margin [%MAC], positive = stable
    double V_stall;   // stall speed at sea level [m/s]
    bool   stable;
};

StaticStability compute_static_stability(const AircraftParams& ac, double rho);
double          compute_trim_alpha(const AircraftParams& ac, double rho, double V);
double          compute_trim_throttle(const AircraftParams& ac, double rho, double V, double alpha);
