#pragma once
#include <cmath>

struct AircraftParams {
    // Wing (x positions measured from nose tip)
    double b     = 10.0;   // wingspan [m]
    double c     = 1.5;    // mean chord [m]
    double x_w   = 3.625;  // wing leading edge from nose [m]
    double i_w   = 2.0;    // wing incidence angle [deg]

    // Horizontal tail
    double b_t    = 3.5;   // tail span [m]
    double c_t    = 0.7;   // tail chord [m]
    double x_t    = 8.5;   // tail leading edge from nose [m]
    double i_t    = 0.0;   // tail incidence angle [deg]
    double tau_e  = 0.45;  // elevator effectiveness (0-1)
    double de_max = 25.0;  // max elevator deflection [deg]

    // Fuselage
    double l_f = 10.0;     // total length [m]

    // Mass & CG
    double mass = 500.0;   // [kg]
    double x_cg = 4.5;     // CG from nose [m]

    // Propulsion
    double T_max  = 2500.0; // max thrust [N]
    double i_T    = 0.0;    // thrust angle from body x-axis [deg]

    // Aero params
    double CD0         = 0.025; // zero-lift drag coefficient
    double e           = 0.85;  // Oswald efficiency factor
    double eta_t       = 0.90;  // tail dynamic pressure ratio
    double alpha_stall = 15.0;  // stall angle of attack [deg]

    // ---- Derived quantities ----
    double S_w()  const { return b * c; }
    double S_t()  const { return b_t * c_t; }
    double AR_w() const { return (b * b) / S_w(); }
    double AR_t() const { return (b_t * b_t) / S_t(); }

    // Wing AC (25% chord from LE)
    double x_ac_w() const { return x_w + 0.25 * c; }
    // Tail AC (25% chord from LE)
    double x_ac_t() const { return x_t + 0.25 * c_t; }

    // Helmbold finite-wing lift slope [per rad]
    static double helmbold(double AR) {
        return (2.0 * M_PI * AR) / (2.0 + std::sqrt(4.0 + AR * AR));
    }
    double CL_alpha_w() const { return helmbold(AR_w()); }
    double CL_alpha_t() const { return helmbold(AR_t()); }

    // Simplified moment of inertia [kg·m²]
    double I_yy() const { return mass * l_f * l_f / 12.0; }
};
