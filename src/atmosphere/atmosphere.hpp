#pragma once
#include <cmath>
#include <algorithm>

namespace Atmosphere {
    constexpr double R   = 287.05;  // specific gas constant [J/kg·K]
    constexpr double g   = 9.81;    // gravitational acceleration [m/s²]
    constexpr double T0  = 288.15;  // sea-level temperature [K]
    constexpr double P0  = 101325.0;// sea-level pressure [Pa]
    constexpr double L   = 0.0065;  // temperature lapse rate [K/m]
    constexpr double h_tp = 11000.0;// tropopause altitude [m]

    struct State {
        double rho;  // density [kg/m³]
        double T;    // temperature [K]
        double P;    // pressure [Pa]
        double a;    // speed of sound [m/s]
    };

    inline State at(double h) {
        h = std::max(h, 0.0);
        double T, P, rho;
        if (h <= h_tp) {
            T   = T0 - L * h;
            P   = P0 * std::pow(T / T0, g / (L * R));
        } else {
            // Stratosphere (isothermal)
            double T_tp = T0 - L * h_tp;
            double P_tp = P0 * std::pow(T_tp / T0, g / (L * R));
            T = T_tp;
            P = P_tp * std::exp(-g * (h - h_tp) / (R * T_tp));
        }
        rho = P / (R * T);
        double a = std::sqrt(1.4 * R * T);
        return {rho, T, P, a};
    }
} // namespace Atmosphere
