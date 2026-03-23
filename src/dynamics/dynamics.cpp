#include "dynamics.hpp"
#include <cmath>
#include <algorithm>

static constexpr double g = 9.81;

FlightState compute_derivatives(const AircraftParams& ac,
                                 const FlightState& s,
                                 const ControlInputs& ctrl) {
    auto atm = Atmosphere::at(s.h);

    AeroState aero_in;
    aero_in.alpha   = s.alpha();
    aero_in.delta_e = ctrl.delta_e;
    aero_in.q       = s.q;
    aero_in.V       = std::max(s.V, 1.0);
    aero_in.rho     = atm.rho;

    auto forces = compute_aero(ac, aero_in);

    const double T       = ac.T_max * ctrl.throttle;
    const double i_T_rad = ac.i_T * (M_PI / 180.0);
    const double V_eff   = std::max(s.V, 1.0);
    const double W       = ac.mass * g;
    const double alpha   = s.alpha();

    FlightState ds;
    ds.x     = s.V * std::cos(s.gamma);
    ds.h     = s.V * std::sin(s.gamma);
    ds.V     = (T * std::cos(alpha + i_T_rad) - forces.D) / ac.mass
               - g * std::sin(s.gamma);
    ds.gamma = (T * std::sin(alpha + i_T_rad) + forces.L - W * std::cos(s.gamma))
               / (ac.mass * V_eff);
    ds.q     = forces.M / ac.I_yy();
    ds.theta = s.q;

    return ds;
}

FlightState rk4_step(const AircraftParams& ac,
                      const FlightState& s,
                      const ControlInputs& ctrl,
                      double dt) {
    auto blend = [](const FlightState& base, const FlightState& d, double scale) {
        FlightState r = base;
        r.x     += d.x     * scale;
        r.h     += d.h     * scale;
        r.V     += d.V     * scale;
        r.gamma += d.gamma * scale;
        r.q     += d.q     * scale;
        r.theta += d.theta * scale;
        return r;
    };

    auto k1 = compute_derivatives(ac, s,                  ctrl);
    auto k2 = compute_derivatives(ac, blend(s, k1, dt/2), ctrl);
    auto k3 = compute_derivatives(ac, blend(s, k2, dt/2), ctrl);
    auto k4 = compute_derivatives(ac, blend(s, k3, dt),   ctrl);

    FlightState r = s;
    r.x     += (dt / 6.0) * (k1.x     + 2*k2.x     + 2*k3.x     + k4.x);
    r.h     += (dt / 6.0) * (k1.h     + 2*k2.h     + 2*k3.h     + k4.h);
    r.V     += (dt / 6.0) * (k1.V     + 2*k2.V     + 2*k3.V     + k4.V);
    r.gamma += (dt / 6.0) * (k1.gamma + 2*k2.gamma + 2*k3.gamma + k4.gamma);
    r.q     += (dt / 6.0) * (k1.q     + 2*k2.q     + 2*k3.q     + k4.q);
    r.theta += (dt / 6.0) * (k1.theta + 2*k2.theta + 2*k3.theta + k4.theta);

    // Ground
    if (r.h < 0.0) {
        r.h     = 0.0;
        r.V     = std::max(r.V, 0.0);
        r.gamma = 0.0;
        r.q     = 0.0;
    }
    return r;
}
