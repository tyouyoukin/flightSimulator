#include "stability.hpp"
#include "aerodynamics/aerodynamics.hpp"
#include <cmath>
#include <algorithm>

static constexpr double g = 9.81;

StaticStability compute_static_stability(const AircraftParams& ac, double rho) {
    const double S_w = ac.S_w();
    const double S_t = ac.S_t();
    const double CLa_w = ac.CL_alpha_w();
    const double CLa_t = ac.CL_alpha_t();
    const double d_eps_da = 2.0 * CLa_w / (M_PI * ac.AR_w());

    // Neutral point (stick-fixed)
    const double tail_factor = ac.eta_t * (S_t / S_w) * CLa_t * (1.0 - d_eps_da);
    const double x_np = (CLa_w * ac.x_ac_w() + tail_factor * ac.x_ac_t())
                       / (CLa_w + tail_factor);

    // Static margin [%MAC]
    const double SM = (x_np - ac.x_cg) / ac.c * 100.0;

    // Stall speed (CL_max ≈ 1.2 for flat plate / simple model)
    constexpr double CL_max = 1.2;
    const double W       = ac.mass * g;
    const double V_stall = std::sqrt(2.0 * W / (rho * S_w * CL_max));

    return {x_np, SM, V_stall, SM > 0.0};
}

double compute_trim_alpha(const AircraftParams& ac, double rho, double V) {
    // Solve L = W for delta_e = 0 by Newton iteration
    const double W     = ac.mass * g;
    const double q_dyn = 0.5 * rho * V * V;
    const double CL_req = W / (q_dyn * ac.S_w());

    // Initial guess from wing only
    double alpha = CL_req / ac.CL_alpha_w();

    for (int i = 0; i < 20; ++i) {
        AeroState as{alpha, 0.0, 0.0, V, rho};
        auto f = compute_aero(ac, as);
        double err = f.L - W;
        // dL/dalpha ≈ CL_alpha * q_dyn * S_w
        double dL = ac.CL_alpha_w() * q_dyn * ac.S_w();
        alpha -= err / dL;
    }
    return alpha;
}

double compute_trim_throttle(const AircraftParams& ac, double rho, double V, double alpha) {
    AeroState as{alpha, 0.0, 0.0, V, rho};
    auto f  = compute_aero(ac, as);
    double T = f.D; // level flight: T = D
    return std::clamp(T / ac.T_max, 0.0, 1.0);
}
