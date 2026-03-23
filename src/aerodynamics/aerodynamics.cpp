#include "aerodynamics.hpp"
#include <cmath>
#include <algorithm>

static constexpr double DEG2RAD = M_PI / 180.0;

// 失速モデル：線形域 → 失速後は cos カーブで CL が減少し 90° でゼロへ
// 抗力は失速後に平板的に増加する
static void stall_model(double CL_alpha, double alpha, double alpha_stall_rad,
                        double& CL_out, double& CD_stall_out) {
    const double abs_a = std::fabs(alpha);
    const double sign  = (alpha >= 0.0) ? 1.0 : -1.0;
    const double CL_max = CL_alpha * alpha_stall_rad;

    if (abs_a <= alpha_stall_rad) {
        // 線形域
        CL_out        = CL_alpha * alpha;
        CD_stall_out  = 0.0;
    } else {
        // 失速後：CL_max から cos カーブで減衰（90° でゼロ）
        const double t = (abs_a - alpha_stall_rad) / (M_PI / 2.0 - alpha_stall_rad);
        const double t_clamp = std::clamp(t, 0.0, 1.0);
        CL_out       = sign * CL_max * std::cos(t_clamp * M_PI / 2.0);
        // 失速後の圧力抗力増加（平板モデル：CD ∝ sin²α）
        CD_stall_out = 1.5 * std::sin(alpha) * std::sin(alpha) * t_clamp;
    }
}

AeroForces compute_aero(const AircraftParams& ac, const AeroState& state) {
    const double V     = std::max(state.V, 1.0);
    const double q_dyn = 0.5 * state.rho * V * V;
    const double S_w   = ac.S_w();
    const double S_t   = ac.S_t();
    const double as_rad = ac.alpha_stall * DEG2RAD;

    // Wing angle of attack (true AoA + wing incidence)
    const double alpha_w = state.alpha + ac.i_w * DEG2RAD;

    // Downwash at tail (Prandtl lifting line approximation)
    const double d_eps_d_alpha = 2.0 * ac.CL_alpha_w() / (M_PI * ac.AR_w());
    const double epsilon = d_eps_d_alpha * state.alpha;

    // Tail AoA: geometric + incidence − downwash + pitch-rate contribution
    const double alpha_pitch = state.q * (ac.x_ac_t() - ac.x_cg) / V;
    const double alpha_t_base = state.alpha + ac.i_t * DEG2RAD - epsilon + alpha_pitch;
    const double alpha_t_eff  = alpha_t_base - ac.tau_e * state.delta_e;

    // Wing lift with stall model
    double CL_w, CD_stall_w;
    stall_model(ac.CL_alpha_w(), alpha_w, as_rad, CL_w, CD_stall_w);
    const double L_w = q_dyn * S_w * CL_w;

    // Tail lift（尾翼は通常 stall しないと仮定）
    const double CL_t = ac.CL_alpha_t() * alpha_t_eff;
    const double L_t  = ac.eta_t * q_dyn * S_t * CL_t;

    // Total CL referenced to wing area
    const double CL_total = CL_w + ac.eta_t * (S_t / S_w) * CL_t;

    // Drag: parasite + induced + stall extra
    const double CD_total = ac.CD0
                          + (CL_total * CL_total) / (M_PI * ac.AR_w() * ac.e)
                          + CD_stall_w;
    const double D_total  = q_dyn * S_w * CD_total;

    // Pitching moment about CG
    const double M_w = L_w * (ac.x_cg - ac.x_ac_w());
    const double M_t = L_t * (ac.x_cg - ac.x_ac_t());

    return {L_w + L_t, D_total, M_w + M_t, CL_total, CD_total, alpha_t_eff};
}
