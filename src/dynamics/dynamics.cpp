#include "dynamics.h"

#include <algorithm>
#include <cmath>

#include "../atmosphere/atmosphere.h"

namespace fs {

Derived derive(const Aircraft& ac, const State& s, const Controls& c) {
    Derived d{};
    const AtmoState at = isa(s.h);
    d.rho = at.rho;

    d.V     = std::sqrt(s.u * s.u + s.w * s.w);
    d.alpha = std::atan2(s.w, s.u);
    d.gamma = s.theta - d.alpha;
    d.qbar  = dynamicPressure(d.rho, d.V);
    d.mach  = d.V / at.a;

    // --- CLadot = 0 のとき alpha_dot は CL, CD に影響しないので，
    //     まず alpha_dot = 0 で力を求め，そこから alpha_dot を厳密に得る。
    //     CLadot != 0 の場合に備えて 1 回だけ不動点反復する。
    double alphadot = 0.0;
    AeroCoeffs aero{};
    for (int it = 0; it < 2; ++it) {
        AeroInput in;
        in.alpha    = d.alpha;
        in.V        = d.V;
        in.q        = s.q;
        in.alphadot = alphadot;
        in.delta    = c.delta;
        aero = computeAero(ac, in);

        const double L = d.qbar * ac.S * aero.CL;
        const double D = d.qbar * ac.S * aero.CD;
        const double T = ac.Tmax * std::clamp(c.throttle, 0.0, 1.0);

        const double ca = std::cos(d.alpha), sa = std::sin(d.alpha);
        const double X = L * sa - D * ca + T;
        const double Z = -L * ca - D * sa;

        const double udot = -s.q * s.w - atmo::g0 * std::sin(s.theta) + X / ac.m;
        const double wdot =  s.q * s.u + atmo::g0 * std::cos(s.theta) + Z / ac.m;

        const double V2 = std::max(d.V * d.V, 1.0);
        alphadot = (s.u * wdot - s.w * udot) / V2;

        d.L = L;
        d.D = D;
        d.nz = -Z / (ac.m * atmo::g0);
        if (ac.CLadot == 0.0) break;  // 反復不要
    }

    d.alphadot = alphadot;
    d.aero = aero;
    d.M = d.qbar * ac.S * ac.cbar * aero.Cm;
    return d;
}

State stateDerivative(const Aircraft& ac, const State& s, const Controls& c) {
    const Derived d = derive(ac, s, c);

    const double T  = ac.Tmax * std::clamp(c.throttle, 0.0, 1.0);
    const double ca = std::cos(d.alpha), sa = std::sin(d.alpha);
    const double X  = d.L * sa - d.D * ca + T;
    const double Z  = -d.L * ca - d.D * sa;

    State ds{};
    ds.u     = -s.q * s.w - atmo::g0 * std::sin(s.theta) + X / ac.m;
    ds.w     =  s.q * s.u + atmo::g0 * std::cos(s.theta) + Z / ac.m;
    ds.q     =  d.M / ac.Iyy;
    ds.theta =  s.q;
    ds.x     =  s.u * std::cos(s.theta) + s.w * std::sin(s.theta);
    ds.h     =  s.u * std::sin(s.theta) - s.w * std::cos(s.theta);
    return ds;
}

static State axpy(const State& a, const State& d, double f) {
    State r;
    r.u = a.u + f * d.u;
    r.w = a.w + f * d.w;
    r.q = a.q + f * d.q;
    r.theta = a.theta + f * d.theta;
    r.x = a.x + f * d.x;
    r.h = a.h + f * d.h;
    return r;
}

State rk4(const Aircraft& ac, const State& s, const Controls& c, double dt) {
    const State k1 = stateDerivative(ac, s, c);
    const State k2 = stateDerivative(ac, axpy(s, k1, dt * 0.5), c);
    const State k3 = stateDerivative(ac, axpy(s, k2, dt * 0.5), c);
    const State k4 = stateDerivative(ac, axpy(s, k3, dt), c);

    State r = s;
    const double f = dt / 6.0;
    r.u     += f * (k1.u + 2 * k2.u + 2 * k3.u + k4.u);
    r.w     += f * (k1.w + 2 * k2.w + 2 * k3.w + k4.w);
    r.q     += f * (k1.q + 2 * k2.q + 2 * k3.q + k4.q);
    r.theta += f * (k1.theta + 2 * k2.theta + 2 * k3.theta + k4.theta);
    r.x     += f * (k1.x + 2 * k2.x + 2 * k3.x + k4.x);
    r.h     += f * (k1.h + 2 * k2.h + 2 * k3.h + k4.h);
    return r;
}

double slewDelta(const Aircraft& ac, double current, double commanded, double dt) {
    commanded = std::clamp(commanded, ac.deltaMin, ac.deltaMax);
    const double maxStep = ac.deltaRate * dt;
    const double diff = std::clamp(commanded - current, -maxStep, maxStep);
    return std::clamp(current + diff, ac.deltaMin, ac.deltaMax);
}

}  // namespace fs
