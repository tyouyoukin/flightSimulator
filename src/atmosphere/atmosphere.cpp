#include "atmosphere.h"

#include <algorithm>
#include <cmath>

namespace fs {

AtmoState isa(double h) {
    using namespace atmo;
    h = std::clamp(h, -1000.0, 20000.0);

    AtmoState s{};
    if (h <= 11000.0) {
        s.T = T0 - L * h;
        s.p = p0 * std::pow(s.T / T0, g0 / (L * R));
    } else {
        // 成層圏下部: 等温層 (216.65 K)
        const double T11 = T0 - L * 11000.0;
        const double p11 = p0 * std::pow(T11 / T0, g0 / (L * R));
        s.T = T11;
        s.p = p11 * std::exp(-g0 * (h - 11000.0) / (R * T11));
    }
    s.rho = s.p / (R * s.T);
    s.a   = std::sqrt(gamma * R * s.T);
    return s;
}

}  // namespace fs
