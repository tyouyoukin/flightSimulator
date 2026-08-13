#include "aerodynamics.h"

#include <algorithm>
#include <cmath>

namespace fs {

double stallBlend(const Aircraft& ac, double alpha) {
    const double M  = ac.stallSharp;
    const double a0 = ac.alphaStall;
    // Anderson の指数ブレンド関数
    const double e1 = std::exp(std::clamp(-M * (alpha - a0), -60.0, 60.0));
    const double e2 = std::exp(std::clamp( M * (alpha + a0), -60.0, 60.0));
    const double num = 1.0 + e1 + e2;
    const double den = (1.0 + e1) * (1.0 + e2);
    return num / den;
}

static double CLflatPlate(double alpha) {
    // 平板の揚力 : 2 sin^2(a) cos(a) * sign(a)
    const double s = std::sin(alpha);
    return 2.0 * s * std::fabs(s) * std::cos(alpha);
}

AeroCoeffs computeAero(const Aircraft& ac, const AeroInput& in) {
    AeroCoeffs out{};

    const double V = std::max(in.V, 1.0);           // 低速での 0 割り防止
    const double qhat     = in.q * ac.cbar / (2.0 * V);
    const double alphahat = in.alphadot * ac.cbar / (2.0 * V);

    // ---- 揚力 --------------------------------------------------------------
    const double CLlin = ac.CL0 + ac.CLalpha * in.alpha + ac.CLq * qhat +
                         ac.CLadot * alphahat + ac.CLdelta * in.delta;
    const double sigma = stallBlend(ac, in.alpha);
    const double CL    = (1.0 - sigma) * CLlin + sigma * CLflatPlate(in.alpha);

    out.CL_linear = CLlin;
    out.CL        = CL;
    out.stalled   = (sigma > 0.35);

    // ---- 抗力 --------------------------------------------------------------
    const double k = ac.k_induced();
    out.CD = ac.CD0 + k * CL * CL + ac.kdelta * in.delta * in.delta
             + sigma * 1.20 * std::sin(in.alpha) * std::sin(in.alpha);  // 失速時の圧力抗力

    // ---- ピッチングモーメント (重心まわり) -------------------------------------
    // 無尾翼機なので中立点は翼の空力中心。h - h_n が負 (重心が前) のとき復元。
    out.Cm = ac.Cm_ac + ac.Cmdelta_ac * in.delta
             + (ac.h - ac.h_n) * CL
             + ac.Cmq * qhat + ac.Cmadot * alphahat;

    return out;
}

double CmSteady(const Aircraft& ac, double alpha, double delta) {
    AeroInput in;
    in.alpha = alpha;
    in.delta = delta;
    in.q = 0.0;
    in.alphadot = 0.0;
    return computeAero(ac, in).Cm;
}

double CLSteady(const Aircraft& ac, double alpha, double delta) {
    AeroInput in;
    in.alpha = alpha;
    in.delta = delta;
    in.q = 0.0;
    in.alphadot = 0.0;
    return computeAero(ac, in).CL;
}

}  // namespace fs
