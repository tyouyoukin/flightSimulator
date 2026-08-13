#include "planform.h"

#include <algorithm>
#include <cmath>

#include "aircraft.h"

namespace fs {

namespace {
constexpr double PI = 3.14159265358979323846;
constexpr int    NSTRIP = 2000;  // 帯状片積分の分割数
inline double deg2rad(double d) { return d * PI / 180.0; }

// 台形則
template <typename F>
double integrate(double x0, double x1, int n, F f) {
    const double dx = (x1 - x0) / n;
    double s = 0.5 * (f(x0) + f(x1));
    for (int i = 1; i < n; ++i) s += f(x0 + i * dx);
    return s * dx;
}
}  // namespace

// ==========================================================================
WingGeom computeGeom(const WingDesign& d) {
    WingGeom g;
    const double S   = std::max(d.S, 1e-3);
    const double AR  = std::clamp(d.AR, 1.5, 20.0);
    const double lam = std::clamp(d.taper, 0.05, 1.0);

    g.b        = std::sqrt(AR * S);
    g.semiSpan = 0.5 * g.b;
    g.cRoot    = 2.0 * S / (g.b * (1.0 + lam));
    g.cTip     = lam * g.cRoot;

    // MAC = (2/S) INT c^2 dy  ->  台形翼の閉形式
    g.cbar = (2.0 / 3.0) * g.cRoot * (1.0 + lam + lam * lam) / (1.0 + lam);
    g.yMAC = (g.b / 6.0) * (1.0 + 2.0 * lam) / (1.0 + lam);

    const double tanLE = std::tan(deg2rad(d.sweepLE));
    g.xLEmac = g.yMAC * tanLE;
    g.xAC    = g.xLEmac + 0.25 * g.cbar;   // ← 空力中心は MAC 前縁から 25%

    // 各弦位置の後退角  tan(L_n) = tan(L_LE) - (4n/AR)(1-l)/(1+l)
    g.sweepC4 = std::atan(tanLE - (1.0 / AR) * (1.0 - lam) / (1.0 + lam));
    g.sweepC2 = std::atan(tanLE - (2.0 / AR) * (1.0 - lam) / (1.0 + lam));

    // ---- 質量中心 (= 重心) --------------------------------------------------
    const double s = g.semiSpan;
    auto chord = [&](double eta) { return g.cRoot * (1.0 - (1.0 - lam) * eta); };
    auto xLE   = [&](double eta) { return eta * s * tanLE; };
    auto xMid  = [&](double eta) { return xLE(eta) + 0.5 * chord(eta); };  // 帯の面心

    const double area_half = integrate(0.0, 1.0, NSTRIP,
                                       [&](double e) { return chord(e) * s; });
    g.xCGwing = integrate(0.0, 1.0, NSTRIP,
                          [&](double e) { return xMid(e) * chord(e) * s; }) / area_half;

    g.wingMass    = d.mass * std::clamp(d.wingMassFrac, 0.05, 0.99);
    g.ballastMass = d.mass - g.wingMass;
    g.xBallast    = d.ballastXFrac * g.cRoot;
    g.xCG = (g.wingMass * g.xCGwing + g.ballastMass * g.xBallast) / d.mass;

    // ---- ピッチ慣性モーメント ------------------------------------------------
    //   Iyy = INT [ (x_strip - x_cg)^2 + c^2/12 ] dm  +  m_ballast (x_b - x_cg)^2
    const double lamMass = g.wingMass / area_half;  // 面密度
    g.Iyy = integrate(0.0, 1.0, NSTRIP, [&](double e) {
                const double c = chord(e);
                const double dx = xMid(e) - g.xCG;
                return (dx * dx + c * c / 12.0) * c * s * lamMass;
            }) +
            g.ballastMass * (g.xBallast - g.xCG) * (g.xBallast - g.xCG);
    g.kY = std::sqrt(g.Iyy / d.mass);

    // ---- 静安定余裕 ---------------------------------------------------------
    g.hn = 0.25;
    g.h  = (g.xCG - g.xLEmac) / g.cbar;
    g.Kn = g.hn - g.h;
    return g;
}

// ==========================================================================
Aircraft buildAircraft(const WingDesign& d) {
    Aircraft ac;
    ac.design = d;
    ac.geom   = computeGeom(d);
    const WingGeom& g = ac.geom;

    const double AR  = g.b * g.b / d.S;
    const double lam = std::clamp(d.taper, 0.05, 1.0);
    const double s   = g.semiSpan;
    const double tanLE = std::tan(deg2rad(d.sweepLE));

    ac.m    = d.mass;
    ac.Iyy  = g.Iyy;
    ac.S    = d.S;
    ac.b    = g.b;
    ac.cbar = g.cbar;
    ac.h    = g.h;
    ac.h_n  = g.hn;

    auto chord = [&](double eta) { return g.cRoot * (1.0 - (1.0 - lam) * eta); };
    auto xC4   = [&](double eta) { return eta * s * tanLE + 0.25 * chord(eta); };

    // ---- 揚力傾斜 : DATCOM 亜音速有限翼公式 (beta = 1, kappa = 1) --------------
    //   CL_alpha = 2 pi A / ( 2 + sqrt( A^2 (1 + tan^2 L_c/2) + 4 ) )
    const double tC2 = std::tan(g.sweepC2);
    ac.CLalpha = 2.0 * PI * AR / (2.0 + std::sqrt(AR * AR * (1.0 + tC2 * tC2) + 4.0));

    // ---- Oswald 効率 (Raymer の経験式) ---------------------------------------
    const double f = 1.0 - 0.045 * std::pow(AR, 0.68);
    ac.e_oswald = (d.sweepLE <= 30.0)
                      ? 1.78 * f - 0.64
                      : 4.61 * f * std::pow(std::cos(deg2rad(d.sweepLE)), 0.15) - 3.1;
    ac.e_oswald = std::clamp(ac.e_oswald, 0.55, 0.99);

    // ---- 動微係数 : DATCOM 翼単体 --------------------------------------------
    const double cosC4 = std::cos(g.sweepC4);
    const double tanC4 = std::tan(g.sweepC4);
    const double xb    = g.Kn;  // (x_ac - x_cg)/cbar

    ac.CLq = (0.5 + 2.0 * xb) * ac.CLalpha;
    ac.Cmq = -0.9 * ac.CLalpha * cosC4 *
             (AR * (0.5 * xb + 2.0 * xb * xb) / (AR + 2.0 * cosC4) +
              (AR * AR * AR * tanC4 * tanC4) / (24.0 * (AR + 6.0 * cosC4)) + 0.125);
    ac.CLadot = 0.0;         // 無尾翼は吹き下ろし遅れが無い
    ac.Cmadot = 0.30 * ac.Cmq;  // 翼単体の概算

    // ---- ねじり下げ × 後退角 による Cm_ac -------------------------------------
    //   Cm_ac,twist = -(2/(S cbar)) INT a eps(eta) c (x_c4 - x_ac) dy
    //   後退翼 + 翼端ねじり下げ  ->  後方にある翼端の揚力が減る  ->  機首上げ (正)
    const double epsTip = deg2rad(d.washout);
    const double CmTwist = -(2.0 / (d.S * g.cbar)) *
        integrate(0.0, 1.0, NSTRIP, [&](double e) {
            return ac.CLalpha * (epsTip * e) * chord(e) * (xC4(e) - g.xAC) * s;
        });
    ac.Cm_ac = d.airfoilCm + CmTwist;

    // 平均ねじり (揚力加重) と翼型ゼロ揚力角から CL0
    const double epsMean = (2.0 / d.S) *
        integrate(0.0, 1.0, NSTRIP,
                  [&](double e) { return (epsTip * e) * chord(e) * s; });
    ac.CL0 = ac.CLalpha * (-deg2rad(d.airfoilA0L) + epsMean);

    // ---- エレボン (薄翼理論のフラップ + 経験補正 0.85) -------------------------
    //   theta_f = acos(2 c_f/c - 1)
    //   dcl/ddelta   = 2 (pi - theta_f + sin theta_f)     ->  tau = それ / (2 pi)
    //   dcm_c4/ddelta = -(1/2) sin(theta_f)(1 - cos theta_f)
    const double cfc = std::clamp(d.flapChordRatio, 0.05, 0.45);
    const double thf = std::acos(2.0 * cfc - 1.0);
    const double tau = 0.85 * (1.0 - (thf - std::sin(thf)) / PI);
    const double cmd2d = -0.85 * 0.5 * std::sin(thf) * (1.0 - std::cos(thf));
    const double e1 = std::clamp(d.flapEtaInner, 0.0, 0.95);

    ac.CLdelta = (2.0 / d.S) * integrate(e1, 1.0, NSTRIP, [&](double e) {
        return ac.CLalpha * tau * chord(e) * s;
    });
    ac.Cmdelta_ac = (2.0 / (d.S * g.cbar)) * integrate(e1, 1.0, NSTRIP, [&](double e) {
        const double c = chord(e);
        return (cmd2d * c * c - ac.CLalpha * tau * c * (xC4(e) - g.xAC)) * s;
    });

    // ---- 抗力 / 推力 / 失速 ---------------------------------------------------
    ac.CD0 = 0.010 + 0.004 / std::sqrt(std::max(AR, 1.0));  // 摩擦 + 形状抗力の概算
    ac.kdelta = 0.50;
    ac.alphaStall = deg2rad(15.0);
    ac.stallSharp = 40.0;
    ac.Tmax = 1.5 * d.mass;  // 推力重量比 ~ 0.15

    ac.name = "Tailless Wing";
    return ac;
}

}  // namespace fs
