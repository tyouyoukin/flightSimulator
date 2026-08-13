#include "stability.h"

#include <algorithm>
#include <cmath>

#include "../aerodynamics/aerodynamics.h"
#include "../atmosphere/atmosphere.h"

namespace fs {

using cplx = std::complex<double>;

// ============================================================== トリム =====

namespace {
// 残差 : f0 = Cm,  f1 = qbar S (CL cosa + CD sina) - W cosa
void trimResidual(const Aircraft& ac, double qbar, double alpha, double delta,
                  double& f0, double& f1) {
    AeroInput in;
    in.alpha = alpha;
    in.delta = delta;
    in.V = 1.0;  // q, alphadot = 0 なので V は係数に影響しない
    const AeroCoeffs c = computeAero(ac, in);

    const double W  = ac.m * atmo::g0;
    const double ca = std::cos(alpha), sa = std::sin(alpha);
    f0 = c.Cm;
    f1 = qbar * ac.S * (c.CL * ca + c.CD * sa) - W * ca;
}
}  // namespace

TrimPoint solveTrim(const Aircraft& ac, double V, double h) {
    TrimPoint t;
    t.V = V;
    t.h = h;

    const AtmoState at = isa(h);
    t.rho  = at.rho;
    t.qbar = dynamicPressure(at.rho, V);

    // 初期推定 : Cm = 0 と CL = W/(qS) を線形式で解く
    const double W = ac.m * atmo::g0;
    const double CLreq = W / std::max(t.qbar * ac.S, 1e-9);
    const double Cma = ac.Cmalpha();
    const double Cmd = ac.Cmdelta();
    const double det = ac.CLalpha * Cmd - ac.CLdelta * Cma;
    double alpha = 0.0, delta = 0.0;
    if (std::fabs(det) > 1e-12) {
        const double rhs1 = CLreq - ac.CL0;   // CLa*a + CLd*d = rhs1
        const double rhs2 = -ac.Cm0();        // Cma*a + Cmd*d = rhs2
        alpha = (rhs1 * Cmd - ac.CLdelta * rhs2) / det;
        delta = (ac.CLalpha * rhs2 - Cma * rhs1) / det;
    }
    alpha = std::clamp(alpha, -0.35, 0.35);
    delta = std::clamp(delta, ac.deltaMin, ac.deltaMax);

    // Newton 法 (数値ヤコビアン)
    const double eps = 1e-7;
    int it = 0;
    double f0 = 0, f1 = 0;
    for (; it < 60; ++it) {
        trimResidual(ac, t.qbar, alpha, delta, f0, f1);
        if (std::fabs(f0) < 1e-12 && std::fabs(f1) < 1e-9) break;

        double a0, a1, b0, b1;
        trimResidual(ac, t.qbar, alpha + eps, delta, a0, a1);
        trimResidual(ac, t.qbar, alpha, delta + eps, b0, b1);
        const double J00 = (a0 - f0) / eps, J01 = (b0 - f0) / eps;
        const double J10 = (a1 - f1) / eps, J11 = (b1 - f1) / eps;
        const double d = J00 * J11 - J01 * J10;
        if (std::fabs(d) < 1e-16) break;

        double da = (-f0 * J11 + f1 * J01) / d;
        double dd = (-f1 * J00 + f0 * J10) / d;
        // ステップ制限 (失速域での発散防止)
        const double lim = 0.10;
        const double mag = std::max(std::fabs(da), std::fabs(dd));
        if (mag > lim) { da *= lim / mag; dd *= lim / mag; }

        alpha += da;
        delta = std::clamp(delta + dd, ac.deltaMin, ac.deltaMax);
    }

    trimResidual(ac, t.qbar, alpha, delta, f0, f1);
    t.alpha = alpha;
    t.delta = delta;
    t.theta = alpha;  // gamma = 0
    t.iterations = it;
    t.residCm = f0;
    t.residLift = f1;

    AeroInput in;
    in.alpha = alpha;
    in.delta = delta;
    in.V = V;
    const AeroCoeffs c = computeAero(ac, in);
    t.CL = c.CL;
    t.CD = c.CD;

    const double L = t.qbar * ac.S * c.CL;
    const double D = t.qbar * ac.S * c.CD;
    t.thrust   = (W - L) * std::sin(alpha) + D * std::cos(alpha);
    t.throttle = t.thrust / ac.Tmax;

    t.ok = std::fabs(f0) < 1e-6 && std::fabs(f1) < 1e-3 &&
           t.throttle >= 0.0 && t.throttle <= 1.0 &&
           std::fabs(alpha) < ac.alphaStall;
    return t;
}

void trimToState(const TrimPoint& t, State& s, Controls& c) {
    s.u = t.V * std::cos(t.alpha);
    s.w = t.V * std::sin(t.alpha);
    s.q = 0.0;
    s.theta = t.theta;
    s.h = t.h;
    c.delta = t.delta;
    c.throttle = std::clamp(t.throttle, 0.0, 1.0);
}

// ====================================================== 安定微係数 =========

Derivatives dimensionalDerivatives(const Aircraft& ac, const TrimPoint& t) {
    Derivatives d{};
    const double Q  = t.qbar;
    const double S  = ac.S;
    const double c  = ac.cbar;
    const double m  = ac.m;
    const double I  = ac.Iyy;
    const double u0 = std::max(t.V, 1.0);

    const double CL1 = t.CL;
    const double CD1 = t.CD;
    const double CLu = 0.0;  // 非圧縮性
    const double CDu = 0.0;
    const double CDalpha = 2.0 * ac.k_induced() * CL1 * ac.CLalpha;
    const double CDdelta = 2.0 * ac.kdelta * t.delta;
    const double Cmu = 0.0;

    d.Xu = -(CDu + 2.0 * CD1) * Q * S / (m * u0);
    d.Xw = -(CDalpha - CL1) * Q * S / (m * u0);
    d.Xdelta = -CDdelta * Q * S / m;

    d.Zu = -(CLu + 2.0 * CL1) * Q * S / (m * u0);
    d.Zw = -(ac.CLalpha + CD1) * Q * S / (m * u0);
    d.Zq = -ac.CLq * Q * S * c / (2.0 * m * u0);
    d.Zwdot = -ac.CLadot * Q * S * c / (2.0 * m * u0 * u0);
    d.Zdelta = -ac.CLdelta * Q * S / m;

    d.Mu = Cmu * Q * S * c / (u0 * I);
    d.Mw = ac.Cmalpha() * Q * S * c / (u0 * I);
    d.Mq = ac.Cmq * Q * S * c * c / (2.0 * u0 * I);
    d.Mwdot = ac.Cmadot * Q * S * c * c / (2.0 * u0 * u0 * I);
    d.Mdelta = ac.Cmdelta() * Q * S * c / I;

    d.Zalpha = u0 * d.Zw;
    d.Malpha = u0 * d.Mw;
    d.Malphadot = u0 * d.Mwdot;
    return d;
}

LinearModel buildLinearModel(const Aircraft& ac, const TrimPoint& t) {
    LinearModel lm;
    lm.trim = t;
    lm.der  = dimensionalDerivatives(ac, t);
    const Derivatives& d = lm.der;
    const double u0 = std::max(t.V, 1.0);
    lm.u0 = u0;

    // theta0 = 0 (水平定常飛行) を仮定した安定軸表現
    const double inv = 1.0 / (1.0 - d.Zwdot);
    const double Zu_ = d.Zu * inv;
    const double Zw_ = d.Zw * inv;
    const double Zq_ = (u0 + d.Zq) * inv;
    const double Zd_ = d.Zdelta * inv;

    lm.A[0][0] = d.Xu;  lm.A[0][1] = d.Xw;  lm.A[0][2] = 0.0;  lm.A[0][3] = -atmo::g0;
    lm.A[1][0] = Zu_;   lm.A[1][1] = Zw_;   lm.A[1][2] = Zq_;  lm.A[1][3] = 0.0;
    lm.A[2][0] = d.Mu + d.Mwdot * Zu_;
    lm.A[2][1] = d.Mw + d.Mwdot * Zw_;
    lm.A[2][2] = d.Mq + d.Mwdot * Zq_;
    lm.A[2][3] = 0.0;
    lm.A[3][0] = 0.0;   lm.A[3][1] = 0.0;   lm.A[3][2] = 1.0;  lm.A[3][3] = 0.0;

    lm.B[0] = d.Xdelta;
    lm.B[1] = Zd_;
    lm.B[2] = d.Mdelta + d.Mwdot * Zd_;
    lm.B[3] = 0.0;

    // 推力は機体 x 軸方向。安定軸では alpha_trim だけ傾いている。
    const double XdT =  ac.Tmax * std::cos(t.alpha) / ac.m;
    const double ZdT = -ac.Tmax * std::sin(t.alpha) / ac.m;
    const double ZdT_ = ZdT * inv;
    lm.Bt[0] = XdT;
    lm.Bt[1] = ZdT_;
    lm.Bt[2] = d.Mwdot * ZdT_;   // 推力線は重心を通るのでピッチングモーメント無し
    lm.Bt[3] = 0.0;
    return lm;
}

// ========================================================== 固有値 ========

void eigen4(const double A[4][4], cplx out[4]) {
    // --- Faddeev-LeVerrier で特性方程式 lambda^4 + c1 l^3 + c2 l^2 + c3 l + c4
    double M[4][4] = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
    double c[5] = {1, 0, 0, 0, 0};

    auto matmul = [](const double X[4][4], const double Y[4][4], double R[4][4]) {
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j) {
                double s = 0;
                for (int k = 0; k < 4; ++k) s += X[i][k] * Y[k][j];
                R[i][j] = s;
            }
    };

    double AM[4][4];
    for (int k = 1; k <= 4; ++k) {
        if (k > 1) {
            // M <- A*M_prev + c[k-1] I
            matmul(A, M, AM);
            for (int i = 0; i < 4; ++i)
                for (int j = 0; j < 4; ++j) M[i][j] = AM[i][j] + (i == j ? c[k - 1] : 0.0);
        }
        matmul(A, M, AM);
        double tr = AM[0][0] + AM[1][1] + AM[2][2] + AM[3][3];
        c[k] = -tr / k;
    }

    // --- Durand-Kerner で 4 次方程式を解く
    auto poly = [&](cplx z) {
        return (((z + c[1]) * z + c[2]) * z + c[3]) * z + c[4];
    };
    cplx r[4] = {cplx(0.4, 0.9), cplx(0.4, 0.9), cplx(0.4, 0.9), cplx(0.4, 0.9)};
    r[1] = r[0] * r[0];
    r[2] = r[1] * r[0];
    r[3] = r[2] * r[0];
    // スケーリング (根の大きさに合わせて初期値を広げる)
    double scale = 1.0;
    for (int i = 1; i <= 4; ++i) scale = std::max(scale, std::pow(std::fabs(c[i]), 1.0 / i));
    for (int i = 0; i < 4; ++i) r[i] *= scale;

    for (int iter = 0; iter < 500; ++iter) {
        double maxd = 0.0;
        for (int i = 0; i < 4; ++i) {
            cplx den(1.0, 0.0);
            for (int j = 0; j < 4; ++j)
                if (j != i) den *= (r[i] - r[j]);
            if (std::abs(den) < 1e-300) continue;
            cplx dz = poly(r[i]) / den;
            r[i] -= dz;
            maxd = std::max(maxd, std::abs(dz));
        }
        if (maxd < 1e-13) break;
    }

    // 共役ペアをきれいにする (数値誤差の除去)
    for (int i = 0; i < 4; ++i)
        if (std::fabs(r[i].imag()) < 1e-9 * std::max(1.0, std::fabs(r[i].real())))
            r[i] = cplx(r[i].real(), 0.0);

    for (int i = 0; i < 4; ++i) out[i] = r[i];
}

namespace {
Mode makeMode(cplx lam) {
    Mode m;
    m.lambda = lam;
    const double re = lam.real(), im = std::fabs(lam.imag());
    m.oscillatory = im > 1e-8;
    m.wn = std::abs(lam);
    m.zeta = (m.wn > 1e-12) ? -re / m.wn : 0.0;
    m.period = m.oscillatory ? 2.0 * M_PI / im : 0.0;
    m.tHalf = (std::fabs(re) > 1e-12) ? std::log(2.0) / (-re) : 0.0;
    m.stable = re < 0.0;
    return m;
}
}  // namespace

ModeSet analyzeModes(const Aircraft& ac, const LinearModel& lm) {
    ModeSet ms;
    eigen4(lm.A, ms.eig);

    // |lambda| でソート : 大きい 2 個 = 短周期, 小さい 2 個 = フゴイド
    cplx e[4] = {ms.eig[0], ms.eig[1], ms.eig[2], ms.eig[3]};
    std::sort(e, e + 4, [](const cplx& a, const cplx& b) { return std::abs(a) > std::abs(b); });
    for (int i = 0; i < 4; ++i) ms.eig[i] = e[i];

    // 各ペアから虚部が正 (または実部が大きい) 方を代表に選ぶ
    auto pick = [](cplx a, cplx b) {
        if (std::fabs(a.imag()) > 1e-9 || std::fabs(b.imag()) > 1e-9)
            return (a.imag() >= b.imag()) ? a : b;
        return (a.real() >= b.real()) ? a : b;  // 実根ペアは不安定側を表示
    };
    ms.shortPeriod = makeMode(pick(e[0], e[1]));
    ms.phugoid     = makeMode(pick(e[2], e[3]));

    // ---- 教科書の近似式 ----------------------------------------------------
    const Derivatives& d = lm.der;
    const double u0 = lm.u0;
    const double sp2 = d.Zw * d.Mq - u0 * d.Mw;
    if (sp2 > 0) {
        ms.wnSpApprox = std::sqrt(sp2);
        ms.zetaSpApprox = -(d.Mq + d.Zw + u0 * d.Mwdot) / (2.0 * ms.wnSpApprox);
    }
    ms.wnPhApprox = std::sqrt(2.0) * atmo::g0 / u0;
    if (lm.trim.CL > 1e-6)
        ms.zetaPhApprox = lm.trim.CD / (std::sqrt(2.0) * lm.trim.CL);

    ms.staticallyStable = ac.Cmalpha() < 0.0;
    return ms;
}

// ============================================= 線形モデルでの飛行 =========

namespace {
struct Lin4 { double v[4]; };

Lin4 linDeriv(const LinearModel& lm, const Lin4& x, double ddelta, double dthr) {
    Lin4 r{};
    for (int i = 0; i < 4; ++i) {
        double s = lm.B[i] * ddelta + lm.Bt[i] * dthr;
        for (int j = 0; j < 4; ++j) s += lm.A[i][j] * x.v[j];
        r.v[i] = s;
    }
    return r;
}
}  // namespace

LinState linStep(const LinearModel& lm, const LinState& x0, double delta, double throttle,
                 double dt) {
    const double dd = delta - lm.trim.delta;
    const double dT = throttle - lm.trim.throttle;
    Lin4 x{{x0.du, x0.dw, x0.dq, x0.dtheta}};

    auto add = [](const Lin4& a, const Lin4& k, double f) {
        Lin4 r;
        for (int i = 0; i < 4; ++i) r.v[i] = a.v[i] + f * k.v[i];
        return r;
    };

    const Lin4 k1 = linDeriv(lm, x, dd, dT);
    const Lin4 k2 = linDeriv(lm, add(x, k1, dt * 0.5), dd, dT);
    const Lin4 k3 = linDeriv(lm, add(x, k2, dt * 0.5), dd, dT);
    const Lin4 k4 = linDeriv(lm, add(x, k3, dt), dd, dT);

    LinState out = x0;
    const double f = dt / 6.0;
    double nv[4];
    for (int i = 0; i < 4; ++i)
        nv[i] = x.v[i] + f * (k1.v[i] + 2 * k2.v[i] + 2 * k3.v[i] + k4.v[i]);
    out.du = nv[0];
    out.dw = nv[1];
    out.dq = nv[2];
    out.dtheta = nv[3];

    // 線形化した hdot = u0*dtheta - dw   (theta0 = 0)
    const double hdot0 = lm.u0 * x0.dtheta - x0.dw;
    const double hdot1 = lm.u0 * out.dtheta - out.dw;
    out.dh = x0.dh + 0.5 * (hdot0 + hdot1) * dt;
    out.dx = x0.dx + (lm.u0 + 0.5 * (x0.du + out.du)) * dt;
    return out;
}

State linToState(const LinearModel& lm, const LinState& x) {
    // 線形モデルは安定軸 (x 軸をトリム速度ベクトルに一致させた軸) で書かれている。
    // 表示・判定を非線形モデルと共通化するため機体軸へ alpha_trim だけ回して戻す。
    const double at = lm.trim.alpha;
    const double us = lm.u0 + x.du;
    const double ws = x.dw;

    State s;
    s.u = us * std::cos(at) - ws * std::sin(at);
    s.w = us * std::sin(at) + ws * std::cos(at);
    s.q = x.dq;
    s.theta = at + x.dtheta;  // theta0 = alpha_trim (gamma = 0)
    s.h = lm.trim.h + x.dh;
    s.x = x.dx;
    return s;
}

}  // namespace fs
