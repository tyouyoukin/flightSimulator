#pragma once
#include <complex>

#include "../aircraft/aircraft.h"
#include "../dynamics/dynamics.h"

// ---------------------------------------------------------------------------
// 縦の静安定・動安定
//
// [1] 水平定常飛行のトリム (gamma = 0, theta = alpha)
//        Cm(alpha, delta) = 0
//        L cos(alpha) + D sin(alpha) = W cos(alpha)
//        T = (W - L) sin(alpha) + D cos(alpha)
//
// [2] 次元付き安定微係数 (安定軸, u0 = V_trim, theta0 = 0)
//        Xu = -(CDu + 2 CD1) Q S / (m u0)
//        Xw = -(CDa  -   CL1) Q S / (m u0)
//        Zu = -(CLu + 2 CL1) Q S / (m u0)
//        Zw = -(CLa  +   CD1) Q S / (m u0)
//        Zq = -CLq   Q S cbar / (2 m u0)
//        Zwdot = -CLadot Q S cbar / (2 m u0^2)
//        Mu = Cmu    Q S cbar / (u0 Iyy)
//        Mw = Cma    Q S cbar / (u0 Iyy)
//        Mq = Cmq    Q S cbar^2 / (2 u0 Iyy)
//        Mwdot = Cmadot Q S cbar^2 / (2 u0^2 Iyy)
//
// [3] 小擾乱線形化 : xdot = A x + B delta + Bt dThrottle,  x = [du, dw, q, dtheta]^T
//
//        | Xu               Xw               0                -g |
//    A = | Zu'              Zw'              u0+Zq'            0 |
//        | Mu+Mwd Zu'       Mw+Mwd Zw'       Mq+Mwd(u0+Zq')    0 |
//        | 0                0                1                 0 |
//
//    ( ' は 1/(1 - Zwdot) 倍。theta0 = 0 なので g sin(theta0) 項は消える )
//
// [4] 固有値 -> 短周期モード / フゴイドモード
// ---------------------------------------------------------------------------

namespace fs {

// ---------------------------------------------------------------- トリム ----
struct TrimPoint {
    bool ok = false;
    double V = 0, h = 0;
    double alpha = 0;     // [rad]
    double delta = 0;     // [rad]
    double theta = 0;     // = alpha (gamma = 0)
    double thrust = 0;    // [N]
    double throttle = 0;  // 0..1
    double CL = 0, CD = 0;
    double qbar = 0, rho = 0;
    double residCm = 0, residLift = 0;
    int iterations = 0;
};

/// 高度 h, 対気速度 V での水平定常飛行トリムを Newton 法で解く
TrimPoint solveTrim(const Aircraft& ac, double V, double h);

/// トリム点を State / Controls に変換
void trimToState(const TrimPoint& t, State& s, Controls& c);

// -------------------------------------------------------- 安定微係数 -------
struct Derivatives {
    double Xu = 0, Xw = 0, Xdelta = 0;
    double Zu = 0, Zw = 0, Zq = 0, Zwdot = 0, Zdelta = 0;
    double Mu = 0, Mw = 0, Mq = 0, Mwdot = 0, Mdelta = 0;
    // 便利な alpha 基準の値
    double Zalpha = 0, Malpha = 0, Malphadot = 0;
};

Derivatives dimensionalDerivatives(const Aircraft& ac, const TrimPoint& t);

// ------------------------------------------------------------ 線形モデル ----
struct LinearModel {
    double A[4][4]{};
    double B[4]{};    // エレボン舵角 delta の入力ベクトル
    double Bt[4]{};   // スロットル (0..1) の入力ベクトル  X_dT = Tmax cos(a0)/m など
    double u0 = 0;
    TrimPoint trim{};
    Derivatives der{};
};

LinearModel buildLinearModel(const Aircraft& ac, const TrimPoint& t);

// --------------------------------------------------------------- モード ----
struct Mode {
    std::complex<double> lambda{};
    bool oscillatory = false;
    double wn = 0;      // 固有角振動数 [rad/s]
    double zeta = 0;    // 減衰比
    double period = 0;  // 周期 [s] (振動モードのみ)
    double tHalf = 0;   // 半減時間 [s] (負なら倍増時間)
    bool stable = false;
};

struct ModeSet {
    std::complex<double> eig[4]{};
    Mode shortPeriod{};
    Mode phugoid{};
    // 教科書の近似式 (検証・表示用)
    double wnSpApprox = 0, zetaSpApprox = 0;
    double wnPhApprox = 0, zetaPhApprox = 0;
    bool staticallyStable = false;  // Cm_alpha < 0
};

ModeSet analyzeModes(const Aircraft& ac, const LinearModel& lm);

/// 実係数 4 次行列の固有値 (Faddeev-LeVerrier + Durand-Kerner)
void eigen4(const double A[4][4], std::complex<double> out[4]);

// -------------------------------------------------- 線形モデルでの飛行 ----
struct LinState {
    double du = 0, dw = 0, dq = 0, dtheta = 0;
    double dh = 0, dx = 0;
};

/// xdot = A x + B (delta - delta_trim) + Bt (throttle - throttle_trim) を RK4 で 1 ステップ
LinState linStep(const LinearModel& lm, const LinState& x, double delta, double throttle,
                 double dt);

/// 線形状態を完全な State に復元 (表示・判定を共通化するため)
State linToState(const LinearModel& lm, const LinState& x);

}  // namespace fs
