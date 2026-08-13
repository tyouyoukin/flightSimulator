#pragma once
#include "../aerodynamics/aerodynamics.h"
#include "../aircraft/aircraft.h"

// ---------------------------------------------------------------------------
// 縦の 3 自由度 剛体運動方程式 (機体軸, 対称面内, v = p = r = 0)
//
//   m (u_dot + q w) = -m g sin(theta) + X_A + T
//   m (w_dot - q u) =  m g cos(theta) + Z_A
//   Iyy q_dot       =  M_A
//   theta_dot       =  q
//
//   V     = sqrt(u^2 + w^2)
//   alpha = atan2(w, u)
//   gamma = theta - alpha
//   qbar  = 1/2 rho V^2
//
//   X_A =  L sin(alpha) - D cos(alpha)
//   Z_A = -L cos(alpha) - D sin(alpha)
//   L = qbar S CL,  D = qbar S CD,  M_A = qbar S cbar Cm
//
//   h_dot = u sin(theta) - w cos(theta)     (h : 上向き正の高度)
//   x_dot = u cos(theta) + w sin(theta)
// ---------------------------------------------------------------------------

namespace fs {

struct State {
    double u = 20.0;   // 機体 x 方向速度 [m/s]
    double w = 0.0;    // 機体 z 方向速度 [m/s] (下向き正)
    double q = 0.0;    // ピッチ角速度 [rad/s]
    double theta = 0.0;// ピッチ角 [rad]
    double x = 0.0;    // 水平距離 [m]
    double h = 500.0;  // 高度 [m]
};

struct Controls {
    double delta = 0.0;     // エレボン舵角 [rad]
    double throttle = 0.0;  // 0..1
};

/// 状態から得られる派生量 (表示・判定用)
struct Derived {
    double V = 0.0;        // 対気速度 [m/s]
    double alpha = 0.0;    // 迎角 [rad]
    double gamma = 0.0;    // 経路角 [rad]
    double qbar = 0.0;     // 動圧 [Pa]
    double rho = 0.0;
    double mach = 0.0;
    double nz = 1.0;       // 荷重倍数 (機体 z 軸)
    double alphadot = 0.0;
    double L = 0.0, D = 0.0, M = 0.0;
    AeroCoeffs aero{};
};

Derived derive(const Aircraft& ac, const State& s, const Controls& c);

/// 状態微分  ds/dt
State stateDerivative(const Aircraft& ac, const State& s, const Controls& c);

/// 古典 4 次 Runge-Kutta で 1 ステップ積分
State rk4(const Aircraft& ac, const State& s, const Controls& c, double dt);

/// アクチュエータの舵角レート制限
double slewDelta(const Aircraft& ac, double current, double commanded, double dt);

}  // namespace fs
