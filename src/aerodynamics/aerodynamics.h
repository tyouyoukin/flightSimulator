#pragma once
#include "../aircraft/aircraft.h"

// ---------------------------------------------------------------------------
// 縦の空力係数モデル
//
//   qhat      = q * cbar / (2 V)          無次元ピッチ角速度
//   alphahat  = alpha_dot * cbar / (2 V)  無次元 alpha 変化率
//
//   CL = CL0 + CLa*alpha + CLq*qhat + CLadot*alphahat + CLd*delta   (失速ブレンド有)
//   CD = CD0 + k CL^2 + kdelta*delta^2
//   Cm = Cm_ac + Cmd_ac*delta + (h - h_n)*CL + Cmq*qhat + Cmadot*alphahat
// ---------------------------------------------------------------------------

namespace fs {

struct AeroInput {
    double alpha = 0.0;      // 迎角 [rad]
    double V     = 20.0;     // 対気速度 [m/s]
    double q     = 0.0;      // ピッチ角速度 [rad/s]
    double alphadot = 0.0;   // [rad/s]
    double delta = 0.0;      // エレボン舵角 [rad] (正 = 後縁下げ)
};

struct AeroCoeffs {
    double CL = 0.0;
    double CD = 0.0;
    double Cm = 0.0;       // 重心まわり
    double CL_linear = 0.0;  // 失速ブレンド前の線形 CL (表示用)
    bool   stalled = false;
};

/// 失速ブレンド関数 sigma(alpha) : 0 (線形域) -> 1 (平板域)
double stallBlend(const Aircraft& ac, double alpha);

/// 空力係数を計算
AeroCoeffs computeAero(const Aircraft& ac, const AeroInput& in);

/// 定常 (q = alphadot = 0) の Cm。Cm-alpha 線図やトリム求解に使う
double CmSteady(const Aircraft& ac, double alpha, double delta);

/// 定常の CL
double CLSteady(const Aircraft& ac, double alpha, double delta);

}  // namespace fs
