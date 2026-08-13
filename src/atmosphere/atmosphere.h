#pragma once
// ---------------------------------------------------------------------------
// 国際標準大気 (ISA) : 0 - 11 km (対流圏)
//   T  = T0 - L*h
//   p  = p0 * (T/T0)^(g/(L*R))
//   rho = p / (R*T)
// ---------------------------------------------------------------------------

namespace fs {

struct AtmoState {
    double rho;  // 密度 [kg/m^3]
    double p;    // 静圧 [Pa]
    double T;    // 温度 [K]
    double a;    // 音速 [m/s]
};

namespace atmo {
constexpr double g0 = 9.80665;    // 重力加速度 [m/s^2]
constexpr double R  = 287.05287;  // 空気の気体定数 [J/(kg K)]
constexpr double L  = 0.0065;     // 気温減率 [K/m]
constexpr double T0 = 288.15;     // 海面温度 [K]
constexpr double p0 = 101325.0;   // 海面気圧 [Pa]
constexpr double rho0 = 1.225;    // 海面密度 [kg/m^3]
constexpr double gamma = 1.4;     // 比熱比
}  // namespace atmo

// 高度 h [m] における ISA 大気状態
AtmoState isa(double h);

// 動圧 qbar = 1/2 rho V^2
inline double dynamicPressure(double rho, double V) { return 0.5 * rho * V * V; }

}  // namespace fs
