#pragma once
#include <string>

#include "planform.h"

// ---------------------------------------------------------------------------
// 無尾翼機 (flying wing) の諸元と縦の空力微係数
//
//  重心まわりのピッチングモーメント係数は，教科書どおり
//
//      Cm = Cm_ac + Cmd_ac * delta + (h - h_n) * CL
//           + Cmq * qhat + Cm_adot * alphahat_dot
//
//  で表す。h = x_cg / cbar, h_n = x_ac / cbar = 0.25 (無尾翼なので中立点 = 翼の空力中心)。
//  静安定余裕  Kn = h_n - h   →   Cm_alpha = -(Kn) * CL_alpha
//
//  この構造体の値はすべて WingDesign (アスペクト比・テーパー比・後退角 …) から
//  planform.cpp が計算する。手で書いた「マジックナンバー」は翼型の 2 つだけ。
// ---------------------------------------------------------------------------

namespace fs {

struct Aircraft {
    std::string name = "Tailless Wing";

    WingDesign design{};
    WingGeom   geom{};

    // ---- 質量特性 ----------------------------------------------------------
    double m   = 3000.0;  // 質量 [kg]
    double Iyy = 1.0e4;   // ピッチ慣性モーメント [kg m^2]

    // ---- 幾何 --------------------------------------------------------------
    double S    = 60.0;   // 主翼面積 [m^2]
    double b    = 17.3;   // 翼幅 [m]
    double cbar = 5.08;   // 平均空力翼弦 [m]

    // ---- 重心 / 中立点 (MAC 前縁からの比) -------------------------------------
    double h   = 0.17;
    double h_n = 0.25;

    // ---- 揚力 --------------------------------------------------------------
    double CL0     = 0.0;   // alpha = 0 の揚力係数 (翼型キャンバ + 平均ねじり)
    double CLalpha = 4.11;  // 揚力傾斜 [1/rad]  (DATCOM)
    double CLq     = 2.3;   // [1/rad]
    double CLadot  = 0.0;   // [1/rad]  無尾翼は吹き下ろし遅れが無いので 0
    double CLdelta = 1.0;   // [1/rad]  エレボン

    // ---- 抗力 : CD = CD0 + k CL^2 + kdelta delta^2 ---------------------------
    double CD0      = 0.011;
    double e_oswald = 0.90;
    double kdelta   = 0.50;

    // ---- ピッチングモーメント (空力中心まわり) --------------------------------
    double Cm_ac      = 0.03;   // 翼型リフレックス + ねじり下げ × 後退角
    double Cmdelta_ac = -0.46;  // [1/rad]
    double Cmq        = -0.75;  // [1/rad]
    double Cmadot     = -0.22;  // [1/rad]

    // ---- 失速モデル ---------------------------------------------------------
    double alphaStall = 15.0 * 3.14159265358979323846 / 180.0;
    double stallSharp = 40.0;

    // ---- 推進 --------------------------------------------------------------
    double Tmax = 4500.0;  // 最大推力 [N]

    // ---- 舵角制限 -----------------------------------------------------------
    double deltaMin  = -25.0 * 3.14159265358979323846 / 180.0;
    double deltaMax  =  25.0 * 3.14159265358979323846 / 180.0;
    double deltaRate =  70.0 * 3.14159265358979323846 / 180.0;  // [rad/s]

    // ---- 派生量 -------------------------------------------------------------
    double aspectRatio() const { return b * b / S; }
    double k_induced() const {
        return 1.0 / (3.14159265358979323846 * aspectRatio() * e_oswald);
    }

    /// 静安定余裕 Kn = h_n - h   (正で静安定)
    double staticMargin() const { return h_n - h; }

    /// Cm_alpha = -(Kn) CL_alpha   [1/rad]
    double Cmalpha() const { return -staticMargin() * CLalpha; }

    /// 重心まわりの舵効き Cm_delta = Cmdelta_ac + (h - h_n) CL_delta
    double Cmdelta() const { return Cmdelta_ac + (h - h_n) * CLdelta; }

    /// 重心まわりの Cm0 (alpha = 0, delta = 0)
    double Cm0() const { return Cm_ac + (h - h_n) * CL0; }
};

/// 難易度プリセット (バラストを後ろへ動かして静安定余裕を削る)
WingDesign makeDesign(int level);
Aircraft   makeAircraft(int level);
const char* levelName(int level);
int numLevels();

}  // namespace fs
