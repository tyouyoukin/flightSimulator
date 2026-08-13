#pragma once

// ---------------------------------------------------------------------------
// 台形翼 (単純テーパー翼) の平面形から
//   ・平均空力翼弦 MAC
//   ・空力中心  x_ac = MAC 前縁から 25%
//   ・質量中心  x_cg = 翼構造の質量中心 + バラストの合成
//   ・ピッチ慣性モーメント Iyy
//   ・縦の空力微係数 (DATCOM 系の推算式 + 帯状片積分)
// を求める。
//
//  記号 (すべて翼頂点 apex = 翼根前縁 を原点, x は後方正)
//     b      翼幅            = sqrt(AR * S)
//     c_r    翼根弦長        = 2S / (b (1 + lambda))
//     c(eta) = c_r (1 - (1 - lambda) eta),   eta = 2y/b  (0..1)
//     cbar   = (2/S) INT c^2 dy = (2/3) c_r (1 + l + l^2)/(1 + l)
//     y_MAC  = (b/6)(1 + 2 l)/(1 + l)
//     x_LEmac = y_MAC tan(Lambda_LE)
//     x_ac   = x_LEmac + 0.25 cbar
//
//  質量中心 : 翼を面密度一定の板として
//     x_cg,wing = INT (x_LE(y) + c/2) c dy / INT c dy
//   (台形翼では厳密に x_LEmac + cbar/2, すなわち MAC の 50% になる。
//    つまりバラスト無しの無尾翼機は Kn = 0.25 - 0.50 = -0.25 で必ず不安定。)
//
//  静安定余裕
//     h   = (x_cg - x_LEmac) / cbar
//     h_n = 0.25
//     Kn  = h_n - h
// ---------------------------------------------------------------------------

namespace fs {

struct Aircraft;  // aircraft.h

/// プレイヤーが設定する設計変数
struct WingDesign {
    double S        = 60.0;   // 主翼面積 [m^2]   (固定 : 翼面荷重を一定に保つ)
    double AR       = 5.0;    // アスペクト比
    double taper    = 0.50;   // テーパー比 lambda = c_tip / c_root
    double sweepLE  = 25.0;   // 前縁後退角 [deg]
    double washout  = -4.0;   // 翼端ねじり下げ [deg] (負 = washout)

    double airfoilCm  = 0.010;  // 翼型の Cm_ac (リフレックス翼型で正)
    double airfoilA0L = -1.0;   // 翼型のゼロ揚力角 [deg]

    double mass         = 3000.0;  // 全備質量 [kg]
    double wingMassFrac = 0.62;    // 翼構造が占める質量割合 (残りはバラスト/搭載物)
    double ballastXFrac = 0.12;    // バラスト位置 / 翼根弦長 (頂点から)

    double flapChordRatio = 0.20;  // エレボン弦長比 c_f / c
    double flapEtaInner   = 0.40;  // エレボン内端の eta (外端は翼端)
};

/// 平面形から求まる幾何・質量特性
struct WingGeom {
    double b = 0, semiSpan = 0;
    double cRoot = 0, cTip = 0;
    double cbar = 0;        // 平均空力翼弦 MAC [m]
    double yMAC = 0;        // MAC のスパン位置 [m]
    double xLEmac = 0;      // MAC 前縁の x [m] (apex 基準)
    double xAC = 0;         // 空力中心 = xLEmac + 0.25 cbar [m]
    double xCGwing = 0;     // 翼構造だけの質量中心 [m]
    double xBallast = 0;    // バラスト位置 [m]
    double xCG = 0;         // 全機の質量中心 [m]
    double Iyy = 0;         // ピッチ慣性モーメント [kg m^2]
    double kY = 0;          // 慣性半径 [m]
    double sweepC4 = 0;     // 1/4 弦後退角 [rad]
    double sweepC2 = 0;     // 1/2 弦後退角 [rad]
    double h = 0;           // x_cg / cbar (MAC 前縁基準)
    double hn = 0.25;       // 中立点 = 空力中心
    double Kn = 0;          // 静安定余裕 = hn - h
    double wingMass = 0, ballastMass = 0;
};

WingGeom computeGeom(const WingDesign& d);

/// 設計変数から機体データ (幾何 + 空力微係数) を生成
Aircraft buildAircraft(const WingDesign& d);

}  // namespace fs
