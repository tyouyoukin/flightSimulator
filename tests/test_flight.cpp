// 依存ライブラリ無しの検証テスト。  ctest / ./fs_tests で実行。
#include <cmath>
#include <complex>
#include <cstdio>
#include <string>

#include "../src/aerodynamics/aerodynamics.h"
#include "../src/atmosphere/atmosphere.h"
#include "../src/dynamics/dynamics.h"
#include "../src/game/game.h"
#include "../src/stability/stability.h"

using namespace fs;

static int g_fail = 0;
static int g_pass = 0;

static void check(bool ok, const std::string& what, const std::string& detail = "") {
    if (ok) {
        ++g_pass;
        std::printf("  [ OK ] %s\n", what.c_str());
    } else {
        ++g_fail;
        std::printf("  [FAIL] %s   %s\n", what.c_str(), detail.c_str());
    }
}

static void near(double a, double b, double tol, const std::string& what) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "got %.6g, want %.6g (tol %.3g)", a, b, tol);
    check(std::fabs(a - b) <= tol, what, buf);
}

static void relNear(double a, double b, double rel, const std::string& what) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "got %.6g, want %.6g (rel %.1f%%)", a, b, rel * 100);
    check(std::fabs(a - b) <= rel * std::fabs(b) + 1e-12, what, buf);
}

// ---------------------------------------------------------------- 大気 ----
static void testAtmosphere() {
    std::printf("\n[1] 国際標準大気\n");
    AtmoState s0 = isa(0.0);
    near(s0.rho, 1.225, 1e-3, "海面密度 = 1.225 kg/m^3");
    near(s0.T, 288.15, 1e-6, "海面温度 = 288.15 K");
    near(s0.a, 340.29, 0.2, "海面音速 = 340.3 m/s");

    AtmoState s11 = isa(11000.0);
    near(s11.T, 216.65, 0.05, "11 km 温度 = 216.65 K");
    near(s11.p, 22632.0, 20.0, "11 km 気圧 = 22632 Pa");
    check(isa(5000.0).rho < s0.rho, "密度は高度とともに減少");
}

// ------------------------------------------------------------ 平面形 ------
static void testPlanform() {
    std::printf("\n[2] 平面形 -> MAC / 空力中心 / 質量中心\n");
    WingDesign d;
    d.S = 60.0; d.AR = 5.0; d.taper = 0.5; d.sweepLE = 25.0;
    WingGeom g = computeGeom(d);

    near(g.b, std::sqrt(5.0 * 60.0), 1e-9, "b = sqrt(AR S)");
    near(g.cRoot, 2 * 60.0 / (g.b * 1.5), 1e-9, "c_root = 2S/(b(1+lambda))");
    near(g.cbar, (2.0 / 3.0) * g.cRoot * (1 + 0.5 + 0.25) / 1.5, 1e-9,
         "MAC = (2/3) c_r (1+l+l^2)/(1+l)");
    near(g.yMAC, (g.b / 6.0) * 2.0 / 1.5, 1e-9, "y_MAC = (b/6)(1+2l)/(1+l)");
    near(g.xLEmac, g.yMAC * std::tan(25.0 * M_PI / 180.0), 1e-9,
         "x_LEmac = y_MAC tan(Lambda_LE)");
    near(g.xAC - g.xLEmac, 0.25 * g.cbar, 1e-9, "空力中心は MAC 前縁から 25%");
    near(g.hn, 0.25, 1e-15, "h_n = 0.25");

    // MAC を面積分の定義 (2/S) INT c^2 dy からも確かめる
    {
        const int n = 200000;
        const double s = g.semiSpan, cr = g.cRoot, lam = 0.5;
        double acc = 0.0;
        for (int i = 0; i < n; ++i) {
            const double e = (i + 0.5) / n;
            const double c = cr * (1 - (1 - lam) * e);
            acc += c * c * (s / n);
        }
        near(2.0 * acc / d.S, g.cbar, 1e-4, "MAC = (2/S) INT c^2 dy と一致");
    }

    // 面密度一定の台形翼の質量中心は厳密に MAC の 50%
    near(g.xCGwing, g.xLEmac + 0.5 * g.cbar, 2e-3,
         "翼だけの質量中心 = MAC 50% (面密度一定の台形翼の厳密解)");

    // バラスト無しなら Kn = 0.25 - 0.50 = -0.25 (無尾翼機が必ず不安定な理由)
    WingDesign dw = d;
    dw.wingMassFrac = 0.999;
    near(computeGeom(dw).Kn, -0.25, 1e-2, "バラスト無しの無尾翼は Kn = -0.25");

    // バラストを前に置くほど重心が前進 -> Kn 増加
    WingDesign df = d, da = d;
    df.ballastXFrac = 0.05;
    da.ballastXFrac = 0.30;
    check(computeGeom(df).Kn > computeGeom(da).Kn, "バラストを前に置くほど Kn が大きい");
    check(computeGeom(df).xCG < computeGeom(da).xCG, "バラストを前に置くと重心が前進");

    // 後退角を増やすと MAC は変わらないが空力中心も重心も後退する
    WingDesign ds = d;
    ds.sweepLE = 40.0;
    WingGeom gs = computeGeom(ds);
    near(gs.cbar, g.cbar, 1e-9, "後退角は MAC の長さを変えない");
    check(gs.xAC > g.xAC, "後退角を増やすと空力中心が後退");
    check(gs.Iyy > g.Iyy, "後退角を増やすとピッチ慣性が増える");

    // アスペクト比を上げると MAC が短くなる
    WingDesign dh = d;
    dh.AR = 9.0;
    check(computeGeom(dh).cbar < g.cbar, "アスペクト比を上げると MAC が短くなる");

    std::printf("        b=%.2f  c_r=%.2f  MAC=%.3f  y_MAC=%.2f  x_ac=%.3f  x_cg=%.3f\n",
                g.b, g.cRoot, g.cbar, g.yMAC, g.xAC, g.xCG);
    std::printf("        Kn=%.4f  Iyy=%.0f kg m^2  k_y/MAC=%.2f\n",
                g.Kn, g.Iyy, g.kY / g.cbar);
}

// -------------------------------------------------------------- 静安定 ----
static void testStaticStability() {
    std::printf("\n[3] 静安定 (Cm_alpha, Cm_delta)\n");
    Aircraft ac = makeAircraft(1);
    near(ac.staticMargin(), ac.geom.Kn, 1e-15, "Kn = h_n - h");
    near(ac.Cmalpha(), -ac.geom.Kn * ac.CLalpha, 1e-12, "Cm_alpha = -Kn * CL_alpha");
    check(ac.Cmalpha() < 0.0, "静安定 (Cm_alpha < 0)");
    check(ac.Cm_ac > 0.0, "リフレックス翼型 + 後退角つきねじり下げで Cm_ac > 0");
    std::printf("        Kn=%.4f  CLa=%.3f  Cma=%.4f  Cm_ac=%.4f  CLd=%.3f  Cmd=%.4f\n",
                ac.geom.Kn, ac.CLalpha, ac.Cmalpha(), ac.Cm_ac, ac.CLdelta, ac.Cmdelta());

    // ねじり下げ 0 では Cm_ac は翼型の値だけ
    WingDesign d0 = makeDesign(1);
    d0.washout = 0.0;
    Aircraft a0 = buildAircraft(d0);
    near(a0.Cm_ac, d0.airfoilCm, 1e-9, "ねじり下げ 0 なら Cm_ac = 翼型 Cm");
    check(ac.Cm_ac > a0.Cm_ac, "後退翼 + ねじり下げで Cm_ac が増える (無尾翼の縦トリム源)");

    // 後退角 0 ではねじり下げても Cm_ac はほぼ変わらない
    WingDesign dn = makeDesign(1);
    dn.sweepLE = 0.0;
    Aircraft an = buildAircraft(dn);
    check(std::fabs(an.Cm_ac - dn.airfoilCm) < std::fabs(ac.Cm_ac - d0.airfoilCm) * 0.4,
          "後退角 0 ではねじり下げの Cm_ac 寄与が小さい");

    // Cm_alpha を有限差分で確認 (モデル内部と定義式の一致)
    const double eps = 1e-6;
    const double dCm = (CmSteady(ac, eps, 0.0) - CmSteady(ac, -eps, 0.0)) / (2 * eps);
    relNear(dCm, ac.Cmalpha(), 1e-4, "数値微分 dCm/dalpha が Cm_alpha() と一致");

    const double dCmd = (CmSteady(ac, 0.0, eps) - CmSteady(ac, 0.0, -eps)) / (2 * eps);
    relNear(dCmd, ac.Cmdelta(), 1e-4, "数値微分 dCm/ddelta が Cm_delta() と一致");
    check(ac.Cmdelta() < 0.0, "エレボン後縁下げで機首下げ (Cm_delta < 0)");

    // 重心後退で不安定化
    Aircraft un = makeAircraft(4);
    check(un.staticMargin() < 0.0, "LEVEL 5 は Kn < 0");
    check(un.Cmalpha() > 0.0, "LEVEL 5 は静不安定 (Cm_alpha > 0)");
}

// ---------------------------------------------------------------- トリム --
static void testTrim() {
    std::printf("\n[4] 水平定常飛行トリム\n");
    Aircraft ac = makeAircraft(1);
    const double Vt = std::round(std::sqrt(2 * ac.m * atmo::g0 /
                                           (isa(500.0).rho * ac.S * 0.5)));

    for (double V : {Vt - 5, Vt, Vt + 5, Vt + 12}) {
        TrimPoint t = solveTrim(ac, V, 500.0);
        char n[128];
        std::snprintf(n, sizeof(n), "V=%.0f m/s : トリム収束", V);
        check(t.ok, n);
        std::snprintf(n, sizeof(n), "V=%.0f m/s : Cm 残差 ~ 0", V);
        near(t.residCm, 0.0, 1e-9, n);
        std::snprintf(n, sizeof(n), "V=%.0f m/s : 揚力残差 ~ 0", V);
        near(t.residLift, 0.0, 1e-6, n);
        std::printf("        alpha=%6.2f deg  delta=%6.2f deg  T=%5.2f N  CL=%.3f\n",
                    t.alpha * 180 / M_PI, t.delta * 180 / M_PI, t.thrust, t.CL);
    }

    // 低速ほど大きな迎角 / 後縁上げが必要
    TrimPoint slow = solveTrim(ac, Vt - 5, 500.0);
    TrimPoint fast = solveTrim(ac, Vt + 12, 500.0);
    check(slow.alpha > fast.alpha, "低速ほど迎角が大きい");
    check(slow.delta < fast.delta, "低速ほど後縁上げ (delta が小さい)");

    // トリム状態から非線形モデルを回して定常が保たれるか
    TrimPoint t = solveTrim(ac, Vt, 500.0);
    State s; Controls c;
    trimToState(t, s, c);
    const double h0 = s.h;
    for (int i = 0; i < 6000; ++i) s = rk4(ac, s, c, 0.005);  // 30 s
    Derived d = derive(ac, s, c);
    near(s.h, h0, 0.5, "トリムから 30 s 積分して高度が保たれる");
    near(d.V, Vt, 0.2, "トリムから 30 s 積分して速度が保たれる");
    near(s.q, 0.0, 1e-3, "ピッチ角速度が 0 のまま");
}

// -------------------------------------------------------------- 固有値 ----
static void testEigenSolver() {
    std::printf("\n[5] 4 次固有値ソルバ\n");
    // 既知の固有値 1, 2, 3, 4 を持つ行列 (相似変換済み)
    double A[4][4] = {{4, 1, 0, 0}, {0, 3, 1, 0}, {0, 0, 2, 1}, {0, 0, 0, 1}};
    std::complex<double> e[4];
    eigen4(A, e);
    double sum = 0, prod = 1;
    for (auto& z : e) { sum += z.real(); prod *= z.real(); }
    near(sum, 10.0, 1e-8, "三角行列の固有値の和 = trace = 10");
    near(prod, 24.0, 1e-6, "固有値の積 = det = 24");

    // 複素固有値をもつ例 : ブロック [[0,1],[-4,-0.4]] -> -0.2 +- j1.99
    double B[4][4] = {{0, 1, 0, 0}, {-4, -0.4, 0, 0}, {0, 0, 0, 1}, {0, 0, -1, -0.2}};
    eigen4(B, e);
    double maxIm = 0;
    for (auto& z : e) maxIm = std::max(maxIm, std::fabs(z.imag()));
    near(maxIm, std::sqrt(4.0 - 0.04), 1e-6, "複素根の虚部 = sqrt(wn^2 - sigma^2)");

    // 一般行列で trace / det の一致を確認
    double C[4][4] = {{-0.05, 0.12, 0.0, -9.81},
                      {-0.60, -7.00, 22.0, 0.0},
                      {0.00, -20.0, -1.50, 0.0},
                      {0.00, 0.00, 1.00, 0.0}};
    eigen4(C, e);
    std::complex<double> s(0, 0), p(1, 0);
    for (auto& z : e) { s += z; p *= z; }
    near(s.real(), -0.05 - 7.00 - 1.50, 1e-8, "固有値の和 = trace");
    check(std::fabs(s.imag()) < 1e-8, "固有値の和は実数 (共役ペア)");
    check(std::fabs(p.imag()) < 1e-6, "固有値の積は実数");
}

// ------------------------------------------------ 動安定モードと近似式 ----
static void testModes() {
    std::printf("\n[6] 短周期モード / フゴイドモード\n");
    Aircraft ac = makeAircraft(1);
    const double Vt = std::round(std::sqrt(2 * ac.m * atmo::g0 /
                                           (isa(500.0).rho * ac.S * 0.5)));
    TrimPoint t = solveTrim(ac, Vt, 500.0);
    LinearModel lm = buildLinearModel(ac, t);
    ModeSet ms = analyzeModes(ac, lm);

    std::printf("        eig = ");
    for (auto& z : ms.eig) std::printf("%.4f%+.4fj  ", z.real(), z.imag());
    std::printf("\n        short period : wn=%.3f  zeta=%.3f   (approx wn=%.3f zeta=%.3f)\n",
                ms.shortPeriod.wn, ms.shortPeriod.zeta, ms.wnSpApprox, ms.zetaSpApprox);
    std::printf("        phugoid      : wn=%.3f  zeta=%.3f  T=%.2f s (approx wn=%.3f zeta=%.3f)\n",
                ms.phugoid.wn, ms.phugoid.zeta, ms.phugoid.period,
                ms.wnPhApprox, ms.zetaPhApprox);

    check(ms.shortPeriod.stable, "短周期モードが安定");
    check(ms.shortPeriod.wn > ms.phugoid.wn * 4, "短周期はフゴイドよりずっと速い");
    relNear(ms.shortPeriod.wn, ms.wnSpApprox, 0.15, "短周期 wn が教科書近似式と一致 (15%)");
    relNear(ms.shortPeriod.zeta, ms.zetaSpApprox, 0.20, "短周期 zeta が近似式と一致 (20%)");
    relNear(ms.phugoid.wn, ms.wnPhApprox, 0.25, "フゴイド wn が sqrt(2) g / u0 と一致 (25%)");
    check(std::fabs(ms.phugoid.zeta) < 0.2, "フゴイドは弱減衰 (|zeta| < 0.2)");
    check(std::fabs(ms.phugoid.zeta) < std::fabs(ms.shortPeriod.zeta),
          "フゴイドの方が減衰が弱い");

    // 静不安定機はどこかの固有値の実部が正
    Aircraft un = makeAircraft(4);
    TrimPoint tu = solveTrim(un, Vt, 500.0);
    LinearModel lu = buildLinearModel(un, tu);
    ModeSet mu = analyzeModes(un, lu);
    double maxRe = -1e9;
    for (auto& z : mu.eig) maxRe = std::max(maxRe, z.real());
    check(maxRe > 0.0, "Kn < 0 で発散根 (Re > 0) が現れる");
    std::printf("        unstable eig = ");
    for (auto& z : mu.eig) std::printf("%.4f%+.4fj  ", z.real(), z.imag());
    std::printf("\n");
}

// -------------------------------------------- 線形モデル vs 非線形モデル ---
static void testLinearVsNonlinear() {
    std::printf("\n[7] 微小舵角ステップでの線形 / 非線形の一致\n");
    Aircraft ac = makeAircraft(1);
    const double Vt = std::round(std::sqrt(2 * ac.m * atmo::g0 /
                                           (isa(500.0).rho * ac.S * 0.5)));
    TrimPoint t = solveTrim(ac, Vt, 500.0);
    LinearModel lm = buildLinearModel(ac, t);

    State s; Controls c;
    trimToState(t, s, c);
    const double step = 0.5 * M_PI / 180.0;  // 0.5 deg のエレボンステップ
    c.delta = t.delta + step;

    LinState x{};
    const double dt = 0.002;
    const int n = 1500;  // 3 s
    for (int i = 0; i < n; ++i) {
        s = rk4(ac, s, c, dt);
        x = linStep(lm, x, c.delta, c.throttle, dt);
    }
    State sl = linToState(lm, x);
    Derived dn = derive(ac, s, c);

    std::printf("        nonlinear: dtheta=%.4f deg  dh=%.3f m  dV=%.4f m/s\n",
                (s.theta - t.theta) * 180 / M_PI, s.h - t.h, dn.V - t.V);
    std::printf("        linear   : dtheta=%.4f deg  dh=%.3f m  dV=%.4f m/s\n",
                x.dtheta * 180 / M_PI, x.dh, x.du);

    near(sl.theta - t.theta, s.theta - t.theta, 0.6e-2, "3 s 後の dtheta が一致 (< 0.35 deg)");
    near(x.dh, s.h - t.h, 0.5, "3 s 後の dh が一致 (< 0.5 m)");
    near(sl.q, s.q, 5e-3, "3 s 後の q が一致");
}

// ----------------------------------------------------------- ゲーム部 -----
static double flyAutopilot(Game& p, double maxTime) {
    double throttle = p.trim().throttle;
    const double dt = 0.005;
    for (int i = 0; i < (int)(maxTime / dt) && p.status() == GameStatus::Flying; ++i) {
        const auto& s = p.state();
        const auto& d = p.derived();
        const double hErr = p.targetH() - s.h;
        const double vErr = p.targetV() - d.V;
        // 高度 -> 目標経路角 -> 目標ピッチ -> 舵角 の縦続 PD
        const double gamCmd = std::max(-0.10, std::min(0.10, 0.004 * hErr));
        const double thCmd  = gamCmd + d.alpha;
        // Cm_delta < 0 : 機首を上げるにはエレボンを後縁上げ (delta を減らす)
        const double dCmd   = p.trim().delta - 1.5 * (thCmd - s.theta) + 0.8 * s.q;
        throttle = std::max(0.0, std::min(1.0, throttle + 0.4 * vErr * dt));
        p.update(dt, dCmd, throttle);
    }
    return p.bestHold();
}

static void testGame() {
    std::printf("\n[8] ゲームロジック\n");
    Game g(1);
    check(g.phase() == Phase::Design, "起動時は設計フェーズ");
    g.startFlight();
    check(g.phase() == Phase::Flight, "startFlight で飛行フェーズへ");
    check(g.status() == GameStatus::Flying, "飛行中");
    check(g.modes().shortPeriod.wn > 0, "モード解析が走る");
    check(g.targetV() > 10 && g.targetV() < 200, "目標速度が妥当");

    // 設計変更が Kn とモードに反映される
    WingDesign d = g.design();
    const double Kn0 = g.aircraft().geom.Kn;
    d.ballastXFrac += 0.10;
    g.applyDesign(d);
    check(g.aircraft().geom.Kn < Kn0, "バラストを後ろへ動かすと Kn が減る");

    // 完璧なパイロット : 高度・速度の PD 制御でクリアできるか
    for (int lv = 0; lv < 4; ++lv) {
        Game p(lv);
        p.startFlight();
        flyAutopilot(p, 120.0);
        char n[128];
        std::snprintf(n, sizeof(n), "PD オートパイロットで LEVEL %d をクリア", lv + 1);
        std::printf("        L%d Kn=%+.4f wsp=%.2f zsp=%.2f  status=%d t=%.1f best=%.1f\n",
                    lv + 1, p.aircraft().geom.Kn, p.modes().shortPeriod.wn,
                    p.modes().shortPeriod.zeta, (int)p.status(), p.time(), p.bestHold());
        check(p.status() == GameStatus::Cleared, n);
    }

    // 線形モデルでも同じ操縦でクリアできる (モデル差が小さいことの確認)
    Game pl(1);
    pl.startFlight();
    pl.setMode(PhysicsMode::Linear);
    flyAutopilot(pl, 120.0);
    check(pl.status() == GameStatus::Cleared, "線形モデルでも同じ操縦でクリアできる");

    // 手を離すと (舵角固定) フゴイドで振動するが即墜落はしない
    Game f(0);
    f.startFlight();
    for (int i = 0; i < 8000; ++i) f.update(0.005, f.controls().delta, f.trim().throttle);
    check(f.state().h > 0.0, "40 s 放置でも即墜落しない (安定機)");
}

int main() {
    std::printf("=========== flightSim 検証テスト ===========\n");
    testAtmosphere();
    testPlanform();
    testStaticStability();
    testTrim();
    testEigenSolver();
    testModes();
    testLinearVsNonlinear();
    testGame();
    std::printf("\n============================================\n");
    std::printf("  pass: %d   fail: %d\n", g_pass, g_fail);
    return g_fail == 0 ? 0 : 1;
}
