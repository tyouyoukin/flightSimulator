// ---------------------------------------------------------------------------
// 無尾翼機 縦安定シミュレータ / ゲーム  --  raylib GUI
//
//   [DESIGN]  アスペクト比・テーパー比・後退角・ねじり下げ・バラスト位置 を設定
//             -> MAC / 空力中心 (MAC 25%) / 質量中心 / Kn / 固有モード が決まる
//   [FLIGHT]  エレボン (フラップ) だけで目標高度・速度バンドに留まる
//
//   ※ raylib の既定フォントは ASCII のみなので画面表記は英語。
//      式と解説は README とソースのコメント (日本語) を参照。
// ---------------------------------------------------------------------------
#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <string>
#include <vector>

#include "raylib.h"
#include "../game/game.h"

using namespace fs;

static const int SCRW = 1440;
static const int SCRH = 900;

static const Color BG      = {14, 17, 23, 255};
static const Color PANEL   = {22, 27, 36, 255};
static const Color GRID    = {40, 48, 60, 255};
static const Color FG      = {214, 222, 235, 255};
static const Color DIM     = {126, 138, 156, 255};
static const Color ACCENT  = {88, 166, 255, 255};
static const Color GOOD    = {63, 185, 111, 255};
static const Color WARN    = {230, 170, 60, 255};
static const Color BAD     = {235, 90, 90, 255};
static const Color CGCOL   = {235, 110, 110, 255};
static const Color ACCOL   = {110, 190, 255, 255};

static inline double R2D(double r) { return r * 180.0 / M_PI; }
static inline double D2R(double d) { return d * M_PI / 180.0; }

static void txt(int x, int y, const char* s, int size, Color c) { DrawText(s, x, y, size, c); }

static void txtf(int x, int y, int size, Color c, const char* fmt, ...) {
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    DrawText(buf, x, y, size, c);
}

static void panel(int x, int y, int w, int h, const char* title) {
    DrawRectangle(x, y, w, h, PANEL);
    DrawRectangleLines(x, y, w, h, GRID);
    if (title) txt(x + 10, y + 7, title, 14, DIM);
}

// ===========================================================================
//  平面形の描画 (上から見た図)
// ===========================================================================
static void drawPlanform(const Aircraft& ac, Rectangle r, bool showElevon,
                         bool labels = true) {
    const WingGeom& g = ac.geom;
    const WingDesign& d = ac.design;

    const double xMax = g.semiSpan * std::tan(D2R(d.sweepLE)) + g.cTip;
    const double xExt = std::max(xMax, g.cRoot) * 1.12;
    const double yExt = g.b * 1.06;
    const double sc = std::min(r.width / yExt, r.height / xExt);

    const double cx = r.x + r.width * 0.5;
    const double cy = r.y + r.height * 0.06;
    auto P = [&](double y, double x) {  // y: spanwise (m), x: chordwise aft (m)
        return Vector2{(float)(cx + y * sc), (float)(cy + x * sc)};
    };
    auto chord = [&](double e) { return g.cRoot * (1.0 - (1.0 - d.taper) * e); };
    auto xle = [&](double e) { return e * g.semiSpan * std::tan(D2R(d.sweepLE)); };

    // 翼平面 (左右)
    for (int sgn = -1; sgn <= 1; sgn += 2) {
        Vector2 a = P(0, 0);
        Vector2 b = P(sgn * g.semiSpan, xle(1.0));
        Vector2 c = P(sgn * g.semiSpan, xle(1.0) + g.cTip);
        Vector2 e = P(0, g.cRoot);
        DrawTriangle(sgn < 0 ? a : e, sgn < 0 ? b : c, sgn < 0 ? c : b, Color{46, 56, 72, 255});
        DrawTriangle(sgn < 0 ? a : e, sgn < 0 ? c : b, sgn < 0 ? e : a, Color{46, 56, 72, 255});
        DrawLineEx(a, b, 2, FG);
        DrawLineEx(b, c, 2, FG);
        DrawLineEx(c, e, 2, FG);

        // エレボン
        if (showElevon) {
            const double e1 = d.flapEtaInner;
            Vector2 f1 = P(sgn * e1 * g.semiSpan, xle(e1) + chord(e1) * (1 - d.flapChordRatio));
            Vector2 f2 = P(sgn * g.semiSpan, xle(1) + g.cTip * (1 - d.flapChordRatio));
            Vector2 f3 = P(sgn * g.semiSpan, xle(1) + g.cTip);
            Vector2 f4 = P(sgn * e1 * g.semiSpan, xle(e1) + chord(e1));
            DrawTriangle(sgn < 0 ? f1 : f4, sgn < 0 ? f2 : f3, sgn < 0 ? f3 : f2,
                         Color{70, 100, 140, 255});
            DrawTriangle(sgn < 0 ? f1 : f4, sgn < 0 ? f3 : f2, sgn < 0 ? f4 : f1,
                         Color{70, 100, 140, 255});
            DrawLineEx(f1, f2, 1.5f, ACCENT);
        }
    }
    // 1/4 弦線
    DrawLineEx(P(-g.semiSpan, xle(1) + 0.25 * g.cTip), P(0, 0.25 * g.cRoot), 1.0f,
               Color{90, 100, 120, 255});
    DrawLineEx(P(0, 0.25 * g.cRoot), P(g.semiSpan, xle(1) + 0.25 * g.cTip), 1.0f,
               Color{90, 100, 120, 255});

    // MAC
    DrawLineEx(P(g.yMAC, g.xLEmac), P(g.yMAC, g.xLEmac + g.cbar), 3.0f, WARN);
    DrawLineEx(P(-g.yMAC, g.xLEmac), P(-g.yMAC, g.xLEmac + g.cbar), 3.0f, WARN);
    if (labels)
        txtf((int)P(g.yMAC, g.xLEmac).x + 6, (int)P(g.yMAC, g.xLEmac).y - 16, 12, WARN,
             "MAC %.2f m", g.cbar);

    // AC / CG の基準線
    Vector2 acL = P(-g.b * 0.50, g.xAC), acR = P(g.b * 0.50, g.xAC);
    DrawLineEx(acL, acR, 1.0f, Fade(ACCOL, 0.45f));
    Vector2 cgL = P(-g.b * 0.50, g.xCG), cgR = P(g.b * 0.50, g.xCG);
    DrawLineEx(cgL, cgR, 1.0f, Fade(CGCOL, 0.45f));

    DrawCircleV(P(g.yMAC, g.xAC), 6, ACCOL);
    DrawCircleV(P(0, g.xCG), 7, CGCOL);
    DrawCircleLinesV(P(0, g.xCG), 9, CGCOL);
    DrawCircleV(P(0, g.xBallast), 5, Color{200, 200, 90, 255});

    if (labels) {
        txt((int)acR.x - 86, (int)acR.y - 16, "AC = 25% MAC", 12, ACCOL);
        txt((int)cgL.x + 2, (int)cgL.y + 5, "CG = mass centroid", 12, CGCOL);
        txtf((int)P(0, g.xBallast).x + 12, (int)P(0, g.xBallast).y - 6, 12,
             Color{200, 200, 90, 255}, "ballast");
        // 静安定余裕
        Vector2 a = P(-g.b * 0.30, g.xCG), b = P(-g.b * 0.30, g.xAC);
        DrawLineEx(a, b, 2.5f, g.Kn > 0 ? GOOD : BAD);
        txtf((int)a.x + 8, (int)((a.y + b.y) / 2) - 8, 14, g.Kn > 0 ? GOOD : BAD,
             "Kn=%+.3f", g.Kn);
    }
}

// ===========================================================================
//  Cm - alpha 線図
// ===========================================================================
static void drawCmAlpha(const Game& gm, Rectangle r) {
    panel((int)r.x, (int)r.y, (int)r.width, (int)r.height, "Cm - alpha   (pitching moment about CG)");
    const Aircraft& ac = gm.aircraft();
    const double a0 = -8, a1 = 20;             // deg
    const double m0 = -0.20, m1 = 0.20;
    auto PX = [&](double ad) { return r.x + 42 + (float)((ad - a0) / (a1 - a0) * (r.width - 56)); };
    auto PY = [&](double cm) { return r.y + r.height - 28 - (float)((cm - m0) / (m1 - m0) * (r.height - 52)); };

    for (int i = 0; i <= 7; ++i) {
        float x = PX(a0 + i * (a1 - a0) / 7);
        DrawLine((int)x, (int)r.y + 24, (int)x, (int)(r.y + r.height - 28), GRID);
    }
    for (int i = 0; i <= 4; ++i) {
        float y = PY(m0 + i * (m1 - m0) / 4);
        DrawLine((int)r.x + 42, (int)y, (int)(r.x + r.width - 14), (int)y, GRID);
        txtf((int)r.x + 6, (int)y - 6, 11, DIM, "%+.2f", m0 + i * (m1 - m0) / 4);
    }
    DrawLine((int)r.x + 42, (int)PY(0), (int)(r.x + r.width - 14), (int)PY(0), Color{80, 92, 110, 255});
    txtf((int)PX(0) - 4, (int)(r.y + r.height - 24), 11, DIM, "0");
    txtf((int)PX(15) - 8, (int)(r.y + r.height - 24), 11, DIM, "15 deg");

    // delta を数本ふって Cm(alpha) を描く
    for (int k = -2; k <= 2; ++k) {
        const double dl = k * D2R(10.0);
        Color col = Fade(DIM, 0.55f);
        Vector2 prev{};
        bool first = true;
        for (int i = 0; i <= 120; ++i) {
            const double ad = a0 + (a1 - a0) * i / 120.0;
            const double cm = CmSteady(ac, D2R(ad), dl);
            Vector2 p{PX(ad), PY(std::clamp(cm, m0, m1))};
            if (!first) DrawLineEx(prev, p, 1.0f, col);
            prev = p;
            first = false;
        }
        txtf((int)PX(a1) - 52, (int)PY(std::clamp(CmSteady(ac, D2R(a1 - 1), dl), m0, m1)) - 6,
             10, col, "d=%+d", (int)R2D(dl));
    }
    // 現在の delta の曲線
    {
        const double dl = gm.controls().delta;
        Vector2 prev{};
        bool first = true;
        for (int i = 0; i <= 160; ++i) {
            const double ad = a0 + (a1 - a0) * i / 160.0;
            const double cm = CmSteady(ac, D2R(ad), dl);
            Vector2 p{PX(ad), PY(std::clamp(cm, m0, m1))};
            if (!first) DrawLineEx(prev, p, 2.4f, ACCENT);
            prev = p;
            first = false;
        }
    }
    // 現在点
    const double aNow = R2D(gm.derived().alpha);
    const double cmNow = gm.derived().aero.Cm;
    DrawCircle((int)PX(std::clamp(aNow, a0, a1)), (int)PY(std::clamp(cmNow, m0, m1)), 6,
               gm.derived().aero.stalled ? BAD : GOOD);
    // トリム点 (Cm = 0)
    DrawCircleLines((int)PX(std::clamp(R2D(gm.trim().alpha), a0, a1)), (int)PY(0), 7, WARN);
    txtf((int)r.x + 46, (int)r.y + 26, 11, DIM,
         "Cm_a=%+.3f /rad   slope<0 = statically stable", ac.Cmalpha());
}

// ===========================================================================
//  s 平面 (固有値)
// ===========================================================================
static void drawSPlane(const Game& gm, Rectangle r) {
    panel((int)r.x, (int)r.y, (int)r.width, (int)r.height, "eigenvalues  (s-plane)");
    const ModeSet& ms = gm.modes();
    double lim = 0.5;
    for (auto& z : ms.eig) lim = std::max(lim, std::max(std::fabs(z.real()), std::fabs(z.imag())));
    lim *= 1.25;
    const float cx = r.x + r.width * 0.62f;
    const float cy = r.y + r.height * 0.55f;
    const float sc = std::min(r.width * 0.36f, r.height * 0.40f) / (float)lim;

    DrawRectangle((int)cx, (int)(r.y + 20), (int)(r.x + r.width - cx - 2),
                  (int)(r.height - 24), Color{48, 24, 24, 120});  // 不安定側 Re > 0
    DrawLine((int)(r.x + 8), (int)cy, (int)(r.x + r.width - 8), (int)cy, GRID);
    DrawLine((int)cx, (int)(r.y + 22), (int)cx, (int)(r.y + r.height - 6), Color{120, 80, 80, 255});

    for (auto& z : ms.eig) {
        const float x = cx + (float)z.real() * sc;
        const float y = cy - (float)z.imag() * sc;
        const Color c = z.real() < 0 ? GOOD : BAD;
        DrawLineEx({x - 6, y - 6}, {x + 6, y + 6}, 2.0f, c);
        DrawLineEx({x - 6, y + 6}, {x + 6, y - 6}, 2.0f, c);
    }
    txtf((int)r.x + 8, (int)r.y + 24, 11, GOOD, "Re<0 stable");
    txtf((int)cx + 6, (int)r.y + 24, 11, BAD, "Re>0");
    txtf((int)r.x + 8, (int)(r.y + r.height - 18), 11, DIM, "%.1f /s", -lim);
}

// ===========================================================================
//  時系列 (高度と舵角)
// ===========================================================================
static void drawHistory(const Game& gm, Rectangle r) {
    panel((int)r.x, (int)r.y, (int)r.width, (int)r.height,
          "time history   altitude (green) / elevon (blue)");
    const auto& hist = gm.history();
    if (hist.size() < 2) return;

    const double t1 = gm.time();
    const double t0 = t1 - gm.historySpan();
    const double hC = gm.targetH();
    const double hSpan = std::max(120.0, gm.bandH() * 5.0);

    auto PX = [&](double t) { return r.x + 46 + (float)((t - t0) / (t1 - t0) * (r.width - 60)); };
    auto PYh = [&](double h) {
        return r.y + r.height * 0.5f - (float)((h - hC) / hSpan * (r.height - 48));
    };
    // 目標バンド
    DrawRectangle((int)r.x + 46, (int)PYh(hC + gm.bandH()), (int)(r.width - 60),
                  (int)(PYh(hC - gm.bandH()) - PYh(hC + gm.bandH())), Color{40, 90, 60, 70});
    DrawLine((int)r.x + 46, (int)PYh(hC), (int)(r.x + r.width - 14), (int)PYh(hC),
             Color{70, 130, 95, 255});
    txtf((int)r.x + 6, (int)PYh(hC) - 6, 11, DIM, "%.0fm", hC);

    Vector2 prevH{}, prevD{};
    bool first = true;
    for (const auto& s : hist) {
        Vector2 ph{PX(s.t), PYh(std::clamp(s.h, hC - hSpan * 0.5, hC + hSpan * 0.5))};
        Vector2 pd{PX(s.t),
                   r.y + r.height - 16 - (float)((s.delta + 0.45) / 0.9 * (r.height - 40))};
        if (!first) {
            DrawLineEx(prevH, ph, 2.0f, GOOD);
            DrawLineEx(prevD, pd, 1.3f, Fade(ACCENT, 0.8f));
        }
        prevH = ph;
        prevD = pd;
        first = false;
    }
}

// ===========================================================================
//  飛行画面 : 側面図 + 水平儀
// ===========================================================================
static void drawAttitude(const Game& gm, Rectangle r) {
    panel((int)r.x, (int)r.y, (int)r.width, (int)r.height, "side view");
    const float cx = r.x + r.width * 0.44f;
    const float cy = r.y + r.height * 0.42f;
    const double th = gm.state().theta;
    const double gam = gm.derived().gamma;

    BeginScissorMode((int)r.x + 1, (int)r.y + 20, (int)r.width - 2, (int)r.height - 22);
    // 真横から見た図 : 水平線は画面に対して水平のまま，機体が theta だけ傾く
    const float hz = r.y + r.height * 0.72f;
    DrawRectangle((int)r.x, (int)r.y + 20, (int)r.width, (int)(hz - r.y - 20),
                  Color{30, 46, 68, 255});
    DrawRectangle((int)r.x, (int)hz, (int)r.width, (int)(r.y + r.height - hz),
                  Color{44, 40, 30, 255});
    DrawLine((int)r.x, (int)hz, (int)(r.x + r.width), (int)hz, Color{120, 132, 150, 255});
    txt((int)r.x + 8, (int)hz + 6, "horizon", 11, DIM);

    // 角度目盛 (機首基準)
    for (int dgr = -30; dgr <= 30; dgr += 10) {
        const float aa = (float)D2R(dgr);
        DrawLineEx({cx + 128 * std::cos(aa), cy - 128 * std::sin(aa)},
                   {cx + 138 * std::cos(aa), cy - 138 * std::sin(aa)}, 1.0f, Fade(DIM, 0.6f));
        if (dgr % 20 == 0)
            txtf((int)(cx + 144 * std::cos(aa)) - 6, (int)(cy - 144 * std::sin(aa)) - 5, 10,
                 Fade(DIM, 0.8f), "%d", dgr);
    }
    DrawLine((int)cx - 150, (int)cy, (int)cx + 150, (int)cy, Fade(DIM, 0.25f));

    // 機体シルエット (無尾翼の横断面) : theta だけ回転
    const float L = 110.0f;
    auto rot = [&](float ax, float ay) {
        const float c = (float)std::cos(-th), sn = (float)std::sin(-th);
        return Vector2{cx + ax * c - ay * sn, cy + ax * sn + ay * c};
    };
    Vector2 nose = rot( 0.52f * L,  0.00f * L);
    Vector2 upA  = rot( 0.20f * L, -0.075f * L);
    Vector2 upB  = rot(-0.20f * L, -0.045f * L);
    Vector2 te   = rot(-0.42f * L,  0.005f * L);
    Vector2 loB  = rot(-0.20f * L,  0.030f * L);
    Vector2 loA  = rot( 0.15f * L,  0.040f * L);
    const Color body = {214, 222, 235, 255};
    DrawTriangle(nose, upA, upB, body);
    DrawTriangle(nose, upB, te, body);
    DrawTriangle(nose, te, loB, body);
    DrawTriangle(nose, loB, loA, body);
    DrawLineEx(nose, upA, 1.5f, FG);
    DrawLineEx(te, nose, 1.0f, Fade(FG, 0.6f));

    // エレボン (後縁でヒンジ, 舵角 delta)
    const double dl = gm.controls().delta;
    Vector2 e0 = te;
    Vector2 e1 = rot(-0.42f * L - 0.30f * L * (float)std::cos(dl),
                     0.30f * L * (float)std::sin(dl));
    DrawLineEx(e0, e1, 5.0f, ACCENT);

    // 速度ベクトル (経路角 gamma) と機体軸
    DrawLineEx({cx, cy}, {cx + 150.0f * (float)std::cos(-th), cy + 150.0f * (float)std::sin(-th)},
               1.0f, Fade(FG, 0.35f));
    Vector2 v1{cx + 150.0f * (float)std::cos(-gam), cy + 150.0f * (float)std::sin(-gam)};
    DrawLineEx({cx, cy}, v1, 2.0f, Fade(GOOD, 0.9f));
    DrawCircleLinesV(v1, 6, GOOD);
    EndScissorMode();

    txtf((int)r.x + 10, (int)r.y + 26, 12, DIM, "theta %+6.2f deg", R2D(th));
    txtf((int)r.x + 10, (int)r.y + 42, 12, GOOD, "gamma %+6.2f deg", R2D(gam));
    txtf((int)r.x + 10, (int)r.y + 58, 12,
         gm.derived().aero.stalled ? BAD : ACCENT, "alpha %+6.2f deg", R2D(gm.derived().alpha));
}

// ===========================================================================
//  テープ (高度 / 速度)
// ===========================================================================
static void drawTape(Rectangle r, const char* label, double value, double target,
                     double band, double span, const char* unit) {
    panel((int)r.x, (int)r.y, (int)r.width, (int)r.height, label);
    auto PY = [&](double v) {
        return r.y + r.height * 0.5f - (float)((v - target) / span * (r.height - 46));
    };
    DrawRectangle((int)r.x + 2, (int)PY(target + band), (int)r.width - 4,
                  (int)(PY(target - band) - PY(target + band)), Color{40, 90, 60, 80});
    DrawLine((int)r.x + 2, (int)PY(target), (int)(r.x + r.width - 2), (int)PY(target), GOOD);

    const double step = span / 5.0;
    const double base = std::floor((target - span * 0.5) / step) * step;
    for (int i = 0; i <= 12; ++i) {
        const double v = base + i * step;
        const float y = PY(v);
        if (y < r.y + 22 || y > r.y + r.height - 4) continue;
        DrawLine((int)r.x + 2, (int)y, (int)r.x + 12, (int)y, GRID);
        txtf((int)r.x + 16, (int)y - 6, 11, DIM, "%.0f", v);
    }
    const float y = std::clamp(PY(value), r.y + 22.0f, r.y + r.height - 12.0f);
    DrawRectangle((int)r.x + 2, (int)y - 11, (int)r.width - 4, 22, Color{30, 40, 55, 235});
    DrawRectangleLines((int)r.x + 2, (int)y - 11, (int)r.width - 4, 22,
                       std::fabs(value - target) <= band ? GOOD : WARN);
    txtf((int)r.x + 8, (int)y - 7, 15, FG, "%7.1f%s", value, unit);
}

// ===========================================================================
//  設計画面
// ===========================================================================
struct DesignRow {
    const char* label;
    double* value;
    double step, lo, hi;
    const char* unit;
    const char* note;
};

static void drawDesignScreen(Game& gm, int& sel) {
    WingDesign& d = gm.designRef();
    DesignRow rows[] = {
        {"aspect ratio  AR",   &d.AR,           0.25, 2.0,  12.0, "",    "b = sqrt(AR*S)"},
        {"taper ratio   lam",  &d.taper,        0.05, 0.15, 1.00, "",    "c_tip / c_root"},
        {"LE sweep      L_LE", &d.sweepLE,      1.0, -10.0, 55.0, "deg", "moves AC and CG aft"},
        {"washout       eps",  &d.washout,      0.5, -10.0,  2.0, "deg", "tip twist  (with sweep -> +Cm0)"},
        {"ballast pos   x_b",  &d.ballastXFrac, 0.01, -0.10, 0.60, "c_r", "moves CG -> sets Kn"},
        {"ballast mass",       &d.wingMassFrac, 0.01, 0.40, 0.90, "wing frac", "wing / total mass"},
        {"elevon chord",       &d.flapChordRatio, 0.01, 0.08, 0.40, "c",  "control power"},
        {"elevon inner",       &d.flapEtaInner, 0.02, 0.10, 0.85, "eta", "inboard end of elevon"},
        {"airfoil Cm_ac",      &d.airfoilCm,    0.002, -0.05, 0.06, "",   "reflex airfoil -> positive"},
    };
    const int NROW = sizeof(rows) / sizeof(rows[0]);

    if (IsKeyPressed(KEY_DOWN)) sel = (sel + 1) % NROW;
    if (IsKeyPressed(KEY_UP))   sel = (sel + NROW - 1) % NROW;
    const double mul = IsKeyDown(KEY_LEFT_SHIFT) ? 5.0 : 1.0;
    bool changed = false;
    if (IsKeyPressed(KEY_RIGHT) || IsKeyPressedRepeat(KEY_RIGHT)) {
        *rows[sel].value = std::min(rows[sel].hi, *rows[sel].value + rows[sel].step * mul);
        changed = true;
    }
    if (IsKeyPressed(KEY_LEFT) || IsKeyPressedRepeat(KEY_LEFT)) {
        *rows[sel].value = std::max(rows[sel].lo, *rows[sel].value - rows[sel].step * mul);
        changed = true;
    }
    if (changed) gm.applyDesign(d);

    const Aircraft& ac = gm.aircraft();
    const WingGeom& g = ac.geom;

    txt(24, 18, "TAILLESS  LONGITUDINAL  STABILITY  --  DESIGN", 24, FG);
    txt(26, 48, "arrows: select / adjust   (SHIFT = x5)      ENTER: fly      1-5: preset      ESC: quit",
        14, DIM);

    // ---- 平面形 -----------------------------------------------------------
    Rectangle pf = {24, 76, 700, 470};
    panel((int)pf.x, (int)pf.y, (int)pf.width, (int)pf.height, "planform  (top view, nose up)");
    drawPlanform(ac, {pf.x + 20, pf.y + 30, pf.width - 40, pf.height - 50}, true);

    // ---- パラメータ --------------------------------------------------------
    Rectangle pr = {740, 76, 400, 470};
    panel((int)pr.x, (int)pr.y, (int)pr.width, (int)pr.height, "design parameters");
    for (int i = 0; i < NROW; ++i) {
        const int y = (int)pr.y + 32 + i * 46;
        if (i == sel) DrawRectangle((int)pr.x + 4, y - 4, (int)pr.width - 8, 42, Color{34, 44, 60, 255});
        txtf((int)pr.x + 14, y, 14, i == sel ? FG : DIM, "%s", rows[i].label);
        txtf((int)pr.x + 250, y, 15, i == sel ? ACCENT : FG, "%7.3f %s", *rows[i].value,
             rows[i].unit);
        txtf((int)pr.x + 14, y + 18, 11, Fade(DIM, 0.85f), "%s", rows[i].note);
    }

    // ---- 導出量 ------------------------------------------------------------
    Rectangle dr = {1156, 76, 260, 470};
    panel((int)dr.x, (int)dr.y, (int)dr.width, (int)dr.height, "derived");
    int y = (int)dr.y + 30;
    auto line = [&](const char* k, const char* fmt, double v, Color c = FG) {
        txtf((int)dr.x + 12, y, 12, DIM, "%s", k);
        char b[64];
        snprintf(b, sizeof(b), fmt, v);
        txt((int)dr.x + 150, y, b, 12, c);
        y += 19;
    };
    line("span  b",        "%8.2f m", g.b);
    line("root chord",     "%8.2f m", g.cRoot);
    line("tip chord",      "%8.2f m", g.cTip);
    line("MAC",            "%8.3f m", g.cbar);
    line("y_MAC",          "%8.2f m", g.yMAC);
    line("x_LE(MAC)",      "%8.3f m", g.xLEmac);
    line("x_ac (25% MAC)", "%8.3f m", g.xAC, ACCOL);
    line("x_cg (mass)",    "%8.3f m", g.xCG, CGCOL);
    line("h = xcg/MAC",    "%8.3f",   g.h);
    line("STATIC MARGIN",  "%+8.4f",  g.Kn, g.Kn > 0 ? GOOD : BAD);
    y += 6;
    line("Iyy",            "%8.0f",   g.Iyy);
    line("k_y / MAC",      "%8.3f",   g.kY / g.cbar);
    line("sweep c/4",      "%8.2f d", R2D(g.sweepC4));
    y += 6;
    line("CL_alpha",       "%8.3f",   ac.CLalpha);
    line("Cm_ac",          "%+8.4f",  ac.Cm_ac, ac.Cm_ac > 0 ? GOOD : BAD);
    line("Cm_alpha",       "%+8.4f",  ac.Cmalpha(), ac.Cmalpha() < 0 ? GOOD : BAD);
    line("Cm_delta",       "%+8.4f",  ac.Cmdelta());
    line("CL_delta",       "%+8.4f",  ac.CLdelta);
    line("Cm_q",           "%+8.4f",  ac.Cmq);
    line("Oswald e",       "%8.3f",   ac.e_oswald);

    // ---- トリムとモード ----------------------------------------------------
    Rectangle tr = {24, 560, 700, 310};
    panel((int)tr.x, (int)tr.y, (int)tr.width, (int)tr.height,
          "trim for level flight  &  longitudinal modes");
    const TrimPoint& t = gm.trim();
    const ModeSet& ms = gm.modes();
    txtf((int)tr.x + 16, (int)tr.y + 34, 14, FG,
         "target  V = %.0f m/s   h = %.0f m", gm.targetV(), gm.targetH());
    txtf((int)tr.x + 16, (int)tr.y + 56, 14, t.ok ? GOOD : BAD,
         "trim %s :  alpha = %+6.2f deg   elevon = %+6.2f deg   thrust = %.0f N (%.0f%%)",
         t.ok ? "OK " : "NG ", R2D(t.alpha), R2D(t.delta), t.thrust, t.throttle * 100);
    if (!t.ok)
        txtf((int)tr.x + 16, (int)tr.y + 76, 12, BAD,
             "cannot trim: check Cm_ac > 0, elevon authority, or stall");

    txtf((int)tr.x + 16, (int)tr.y + 108, 13, DIM, "SHORT PERIOD");
    txtf((int)tr.x + 150, (int)tr.y + 108, 13, ms.shortPeriod.stable ? GOOD : BAD,
         "wn = %6.3f rad/s   zeta = %+6.3f   T = %5.2f s   lambda = %+.3f %+.3fj",
         ms.shortPeriod.wn, ms.shortPeriod.zeta, ms.shortPeriod.period,
         ms.shortPeriod.lambda.real(), ms.shortPeriod.lambda.imag());
    txtf((int)tr.x + 16, (int)tr.y + 132, 13, DIM, "PHUGOID");
    txtf((int)tr.x + 150, (int)tr.y + 132, 13, ms.phugoid.stable ? GOOD : BAD,
         "wn = %6.3f rad/s   zeta = %+6.3f   T = %5.2f s   lambda = %+.3f %+.3fj",
         ms.phugoid.wn, ms.phugoid.zeta, ms.phugoid.period,
         ms.phugoid.lambda.real(), ms.phugoid.lambda.imag());
    txtf((int)tr.x + 16, (int)tr.y + 158, 11, Fade(DIM, 0.9f),
         "textbook approx:  sp wn=%.3f zeta=%.3f     ph wn=sqrt(2)g/u0=%.3f zeta=CD/(sqrt2 CL)=%.3f",
         ms.wnSpApprox, ms.zetaSpApprox, ms.wnPhApprox, ms.zetaPhApprox);
    txtf((int)tr.x + 16, (int)tr.y + 186, 12, DIM,
         "Kn = h_n - h = 0.25 - x_cg/MAC.   Cm_alpha = -Kn CL_alpha.");
    txtf((int)tr.x + 16, (int)tr.y + 204, 12, DIM,
         "A uniform trapezoidal wing has its mass centroid at 50%% MAC  ->  Kn = -0.25.");
    txtf((int)tr.x + 16, (int)tr.y + 222, 12, DIM,
         "Move the ballast forward to buy static margin; sweep + washout give Cm_ac > 0 for trim.");
    txtf((int)tr.x + 16, (int)tr.y + 250, 13, FG, "%s", levelName(gm.level()));
    txtf((int)tr.x + 16, (int)tr.y + 272, 14, ACCENT, "press ENTER to fly");

    drawCmAlpha(gm, {740, 560, 400, 310});
    drawSPlane(gm, {1156, 560, 260, 310});
}

// ===========================================================================
//  飛行画面
// ===========================================================================
static void drawFlightScreen(Game& gm) {
    const Aircraft& ac = gm.aircraft();
    const Derived& d = gm.derived();
    const State& s = gm.state();

    txtf(24, 16, 20, FG, "FLIGHT   %s", levelName(gm.level()));
    txtf(24, 42, 13, DIM,
         "UP/DOWN elevon    W/S throttle    TAB model    P pitch damper    R restart    D design    ESC quit");

    // 姿勢 + テープ
    drawAttitude(gm, {24, 70, 560, 330});
    drawTape({596, 70, 120, 330}, "IAS m/s", d.V, gm.targetV(), gm.bandV(), 40.0, "");
    drawTape({724, 70, 130, 330}, "ALT m", s.h, gm.targetH(), gm.bandH(), 200.0, "");

    // ---- 数値パネル --------------------------------------------------------
    Rectangle np = {866, 70, 274, 330};
    panel((int)np.x, (int)np.y, (int)np.width, (int)np.height, "state / aero");
    int y = (int)np.y + 30;
    auto line = [&](const char* k, const char* fmt, double v, Color c = FG) {
        txtf((int)np.x + 12, y, 12, DIM, "%s", k);
        char b[64];
        snprintf(b, sizeof(b), fmt, v);
        txt((int)np.x + 150, y, b, 12, c);
        y += 18;
    };
    line("V", "%8.2f m/s", d.V);
    line("h", "%8.1f m", s.h);
    line("alpha", "%+8.2f deg", R2D(d.alpha), d.aero.stalled ? BAD : FG);
    line("theta", "%+8.2f deg", R2D(s.theta));
    line("gamma", "%+8.2f deg", R2D(d.gamma));
    line("q", "%+8.2f deg/s", R2D(s.q));
    line("nz", "%+8.2f g", d.nz);
    y += 6;
    line("CL", "%8.3f", d.aero.CL);
    line("CD", "%8.4f", d.aero.CD);
    line("Cm", "%+8.4f", d.aero.Cm, std::fabs(d.aero.Cm) < 0.002 ? GOOD : FG);
    line("qbar", "%8.0f Pa", d.qbar);
    y += 6;
    line("elevon", "%+8.2f deg", R2D(gm.controls().delta), ACCENT);
    line("trim need", "%+8.2f deg", R2D(gm.trimDeltaNow()), WARN);
    line("throttle", "%8.0f %%", gm.controls().throttle * 100);
    y += 6;
    line("Kn", "%+8.4f", ac.geom.Kn, ac.geom.Kn > 0 ? GOOD : BAD);
    line("MAC", "%8.3f m", ac.cbar);

    // ---- モード / 平面形 ---------------------------------------------------
    Rectangle mp = {1156, 70, 260, 330};
    panel((int)mp.x, (int)mp.y, (int)mp.width, (int)mp.height, "aircraft");
    drawPlanform(ac, {mp.x + 14, mp.y + 26, mp.width - 28, mp.height - 44}, true, false);

    // ---- 下段 -------------------------------------------------------------
    drawCmAlpha(gm, {24, 414, 560, 300});
    drawHistory(gm, {596, 414, 544, 300});
    drawSPlane(gm, {1156, 414, 260, 300});

    // ---- エレボンバー + ステータス -----------------------------------------
    Rectangle sb = {24, 726, 1392, 150};
    panel((int)sb.x, (int)sb.y, (int)sb.width, (int)sb.height, nullptr);

    // エレボンバー
    const float bx = sb.x + 20, bw = 440, by = sb.y + 34, bh = 26;
    DrawRectangle((int)bx, (int)by, (int)bw, (int)bh, Color{30, 36, 48, 255});
    auto dpos = [&](double dl) {
        return bx + bw * (float)((dl - ac.deltaMin) / (ac.deltaMax - ac.deltaMin));
    };
    DrawLine((int)dpos(0), (int)by - 4, (int)dpos(0), (int)(by + bh + 4), GRID);
    DrawLine((int)dpos(gm.trimDeltaNow()), (int)by - 6, (int)dpos(gm.trimDeltaNow()),
             (int)(by + bh + 6), WARN);
    const float dx = dpos(gm.controls().delta);
    DrawRectangle((int)dx - 3, (int)by, 6, (int)bh, ACCENT);
    txtf((int)bx, (int)by - 18, 12, DIM,
         "ELEVON  %+.2f deg   (yellow = elevon needed to trim at this speed)",
         R2D(gm.controls().delta));
    txtf((int)bx, (int)(by + bh + 8), 11, DIM, "TE up (nose up)");
    txtf((int)(bx + bw - 92), (int)(by + bh + 8), 11, DIM, "TE down");

    // スロットルバー
    const float tx = sb.x + 500, tw = 180;
    DrawRectangle((int)tx, (int)by, (int)tw, (int)bh, Color{30, 36, 48, 255});
    DrawRectangle((int)tx, (int)by, (int)(tw * gm.controls().throttle), (int)bh,
                  Color{70, 110, 90, 255});
    txtf((int)tx, (int)by - 18, 12, DIM, "THROTTLE %.0f%%", gm.controls().throttle * 100);

    // ホールドゲージ
    const float hx = sb.x + 720, hw = 400;
    DrawRectangle((int)hx, (int)by, (int)hw, (int)bh, Color{30, 36, 48, 255});
    DrawRectangle((int)hx, (int)by, (int)(hw * gm.holdTime() / gm.holdRequired()), (int)bh,
                  gm.inBand() ? GOOD : Color{70, 80, 95, 255});
    DrawRectangleLines((int)hx, (int)by, (int)hw, (int)bh, GRID);
    txtf((int)hx, (int)by - 18, 12, gm.inBand() ? GOOD : DIM,
         "IN BAND  %.1f / %.0f s      (alt %.0f+-%.0f m,  V %.0f+-%.0f m/s)",
         gm.holdTime(), gm.holdRequired(), gm.targetH(), gm.bandH(), gm.targetV(), gm.bandV());

    txtf((int)sb.x + 20, (int)sb.y + 84, 13, DIM, "model:");
    txtf((int)sb.x + 80, (int)sb.y + 84, 13, ACCENT, "%s",
         gm.mode() == PhysicsMode::Nonlinear ? "NONLINEAR 3-DOF (u,w,q,theta)"
                                             : "LINEAR  xdot = A x + B delta");
    txtf((int)sb.x + 380, (int)sb.y + 84, 13, gm.sas() ? GOOD : DIM,
         "pitch damper: %s", gm.sas() ? "ON  (delta += 0.6 q)" : "OFF");
    txtf((int)sb.x + 640, (int)sb.y + 84, 13, DIM, "t = %.1f s", gm.time());
    txtf((int)sb.x + 760, (int)sb.y + 84, 13, DIM, "best hold = %.1f s", gm.bestHold());

    if (d.aero.stalled)
        txtf((int)sb.x + 20, (int)sb.y + 110, 16, BAD, "STALL");

    // 終了メッセージ
    if (gm.status() != GameStatus::Flying) {
        const Color c = gm.status() == GameStatus::Cleared ? GOOD : BAD;
        DrawRectangle(SCRW / 2 - 260, SCRH / 2 - 70, 520, 140, Color{10, 12, 18, 235});
        DrawRectangleLines(SCRW / 2 - 260, SCRH / 2 - 70, 520, 140, c);
        txtf(SCRW / 2 - 240, SCRH / 2 - 44, 34, c, "%s", gm.message().c_str());
        txtf(SCRW / 2 - 240, SCRH / 2 + 6, 15, FG, "R: retry     D: back to design     1-5: level");
    }
}

// ===========================================================================
int main() {
    SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_VSYNC_HINT);
    InitWindow(SCRW, SCRH, "Tailless Longitudinal Stability");
    SetTargetFPS(60);
    SetExitKey(KEY_NULL);

    Game gm(0);
    int sel = 4;                 // 設計画面の選択行 (バラスト位置)
    double deltaCmd = 0.0;
    double throttle = 0.0;
    bool init = false;

    while (!WindowShouldClose()) {
        if (IsKeyPressed(KEY_ESCAPE)) break;

        // ---- 入力 ---------------------------------------------------------
        for (int k = 0; k < numLevels(); ++k)
            if (IsKeyPressed(KEY_ONE + k)) {
                gm.reset(k);
                init = false;
            }

        if (gm.phase() == Phase::Design) {
            if (IsKeyPressed(KEY_ENTER) || IsKeyPressed(KEY_KP_ENTER)) {
                gm.startFlight();
                init = false;
            }
        } else {
            if (!init) {
                deltaCmd = gm.controls().delta;
                throttle = gm.trim().throttle;
                init = true;
            }
            if (IsKeyPressed(KEY_D)) gm.enterDesign();
            if (IsKeyPressed(KEY_R)) { gm.restartFlight(); init = false; }
            if (IsKeyPressed(KEY_TAB)) gm.toggleMode();
            if (IsKeyPressed(KEY_P)) gm.setSAS(!gm.sas());

            const double dt = std::min((double)GetFrameTime(), 0.05);
            const double rate = D2R(22.0);  // 操縦入力レート [rad/s]
            if (IsKeyDown(KEY_UP))   deltaCmd -= rate * dt;  // 後縁上げ = 機首上げ
            if (IsKeyDown(KEY_DOWN)) deltaCmd += rate * dt;
            deltaCmd = std::clamp(deltaCmd, gm.aircraft().deltaMin, gm.aircraft().deltaMax);
            if (IsKeyDown(KEY_W)) throttle += 0.35 * dt;
            if (IsKeyDown(KEY_S)) throttle -= 0.35 * dt;
            throttle = std::clamp(throttle, 0.0, 1.0);

            // 物理は固定ステップで積分 (描画レートに依存させない)
            static double acc = 0.0;
            acc += dt;
            const double H = 0.002;
            int guard = 0;
            while (acc >= H && guard++ < 200) {
                gm.update(H, deltaCmd, throttle);
                acc -= H;
            }
        }

        // ---- 描画 ---------------------------------------------------------
        BeginDrawing();
        ClearBackground(BG);
        if (gm.phase() == Phase::Design) drawDesignScreen(gm, sel);
        else                             drawFlightScreen(gm);
        EndDrawing();
    }
    CloseWindow();
    return 0;
}
