#include "gui.hpp"
#include "aerodynamics/aerodynamics.hpp"
#include "atmosphere/atmosphere.hpp"
#include "stability/stability.hpp"

#include <imgui.h>
#include <implot.h>
#include <GLFW/glfw3.h>
#include <cmath>
#include <cstring>
#include <algorithm>
#include <string>

// ------------------------------------------------------------------ constants
static constexpr double RAD2DEG = 180.0 / M_PI;
static constexpr double DEG2RAD = M_PI / 180.0;
static constexpr double g_accel = 9.81;

// ------------------------------------------------------------------ Simulator
void Simulator::reset() {
    // 位置・速度を含む全状態をデフォルト値に戻す
    state = FlightState{};
    update_stability();
    auto atm = Atmosphere::at(state.h);
    double trim_alpha    = compute_trim_alpha(ac, atm.rho, state.V);
    state.theta          = trim_alpha;
    state.gamma          = 0.0;
    state.q              = 0.0;
    ctrl.delta_e         = 0.0;
    ctrl.throttle        = compute_trim_throttle(ac, atm.rho, state.V, trim_alpha);
    sim_time             = 0.0;
    crashed              = false;
    hist_t.clear(); hist_h.clear(); hist_V.clear();
    hist_alpha.clear(); hist_theta.clear(); hist_q.clear();
}

void Simulator::update_stability() {
    auto atm = Atmosphere::at(state.h);
    stability = compute_static_stability(ac, atm.rho);
}

void Simulator::step(double real_dt) {
    if (!running || crashed) return;
    // Run physics at fixed dt, capped to avoid spiral on lag spikes
    double budget = std::min(real_dt, 0.05);
    double elapsed = 0.0;
    while (elapsed < budget) {
        state = rk4_step(ac, state, ctrl, dt);
        sim_time += dt;
        elapsed  += dt;

        // Record history (downsample to every ~0.05s)
        if (hist_t.empty() || (sim_time - hist_t.back()) >= 0.05f) {
            auto push = [](std::vector<float>& v, float val, int max) {
                v.push_back(val);
                if ((int)v.size() > max) v.erase(v.begin());
            };
            push(hist_t,     (float)sim_time,                  HISTORY);
            push(hist_h,     (float)state.h,                   HISTORY);
            push(hist_V,     (float)state.V,                   HISTORY);
            push(hist_alpha, (float)(state.alpha() * RAD2DEG), HISTORY);
            push(hist_theta, (float)(state.theta   * RAD2DEG), HISTORY);
            push(hist_q,     (float)(state.q        * RAD2DEG), HISTORY);
        }

        if (state.h <= 0.0 && state.V < 1.0) {
            crashed = true;
            running = false;
            break;
        }
    }
}

// ------------------------------------------------------------------ helpers
static ImVec2 operator+(ImVec2 a, ImVec2 b) { return {a.x+b.x, a.y+b.y}; }
static ImVec2 operator-(ImVec2 a, ImVec2 b) { return {a.x-b.x, a.y-b.y}; }
static ImVec2 operator*(ImVec2 a, float s)  { return {a.x*s, a.y*s}; }

static void draw_diamond(ImDrawList* dl, ImVec2 p, float r, ImU32 col) {
    dl->AddQuadFilled({p.x, p.y-r}, {p.x+r, p.y}, {p.x, p.y+r}, {p.x-r, p.y}, col);
    dl->AddQuad({p.x, p.y-r}, {p.x+r, p.y}, {p.x, p.y+r}, {p.x-r, p.y}, IM_COL32(0,0,0,200), 1.5f);
}

static void draw_triangle_up(ImDrawList* dl, ImVec2 p, float r, ImU32 col) {
    dl->AddTriangleFilled({p.x, p.y-r}, {p.x+r, p.y+r}, {p.x-r, p.y+r}, col);
    dl->AddTriangle({p.x, p.y-r}, {p.x+r, p.y+r}, {p.x-r, p.y+r}, IM_COL32(0,0,0,200), 1.5f);
}

// ------------------------------------------------------------------ design panel
static void draw_design_panel(Simulator& sim) {
    ImGui::PushItemWidth(160.0f);
    bool changed = false;
    AircraftParams& ac = sim.ac;

    // double を float 経由で編集するヘルパー（ImGui は float のみ対応）
    auto sd = [&](const char* label, double& val, float vmin, float vmax) -> bool {
        float fv = (float)val;
        if (ImGui::SliderFloat(label, &fv, vmin, vmax)) {
            val = (double)fv;
            return true;
        }
        return false;
    };

    if (ImGui::CollapsingHeader("主翼", ImGuiTreeNodeFlags_DefaultOpen)) {
        changed |= sd("翼幅 b [m]",       ac.b,   2.0f, 30.0f);
        changed |= sd("翼弦長 c [m]",     ac.c,   0.3f,  5.0f);
        changed |= sd("取付角 i_w [deg]", ac.i_w, -5.0f, 10.0f);
        changed |= sd("翼位置 x_w [m]",  ac.x_w,  0.5f, (float)(ac.l_f - ac.c));
    }

    if (ImGui::CollapsingHeader("水平尾翼", ImGuiTreeNodeFlags_DefaultOpen)) {
        changed |= sd("尾翼幅 b_t [m]",       ac.b_t,  1.0f, 10.0f);
        changed |= sd("尾翼弦長 c_t [m]",     ac.c_t,  0.2f,  2.0f);
        changed |= sd("尾翼取付角 i_t [deg]", ac.i_t, -5.0f,  5.0f);
        changed |= sd("尾翼位置 x_t [m]",    ac.x_t,  2.0f, (float)(ac.l_f - ac.c_t));
    }

    if (ImGui::CollapsingHeader("胴体・重量")) {
        changed |= sd("機体全長 l_f [m]",  ac.l_f,  3.0f, 30.0f);
        changed |= sd("質量 m [kg]",       ac.mass, 10.0f, 5000.0f);
        changed |= sd("重心位置 x_cg [m]", ac.x_cg, 0.5f, (float)(ac.l_f - 0.5f));
    }

    if (ImGui::CollapsingHeader("推進")) {
        changed |= sd("最大推力 T [N]", ac.T_max, 0.0f, 20000.0f);
    }

    if (ImGui::CollapsingHeader("空力パラメータ")) {
        changed |= sd("CD0",          ac.CD0,         0.005f, 0.1f);
        changed |= sd("Oswald e",     ac.e,           0.5f,   1.0f);
        changed |= sd("失速角 [deg]", ac.alpha_stall, 5.0f,  30.0f);
    }
    ImGui::PopItemWidth();

    if (changed) sim.update_stability();

    ImGui::Separator();

    // Stability display
    auto& stab = sim.stability;
    ImGui::Text("中立点 NP: %.2f m", stab.x_np);

    float sm = (float)stab.SM;
    ImGui::Text("静安定マージン SM: %.1f %%MAC", sm);

    // Color bar for SM
    ImVec2 bar_pos = ImGui::GetCursorScreenPos();
    float bar_w = ImGui::GetContentRegionAvail().x;
    float bar_h = 14.0f;
    ImDrawList* dl = ImGui::GetWindowDrawList();
    // background
    dl->AddRectFilled(bar_pos, {bar_pos.x + bar_w, bar_pos.y + bar_h}, IM_COL32(60,60,60,255), 3.0f);
    // fill
    float clamp_sm = std::clamp(sm, -30.0f, 30.0f);
    float fill_frac = (clamp_sm + 30.0f) / 60.0f;
    ImU32 bar_col = stab.stable ? IM_COL32(60,200,80,255) : IM_COL32(220,60,60,255);
    dl->AddRectFilled(bar_pos, {bar_pos.x + bar_w * fill_frac, bar_pos.y + bar_h}, bar_col, 3.0f);
    dl->AddRect(bar_pos, {bar_pos.x + bar_w, bar_pos.y + bar_h}, IM_COL32(180,180,180,200), 3.0f);
    ImGui::Dummy({bar_w, bar_h + 2.0f});

    ImGui::Text("失速速度 Vs: %.1f m/s", stab.V_stall);
    if (!stab.stable)
        ImGui::TextColored({1.0f, 0.3f, 0.3f, 1.0f}, "警告: 静不安定");

    ImGui::Separator();

    // Derived info
    ImGui::Text("翼面積 S:  %.2f m²",  ac.S_w());
    ImGui::Text("アスペクト比 AR: %.2f", ac.AR_w());
    ImGui::Text("CL_alpha: %.3f /rad",  ac.CL_alpha_w());
    ImGui::Text("I_yy: %.0f kg·m²",    ac.I_yy());
}

// ------------------------------------------------------------------ side view
static void draw_side_view(const AircraftParams& ac, const StaticStability& stab) {
    ImVec2 canvas_pos  = ImGui::GetCursorScreenPos();
    ImVec2 canvas_size = ImGui::GetContentRegionAvail();
    canvas_size.y = 120.0f;
    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->AddRectFilled(canvas_pos, canvas_pos + canvas_size, IM_COL32(25, 30, 40, 255), 4.0f);

    float margin = 12.0f;
    float scale  = (canvas_size.x - 2.0f * margin) / (float)ac.l_f;

    // coordinate transform: world x [m] → screen
    auto wx = [&](double x_m) -> float {
        return canvas_pos.x + margin + (float)x_m * scale;
    };
    float cy = canvas_pos.y + canvas_size.y * 0.5f;

    // Fuselage
    float fuse_h = 8.0f;
    dl->AddRectFilled({wx(0), cy - fuse_h * 0.5f}, {wx(ac.l_f), cy + fuse_h * 0.5f},
                      IM_COL32(160, 160, 170, 230), 3.0f);

    // Wing
    float wing_w = (float)(ac.c) * scale;
    float wing_h = 18.0f;
    dl->AddRectFilled({wx(ac.x_w), cy - wing_h * 0.5f},
                      {wx(ac.x_w) + wing_w, cy + wing_h * 0.5f},
                      IM_COL32(80, 140, 210, 220), 2.0f);
    dl->AddRect({wx(ac.x_w), cy - wing_h * 0.5f},
                {wx(ac.x_w) + wing_w, cy + wing_h * 0.5f},
                IM_COL32(120, 180, 255, 200), 2.0f);

    // Tail
    float tail_w = (float)(ac.c_t) * scale;
    float tail_h = 12.0f;
    dl->AddRectFilled({wx(ac.x_t), cy - tail_h * 0.5f},
                      {wx(ac.x_t) + tail_w, cy + tail_h * 0.5f},
                      IM_COL32(80, 180, 120, 200), 2.0f);
    dl->AddRect({wx(ac.x_t), cy - tail_h * 0.5f},
                {wx(ac.x_t) + tail_w, cy + tail_h * 0.5f},
                IM_COL32(120, 220, 160, 200), 2.0f);

    float marker_y = canvas_pos.y + canvas_size.y - 18.0f;

    // CG - filled circle (red)
    float cg_x = wx(ac.x_cg);
    dl->AddCircleFilled({cg_x, marker_y}, 6.0f, IM_COL32(220, 60, 60, 255));
    dl->AddCircle({cg_x, marker_y}, 6.0f, IM_COL32(255, 100, 100, 255), 12, 1.5f);

    // AC - diamond (yellow)
    draw_diamond(dl, {wx(ac.x_ac_w()), marker_y}, 6.0f, IM_COL32(240, 200, 50, 255));

    // NP - triangle (green)
    draw_triangle_up(dl, {(float)wx(stab.x_np), marker_y}, 6.0f, IM_COL32(60, 220, 80, 255));

    // Legend
    dl->AddText({canvas_pos.x + margin, canvas_pos.y + 4.0f}, IM_COL32(255,255,255,180),
                "* CG   + AC   ^ NP");

    ImGui::Dummy({canvas_size.x, canvas_size.y + 4.0f});
}

// ------------------------------------------------------------------ flight view
static void draw_flight_view(Simulator& sim) {
    ImVec2 avail = ImGui::GetContentRegionAvail();
    avail.y -= 160.0f; // reserve space for plots below
    if (avail.y < 100.0f) avail.y = 100.0f;

    ImVec2 canvas_pos = ImGui::GetCursorScreenPos();
    ImDrawList* dl = ImGui::GetWindowDrawList();

    // Sky gradient
    dl->AddRectFilledMultiColor(canvas_pos, canvas_pos + avail,
        IM_COL32(10, 40, 80, 255), IM_COL32(10, 40, 80, 255),
        IM_COL32(30, 100, 180, 255), IM_COL32(30, 100, 180, 255));

    // Camera: center on aircraft
    float px_per_m = 2.5f;
    float cam_x = (float)sim.state.x;
    float cam_h = (float)sim.state.h;
    float cx = canvas_pos.x + avail.x * 0.4f;
    float cy = canvas_pos.y + avail.y * 0.65f;

    auto world_to_screen = [&](double wx, double wh) -> ImVec2 {
        return {cx + ((float)wx - cam_x) * px_per_m,
                cy - ((float)wh - cam_h) * px_per_m};
    };

    // Grid lines (altitude)
    float grid_spacing = 100.0f; // metres
    float h_first = std::floor(cam_h / grid_spacing) * grid_spacing;
    for (float gh = h_first - grid_spacing; gh <= h_first + avail.y / px_per_m + grid_spacing; gh += grid_spacing) {
        if (gh < -1.0f) continue;
        auto p = world_to_screen(cam_x, gh);
        dl->AddLine({canvas_pos.x, p.y}, {canvas_pos.x + avail.x, p.y},
                    IM_COL32(255, 255, 255, 20), 1.0f);
        char buf[32];
        snprintf(buf, sizeof(buf), "%.0fm", gh);
        dl->AddText({canvas_pos.x + 4.0f, p.y - 12.0f}, IM_COL32(200, 200, 200, 120), buf);
    }

    // Ground
    auto ground_p = world_to_screen(cam_x - 5000.0, 0.0);
    auto ground_q = world_to_screen(cam_x + 5000.0, 0.0);
    dl->AddRectFilled({ground_p.x, ground_p.y},
                      {ground_q.x, canvas_pos.y + avail.y},
                      IM_COL32(40, 90, 40, 255));
    dl->AddLine(ground_p, {ground_q.x, ground_p.y}, IM_COL32(80, 160, 80, 255), 2.0f);

    // Aircraft symbol (rotated based on pitch)
    ImVec2 ac_screen = world_to_screen(sim.state.x, sim.state.h);
    float theta_f = (float)sim.state.theta;
    float cos_t = std::cos(theta_f), sin_t = std::sin(theta_f);

    auto rot = [&](float lx, float ly) -> ImVec2 {
        return {ac_screen.x + (lx * cos_t - ly * sin_t),
                ac_screen.y - (lx * sin_t + ly * cos_t)};
    };

    float sc = 16.0f; // aircraft display scale
    // Fuselage
    dl->AddLine(rot(-sc, 0), rot(sc, 0), IM_COL32(230, 230, 240, 255), 3.0f);
    // Wings
    dl->AddLine(rot(-0.2f*sc, -sc*0.8f), rot(-0.2f*sc, sc*0.8f), IM_COL32(100, 160, 230, 255), 2.5f);
    // Tail
    dl->AddLine(rot(-sc*0.8f, -sc*0.3f), rot(-sc*0.8f, sc*0.3f), IM_COL32(100, 200, 130, 255), 2.0f);
    // Nose dot
    dl->AddCircleFilled(rot(sc, 0), 3.0f, IM_COL32(255, 200, 100, 255));

    // ---- HUD overlay ----
    float hx = canvas_pos.x + avail.x - 220.0f;
    float hy = canvas_pos.y + 10.0f;

    auto hud_text = [&](const char* label, const char* val) {
        dl->AddText({hx, hy}, IM_COL32(0, 0, 0, 120),   label); // shadow
        dl->AddText({hx-1, hy-1}, IM_COL32(180,255,180,255), label);
        dl->AddText({hx + 110.0f, hy}, IM_COL32(0,0,0,120), val);
        dl->AddText({hx + 109.0f, hy-1}, IM_COL32(255,255,150,255), val);
        hy += 16.0f;
    };
    char buf[64];
    snprintf(buf,sizeof(buf),"%.1f m/s", sim.state.V);     hud_text("対気速度:", buf);
    snprintf(buf,sizeof(buf),"%.0f m",   sim.state.h);     hud_text("高度:", buf);
    snprintf(buf,sizeof(buf),"%.1f°",    sim.state.theta * RAD2DEG); hud_text("ピッチ角:", buf);
    snprintf(buf,sizeof(buf),"%.1f°",    sim.state.gamma * RAD2DEG); hud_text("飛行経路角:", buf);
    snprintf(buf,sizeof(buf),"%.1f°",    sim.state.alpha() * RAD2DEG); hud_text("迎え角 α:", buf);
    snprintf(buf,sizeof(buf),"%.1f m/s", sim.state.V * std::sin(sim.state.gamma)); hud_text("昇降率:", buf);
    // G-load
    auto atm = Atmosphere::at(sim.state.h);
    AeroState asin{sim.state.alpha(), sim.ctrl.delta_e, sim.state.q, std::max(sim.state.V,1.0), atm.rho};
    auto af = compute_aero(sim.ac, asin);
    float nz = (float)(af.L / (sim.ac.mass * g_accel));
    snprintf(buf,sizeof(buf),"%.2f G",   nz); hud_text("荷重倍数:", buf);

    // 失速警告
    const double alpha_deg = sim.state.alpha() * RAD2DEG;
    const double stall_margin = sim.ac.alpha_stall - std::fabs(alpha_deg);
    if (stall_margin < 3.0) {
        ImU32 warn_col = (stall_margin < 0.0)
            ? IM_COL32(255, 50, 50, 220)   // 失速中：赤
            : IM_COL32(255, 200, 0, 220);  // 失速近接：黄
        const char* warn_msg = (stall_margin < 0.0) ? "  STALL  " : " STALL WARNING ";
        ImVec2 warn_pos = {canvas_pos.x + avail.x * 0.5f - 60.0f,
                           canvas_pos.y + avail.y * 0.5f - 20.0f};
        dl->AddRectFilled({warn_pos.x - 4, warn_pos.y - 2},
                          {warn_pos.x + 130, warn_pos.y + 18}, IM_COL32(0,0,0,180), 4.0f);
        dl->AddText(warn_pos, warn_col, warn_msg);
    }

    ImGui::Dummy(avail);

    // ---- Time plots ----
    if (!sim.hist_t.empty()) {
        float plot_h = 150.0f;
        if (ImPlot::BeginPlot("##history", {-1.0f, plot_h},
            ImPlotFlags_NoTitle | ImPlotFlags_NoMouseText)) {
            ImPlot::SetupAxes("時刻 [s]", nullptr, ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);
            ImPlot::PlotLine("高度 [m]",      sim.hist_t.data(), sim.hist_h.data(),     (int)sim.hist_t.size());
            ImPlot::PlotLine("速度 [m/s]",    sim.hist_t.data(), sim.hist_V.data(),     (int)sim.hist_t.size());
            ImPlot::PlotLine("迎え角 α [°]", sim.hist_t.data(), sim.hist_alpha.data(), (int)sim.hist_t.size());
            ImPlot::PlotLine("ピッチ θ [°]", sim.hist_t.data(), sim.hist_theta.data(), (int)sim.hist_t.size());
            ImPlot::EndPlot();
        }
    }
}

// ------------------------------------------------------------------ render_gui
void render_gui(Simulator& sim) {
    ImGuiIO& io = ImGui::GetIO();

    // Full-screen window
    ImGui::SetNextWindowPos({0, 0});
    ImGui::SetNextWindowSize(io.DisplaySize);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {6.0f, 6.0f});
    ImGui::Begin("##main", nullptr,
        ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoBringToFrontOnFocus);
    ImGui::PopStyleVar();

    // ---- Toolbar ----
    if (sim.running) {
        ImGui::PushStyleColor(ImGuiCol_Button, {0.7f, 0.2f, 0.2f, 1.0f});
        if (ImGui::Button(" Stop ")) sim.running = false;
        ImGui::PopStyleColor();
    } else {
        ImGui::PushStyleColor(ImGuiCol_Button, {0.2f, 0.6f, 0.2f, 1.0f});
        if (ImGui::Button(" Start ")) {
            if (sim.crashed) sim.reset();
            sim.running = true;
        }
        ImGui::PopStyleColor();
    }
    ImGui::SameLine();
    if (ImGui::Button(" Reset ")) { sim.reset(); sim.running = false; }
    ImGui::SameLine();
    ImGui::Text("Time: %.1f s", sim.sim_time);
    ImGui::SameLine(0, 30.0f);

    // Throttle slider
    float thr = (float)sim.ctrl.throttle * 100.0f;
    ImGui::PushItemWidth(120.0f);
    if (ImGui::SliderFloat("Throttle%%", &thr, 0.0f, 100.0f)) {
        sim.ctrl.throttle = thr / 100.0f;
    }
    ImGui::PopItemWidth();
    ImGui::SameLine();
    float de_deg = (float)(sim.ctrl.delta_e * RAD2DEG);
    ImGui::Text("Elevator: %+.1f deg", de_deg);

    if (sim.crashed) {
        ImGui::SameLine(0, 20.0f);
        ImGui::TextColored({1.0f, 0.3f, 0.3f, 1.0f}, "CRASHED");
    }
    ImGui::Separator();

    // ---- Keyboard input ----
    if (sim.running) {
        double de_rate = 20.0 * DEG2RAD * io.DeltaTime;
        double de_max  = sim.ac.de_max * DEG2RAD;
        bool up   = ImGui::IsKeyDown(ImGuiKey_UpArrow)   || ImGui::IsKeyDown(ImGuiKey_W);
        bool down = ImGui::IsKeyDown(ImGuiKey_DownArrow) || ImGui::IsKeyDown(ImGuiKey_S);
        if (up)
            sim.ctrl.delta_e = std::min(sim.ctrl.delta_e + de_rate, de_max);
        if (down)
            sim.ctrl.delta_e = std::max(sim.ctrl.delta_e - de_rate, -de_max);
        if (!up && !down) {
            // neutral return
            sim.ctrl.delta_e *= (1.0 - 3.0 * io.DeltaTime);
        }
        if (ImGui::IsKeyDown(ImGuiKey_Equal) || ImGui::IsKeyDown(ImGuiKey_KeypadAdd))
            sim.ctrl.throttle = std::min(sim.ctrl.throttle + 0.5 * io.DeltaTime, 1.0);
        if (ImGui::IsKeyDown(ImGuiKey_Minus) || ImGui::IsKeyDown(ImGuiKey_KeypadSubtract))
            sim.ctrl.throttle = std::max(sim.ctrl.throttle - 0.5 * io.DeltaTime, 0.0);
    }

    // ---- Two-column layout ----
    float left_w = 280.0f;
    ImGui::Columns(2, "main_cols", false);
    ImGui::SetColumnWidth(0, left_w);

    // Left: design panel + side view
    ImGui::BeginChild("##design", {left_w - 8.0f, 0.0f}, ImGuiChildFlags_None);
    draw_design_panel(sim);
    ImGui::Separator();
    ImGui::Text("Aircraft side view");
    draw_side_view(sim.ac, sim.stability);
    ImGui::Text("Controls: Up/Down/W/S = elevator  +/- = throttle");
    ImGui::EndChild();

    ImGui::NextColumn();

    // Right: flight view
    ImGui::BeginChild("##flight", {0.0f, 0.0f}, ImGuiChildFlags_None);
    draw_flight_view(sim);
    ImGui::EndChild();

    ImGui::Columns(1);
    ImGui::End();
}
