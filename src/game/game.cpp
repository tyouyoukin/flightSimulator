#include "game.h"

#include <algorithm>
#include <cmath>

#include "../atmosphere/atmosphere.h"

namespace fs {

Game::Game(int level) { reset(level); }

void Game::reset(int level) {
    level_ = std::clamp(level, 0, numLevels() - 1);
    design_ = makeDesign(level_);
    phase_ = Phase::Design;
    applyDesign(design_);
}

void Game::applyDesign(const WingDesign& d) {
    design_ = d;
    ac_ = buildAircraft(design_);
    rebuildAnalysis();
    placeAtStart();
}

void Game::rebuildAnalysis() {
    // 目標速度は CL = 0.5 相当 (設計が変わっても妥当な速度になるように)
    const AtmoState at = isa(targetH_);
    targetV_ = std::sqrt(2.0 * ac_.m * atmo::g0 / (at.rho * ac_.S * 0.50));
    targetV_ = std::round(targetV_);
    bandV_ = std::max(3.0, 0.10 * targetV_);

    trim_  = solveTrim(ac_, targetV_, targetH_);
    lin_   = buildLinearModel(ac_, trim_);
    modes_ = analyzeModes(ac_, lin_);
}

void Game::placeAtStart() {
    // トリムから少しずらしてスタート : プレイヤーがトリムを取り直す
    trimToState(trim_, st_, ctl_);
    st_.h -= 25.0;
    ctl_.delta = trim_.delta - 1.0 * M_PI / 180.0;

    lst_ = LinState{};
    lst_.dh = st_.h - trim_.h;
    lst_.dtheta = st_.theta - trim_.theta;

    t_ = 0.0;
    hold_ = 0.0;
    bestHold_ = 0.0;
    stallTimer_ = 0.0;
    inBand_ = false;
    status_ = GameStatus::Flying;
    msg_.clear();
    hist_.clear();
    der_ = derive(ac_, st_, ctl_);
}

void Game::startFlight() {
    ac_ = buildAircraft(design_);
    rebuildAnalysis();
    placeAtStart();
    phase_ = Phase::Flight;
}

void Game::restartFlight() {
    placeAtStart();
    phase_ = Phase::Flight;
}

void Game::setMode(PhysicsMode m) {
    if (m == mode_) return;
    mode_ = m;
    if (m == PhysicsMode::Linear) {
        // 非線形状態を安定軸の摂動量へ写す
        const double at = trim_.alpha;
        const double us =  st_.u * std::cos(at) + st_.w * std::sin(at);
        const double ws = -st_.u * std::sin(at) + st_.w * std::cos(at);
        lst_.du = us - lin_.u0;
        lst_.dw = ws;
        lst_.dq = st_.q;
        lst_.dtheta = st_.theta - trim_.theta;
        lst_.dh = st_.h - trim_.h;
        lst_.dx = st_.x;
    }
}

double Game::trimDeltaNow() const {
    const TrimPoint t = solveTrim(ac_, std::max(der_.V, 10.0), st_.h);
    return t.delta;
}

void Game::update(double dt, double deltaCmd, double throttleCmd) {
    if (phase_ != Phase::Flight || status_ != GameStatus::Flying) return;

    // ピッチダンパー (SAS) : 静不安定機を人間が扱えるようにする補助
    if (sas_) deltaCmd += 0.60 * st_.q;

    ctl_.delta = slewDelta(ac_, ctl_.delta, deltaCmd, dt);
    ctl_.throttle = std::clamp(throttleCmd, 0.0, 1.0);

    if (mode_ == PhysicsMode::Nonlinear) {
        st_ = rk4(ac_, st_, ctl_, dt);
    } else {
        lst_ = linStep(lin_, lst_, ctl_.delta, ctl_.throttle, dt);
        st_ = linToState(lin_, lst_);
    }
    der_ = derive(ac_, st_, ctl_);
    t_ += dt;

    // ---- バンド判定 --------------------------------------------------------
    const bool hOK = std::fabs(st_.h - targetH_) <= bandH_;
    const bool vOK = std::fabs(der_.V - targetV_) <= bandV_;
    inBand_ = hOK && vOK;
    if (inBand_) {
        hold_ += dt;
        bestHold_ = std::max(bestHold_, hold_);
        if (hold_ >= holdRequired_) {
            status_ = GameStatus::Cleared;
            msg_ = "LEVEL CLEAR";
        }
    } else {
        hold_ = 0.0;
    }

    // ---- 失敗判定 ----------------------------------------------------------
    if (st_.h <= 0.0) {
        status_ = GameStatus::Crashed;
        msg_ = "CRASHED  (h = 0)";
    } else if (std::fabs(st_.theta) > 85.0 * M_PI / 180.0) {
        status_ = GameStatus::Crashed;
        msg_ = "LOST CONTROL  (|theta| > 85 deg)";
    }
    if (der_.aero.stalled) {
        stallTimer_ += dt;
        if (stallTimer_ > 8.0) {
            status_ = GameStatus::Crashed;
            msg_ = "DEEP STALL";
        }
    } else {
        stallTimer_ = 0.0;
    }

    // ---- 履歴 --------------------------------------------------------------
    hist_.push_back(Sample{t_, st_.h, der_.V, der_.alpha * 180.0 / M_PI, ctl_.delta, st_.theta});
    while (!hist_.empty() && t_ - hist_.front().t > histSpan_) hist_.pop_front();
}

}  // namespace fs
