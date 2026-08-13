#pragma once
#include <deque>
#include <string>

#include "../stability/stability.h"

// ---------------------------------------------------------------------------
// ゲーム : 無尾翼機のエレボン (フラップ) だけを操作して水平飛行を維持する
//
//   [設計フェーズ]  アスペクト比 / テーパー比 / 前縁後退角 / ねじり下げ /
//                   バラスト位置 を設定する。
//                   → 平均空力翼弦 MAC・空力中心 (MAC 25%)・質量中心 が決まり，
//                     静安定余裕 Kn = 0.25 - x_cg/MAC と固有モードが決まる。
//
//   [飛行フェーズ]  目標高度バンド ± band [m] かつ 目標速度バンド ± vband [m/s] に
//                   holdRequired 秒間 連続で留まればクリア。
// ---------------------------------------------------------------------------

namespace fs {

enum class PhysicsMode { Nonlinear, Linear };
enum class Phase { Design, Flight };
enum class GameStatus { Flying, Cleared, Crashed };

struct Sample {
    double t = 0, h = 0, V = 0, alphaDeg = 0, delta = 0, theta = 0;
};

class Game {
public:
    explicit Game(int level = 0);

    // ---- 設計フェーズ ------------------------------------------------------
    void enterDesign() { phase_ = Phase::Design; }
    void applyDesign(const WingDesign& d);   // 設計を反映して機体を作り直す
    void startFlight();                      // 設計を確定して飛行開始
    Phase phase() const { return phase_; }
    const WingDesign& design() const { return design_; }
    WingDesign& designRef() { return design_; }  // GUI から直接いじる用

    void reset(int level);
    void reset() { reset(level_); }
    void restartFlight();  // 同じ設計のまま飛行だけやり直す

    // ---- 飛行フェーズ ------------------------------------------------------
    /// dt [s] 進める。deltaCmd は指令舵角 [rad], throttleCmd は 0..1
    void update(double dt, double deltaCmd, double throttleCmd);

    void setMode(PhysicsMode m);
    void toggleMode() {
        setMode(mode_ == PhysicsMode::Nonlinear ? PhysicsMode::Linear : PhysicsMode::Nonlinear);
    }
    void setSAS(bool on) { sas_ = on; }
    bool sas() const { return sas_; }

    // ---- アクセサ ----------------------------------------------------------
    const Aircraft&    aircraft() const { return ac_; }
    const State&       state()    const { return st_; }
    const Controls&    controls() const { return ctl_; }
    const Derived&     derived()  const { return der_; }
    const TrimPoint&   trim()     const { return trim_; }
    const LinearModel& linear()   const { return lin_; }
    const ModeSet&     modes()    const { return modes_; }
    PhysicsMode        mode()     const { return mode_; }
    GameStatus         status()   const { return status_; }
    int    level()        const { return level_; }
    double time()         const { return t_; }
    double holdTime()     const { return hold_; }
    double bestHold()     const { return bestHold_; }
    double holdRequired() const { return holdRequired_; }
    double targetH()      const { return targetH_; }
    double bandH()        const { return bandH_; }
    double targetV()      const { return targetV_; }
    double bandV()        const { return bandV_; }
    bool   inBand()       const { return inBand_; }
    const std::string& message() const { return msg_; }
    const std::deque<Sample>& history() const { return hist_; }
    double historySpan()  const { return histSpan_; }

    /// いまの速度・高度で水平定常飛行するのに必要な舵角 (アシスト表示用)
    double trimDeltaNow() const;

private:
    void rebuildAnalysis();
    void placeAtStart();

    WingDesign design_{};
    Aircraft ac_{};
    State st_{};
    Controls ctl_{};
    Derived der_{};
    TrimPoint trim_{};
    LinearModel lin_{};
    ModeSet modes_{};
    LinState lst_{};

    Phase phase_ = Phase::Design;
    PhysicsMode mode_ = PhysicsMode::Nonlinear;
    GameStatus status_ = GameStatus::Flying;
    bool sas_ = false;

    int level_ = 0;
    double t_ = 0.0;
    double hold_ = 0.0, bestHold_ = 0.0, holdRequired_ = 20.0;
    double targetH_ = 500.0, bandH_ = 25.0;
    double targetV_ = 41.0,  bandV_ = 4.0;
    bool inBand_ = false;
    double stallTimer_ = 0.0;
    std::string msg_;

    std::deque<Sample> hist_;
    double histSpan_ = 60.0;
};

}  // namespace fs
