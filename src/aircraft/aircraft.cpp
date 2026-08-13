#include "aircraft.h"

#include <cstdio>

namespace fs {

namespace {
struct Preset {
    const char* name;
    double ballastXFrac;  // バラスト位置 / 翼根弦長
    double washout;       // 翼端ねじり下げ [deg]
};

// バラストを後ろへ動かすほど重心が後退し，静安定余裕 Kn が減る
const Preset kPresets[] = {
    {"LEVEL 1  nose ballast",      0.04, -5.0},
    {"LEVEL 2  forward ballast",   0.11, -4.5},
    {"LEVEL 3  mid ballast",       0.17, -4.0},
    {"LEVEL 4  aft ballast",       0.23, -3.5},
    {"LEVEL 5  far aft  (UNSTABLE)", 0.29, -3.0},
};
constexpr int kNumPresets = sizeof(kPresets) / sizeof(kPresets[0]);

int clampLevel(int l) { return l < 0 ? 0 : (l >= kNumPresets ? kNumPresets - 1 : l); }
}  // namespace

int numLevels() { return kNumPresets; }

const char* levelName(int level) { return kPresets[clampLevel(level)].name; }

WingDesign makeDesign(int level) {
    const Preset& p = kPresets[clampLevel(level)];
    WingDesign d;
    d.ballastXFrac = p.ballastXFrac;
    d.washout = p.washout;
    return d;
}

Aircraft makeAircraft(int level) {
    Aircraft ac = buildAircraft(makeDesign(level));
    static char buf[64];
    std::snprintf(buf, sizeof(buf), "Tailless Wing  L%d", clampLevel(level) + 1);
    ac.name = buf;
    return ac;
}

}  // namespace fs
