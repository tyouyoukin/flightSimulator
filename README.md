# FlightSim

飛行力学に基づいた飛行シミュレーションゲーム。機体パラメータ（翼形状・位置・重心など）をGUIで設計し、リアルタイムで飛行特性を確認・操縦できる。

## 要件

- CMake 3.16 以上
- g++ / clang++ （C++17対応）
- macOS：Xcode Command Line Tools
  ```
  xcode-select --install
  ```
- OpenGL（macOS標準搭載）

## ビルド手順

```bash
git clone <repo-url>
cd flightSim
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
./build/FlightSim
```

デバッグビルド：

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j$(nproc)
```

## ブランチ戦略

| ブランチ | 役割 |
| --- | --- |
| `main` | 安定版。動作確認済みのリリース |
| `develop` | 開発統合ブランチ |
| `feature/*` | 機能単位の開発ブランチ |

```
feature/xxx  →  develop  →  main
```

## 実装フェーズ

詳細は [REQUIREMENTS.md](REQUIREMENTS.md) を参照。

- **Phase 1**：2D縦断面シミュレーター（平板モデル）
- **Phase 2**：NACA翼型データ・動安定解析
- **Phase 3**：3D拡張（6-DOF）
- **Phase 4**：ゲーム化

## 依存ライブラリ（CMake FetchContent で自動取得）

| ライブラリ | 用途 |
| --- | --- |
| GLFW | ウィンドウ・入力管理 |
| Dear ImGui | GUI |
| ImPlot | グラフ描画 |
| Eigen | 線形代数・固有値解析 |
| Google Test | 単体テスト |
