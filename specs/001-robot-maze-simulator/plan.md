# Implementation Plan: ロボットリアルタイム経路生成・迷路走破シミュレータ

**Branch**: `001-robot-maze-simulator` | **Date**: 2026-03-24 | **Spec**: [spec.md](spec.md)  
**Input**: Feature specification from `/specs/001-robot-maze-simulator/spec.md`

## Summary

12 m×12 m のランダム迷路（再現可能）を生成し、LRF センサで未知環境を逐次探索しながら  
リアルタイムに経路を生成・更新するロボット走破シミュレータ。  
技術スタック: Rust stable + egui/eframe（GUI）、A\* 経路生成、DDA レイキャスティング、  
Recursive Backtracking 迷路生成。

## Technical Context

**Language/Version**: Rust stable（MSRV を `Cargo.toml` の `rust-version` で宣言）  
**Primary Dependencies**: `egui` + `eframe`（UI）、`rand` + `rand_chacha`（乱数）、`ordered-float`（A\* 優先キュー）  
**Storage**: N/A（ファイル永続化なし。シードは起動引数で指定）  
**Testing**: `cargo test`（ユニット・インテグレーション）、`cargo bench`（経路生成・LRF性能）  
**Target Platform**: Windows / Linux / macOS デスクトップ（ローカル実行）  
**Project Type**: デスクトップアプリケーション（シングルバイナリ）  
**Performance Goals**: LRF 50 Hz（20 ms/cycle）、経路再生成 50 Hz、UI 30 fps  
**Constraints**: ロボット 1 m/s 実時間、迷路 240×240 グリッド、シングルスレッド許容  
**Scale/Scope**: シングルユーザー・ローカル。ネットワーク不要

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| 原則 | 確認 | 根拠 |
|------|------|------|
| I. Rust-First | ✅ | 全ソースを Rust stable で実装 |
| II. Safety & Correctness | ✅ | ライブラリ関数は `Result`/`Option` 返却。`unsafe` は使用しない方針 |
| III. Test-First | ✅ | 各モジュールのテストを実装前に記述（TDD） |
| IV. Performance-Aware Design | ✅ | A\*・DDA の計算量を事前分析済み（research.md）。ベンチマークを添付する |
| V. Simplicity & Minimal Dependencies | ✅ | 依存は `egui/eframe`・`rand/rand_chacha`・`ordered-float` のみ。`cargo audit` を CI に組み込む |

**Post-design re-check**: Phase 1 完了後も全原則の違反なし。`unsafe` ブロック不使用。

## Project Structure

### Documentation (this feature)

```text
specs/001-robot-maze-simulator/
├── plan.md              # このファイル
├── research.md          # Phase 0 出力
├── data-model.md        # Phase 1 出力
├── quickstart.md        # Phase 1 出力
├── contracts/
│   └── module-contracts.md   # Phase 1 出力
└── tasks.md             # Phase 2 出力 (/speckit.tasks コマンド)
```

### Source Code (repository root)

```text
src/
├── main.rs              # エントリポイント（引数解析・eframe 起動）
├── maze/
│   ├── mod.rs
│   ├── types.rs         # CellType, GridPos, Maze
│   └── generator.rs     # Recursive Backtracking 迷路生成
├── sensor/
│   ├── mod.rs
│   └── lrf.rs           # DDA レイキャスティング → LrfScan
├── planner/
│   ├── mod.rs
│   ├── astar.rs         # A* 経路計画
│   └── frontier.rs      # 最近傍フロンティア探索
├── robot/
│   ├── mod.rs
│   ├── types.rs         # Robot, RobotState, KnownMap, KnownCell, Path
│   └── controller.rs    # 位置更新・マップ更新・状態遷移
├── simulation/
│   ├── mod.rs
│   └── engine.rs        # SimulationState, tick(), restart()
└── ui/
    ├── mod.rs
    └── app.rs           # egui App 実装（描画・ボタン制御）

tests/
├── maze_test.rs         # 迷路生成の性質テスト
├── sensor_test.rs       # LRF レイキャスティングテスト
├── planner_test.rs      # A* 経路生成テスト
└── simulation_test.rs   # 統合テスト（tick ループ）

benches/
├── lrf_bench.rs         # LRF スキャン性能
└── planner_bench.rs     # 経路生成性能
```

**Structure Decision**: シングルプロジェクト（Cargo workspace 不使用）。  
将来マルチクレート化が必要になった際に workspace へ移行する。

## Complexity Tracking

> 憲法違反なし。追記事項なし。
