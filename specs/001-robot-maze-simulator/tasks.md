---
description: "Task list for robot maze simulator implementation"
---

# Tasks: ロボットリアルタイム経路生成・迷路走破シミュレータ

**Input**: Design documents from `/specs/001-robot-maze-simulator/`  
**Prerequisites**: plan.md ?, spec.md ?, research.md ?, data-model.md ?, contracts/ ?

**Constitution**: TDD 必須（III. Test-First）。各タスクは実装前にテストを記述すること。

## Format: `[ID] [P?] [Story] Description`

- **[P]**: 並列実行可能（他タスクとファイルが独立）
- **[Story]**: 対応するユーザーストーリー（US1?US4）
- 各タスクにファイルパスを明記

---

## Phase 1: Setup（プロジェクト初期化）

**Purpose**: Cargo プロジェクト作成・依存設定・CI 基盤

- [x] T001 `cargo init` でプロジェクトを初期化し、`Cargo.toml` に `rust-version`（MSRV）を宣言する
- [x] T002 `Cargo.toml` に依存クレートを追加する（`egui`・`eframe`・`rand`・`rand_chacha`・`ordered-float`）
- [x] T003 [P] `rustfmt.toml` を作成しフォーマット設定を記述する
- [x] T004 [P] `.github/workflows/ci.yml` を作成し `cargo fmt --check`・`cargo clippy -- -D warnings`・`cargo test`・`cargo audit` を CI ステップに定義する
- [x] T005 `src/` 以下のモジュール宣言（`maze/`・`sensor/`・`planner/`・`robot/`・`simulation/`・`ui/`）を `src/main.rs` に追加し、空の `mod.rs` をそれぞれ作成する
- [x] T006 `tests/` ディレクトリを作成し、`maze_test.rs`・`sensor_test.rs`・`planner_test.rs`・`simulation_test.rs` の空ファイルを配置する
- [x] T007 `benches/` ディレクトリを作成し、`lrf_bench.rs`・`planner_bench.rs` の空ファイルを配置する（`Cargo.toml` に `[[bench]]` エントリも追記）

---

## Phase 2: Foundational（共通型定義・座標変換）

**Purpose**: 全ユーザーストーリーが依存する共通型・変換関数を先行実装する

?? **CRITICAL**: このフェーズが完了するまで US1?US4 の実装を開始できない

- [x] T008 `src/maze/types.rs` に `CellType`・`GridPos`・`WorldPos` を定義し、`GridPos ? WorldPos` 変換関数を実装する
- [x] T009 `tests/maze_test.rs` に T008 の変換関数ユニットテストを記述する（原点・端点・往復変換の検証）
- [x] T010 `src/robot/types.rs` に `KnownCell`・`KnownMap`・`RobotState`・`Path`・`Robot` を定義する
- [x] T011 `src/sensor/lrf.rs` に `LrfScan` 型を定義する
- [x] T012 `src/simulation/engine.rs` に `SimulationState` 型の骨格（フィールド定義のみ）を作成する

**Checkpoint**: 共通型が `cargo check` でエラーなしにコンパイルされること

---

## Phase 3: US1 ? 迷路の自動生成と探索開始（Priority: P1）? MVP

**Goal**: ランダム迷路を生成し、「開始」ボタンでロボットが動き出す最小動作ループを確立する

**Independent Test**: `cargo test maze` で迷路生成テストが全パスし、開始からゴールへの唯一ルートが自動検証される

- [x] T013 [US1] `tests/maze_test.rs` に迷路生成の性質テストを記述する（シード再現性・ルート唯一性・外周開口部・スタート/ゴールが Passage の検証）
- [x] T014 [US1] `src/maze/generator.rs` に Recursive Backtracking アルゴリズムを実装する（`ChaCha8Rng` シード使用、240×240 グリッド対応）
- [x] T015 [US1] `src/maze/generator.rs` に外周開口部のゴールセル選択ロジックを実装する（外周 Passage セルをランダムに 1 つ選択）
- [x] T016 [US1] `src/maze/mod.rs` に `MazeGenerator::generate(seed: u64) -> Maze` と `Maze::is_passable_for_robot(pos: GridPos) -> bool` を公開する
- [x] T017 [US1] `src/simulation/engine.rs` に `Simulation::restart(seed: Option<u64>) -> Simulation` を実装する（迷路生成・ロボット初期状態リセット）
- [x] T018 [US1] `src/main.rs` にコマンドライン引数（`--seed`）パースを実装し、`Simulation::restart` へシードを渡す
- [x] T019 [US1] `tests/simulation_test.rs` に `restart` のユニットテストを記述する（初期状態検証・シード再現性）

---

## Phase 4: US2 ? LRF によるリアルタイムマップ更新（Priority: P2）

**Goal**: ロボットが LRF でスキャンし、保有マップをリアルタイムで更新する

**Independent Test**: `cargo test sensor` が全パスし、探索中にロボット保有マップが逐次更新されることを単体テストで検証できる

- [x] T020 [US2] `tests/sensor_test.rs` に LRF テストを記述する（壁検出・貫通なし・5 m 打ち切り・720 点以下・原点一致の検証）
- [x] T021 [US2] `src/sensor/lrf.rs` に DDA レイキャスティングを実装する（`Lrf::scan(robot_pos: WorldPos, maze: &Maze) -> LrfScan`）
- [x] T022 [US2] `src/robot/controller.rs` に `Robot::update_map(scan: &LrfScan)` を実装する（FreeSpace/Wall セルへの更新・既存エントリ不変）
- [x] T023 [US2] `tests/sensor_test.rs` に `update_map` のテストを追記する（観測済み領域の KnownCell 種別、未観測領域が Unknown のまま）
- [x] T024 [P] [US2] `benches/lrf_bench.rs` に LRF スキャン性能ベンチマークを実装する（目標: 240×240 グリッドで 20 ms 以内）

---

## Phase 5: US3 ? 経路生成とロボット移動（Priority: P3）

**Goal**: 保有マップを参照して衝突なしの経路を生成し、ロボットを目標へ向かわせる

**Independent Test**: `cargo test planner` が全パスし、生成経路がロボット幅（0.5 m）の clearance を持ち壁に接触しないことが検証される

- [x] T025 [US3] `tests/planner_test.rs` に A\* テストを記述する（最短経路・通路幅保証・到達不可時 None・ウェイポイント先頭が現在位置）
- [x] T026 [US3] `src/planner/astar.rs` に A\* アルゴリズムを実装する（`Planner::plan(robot_pos, goal, known_map) -> Option<Path>`、ヒューリスティック=ユークリッド距離）
- [x] T027 [US3] `src/planner/frontier.rs` に最近傍フロンティア探索を実装する（`FrontierFinder::nearest_frontier(robot_pos, known_map) -> Option<WorldPos>`、BFS 使用）
- [x] T028 [US3] `src/robot/controller.rs` に `Robot::update(delta_ms: u64, path: &Option<Path>)` を実装する（1 m/s 速度更新・到達判定 ? 0.05 m・RobotState 遷移）
- [x] T029 [US3] `tests/planner_test.rs` に `FrontierFinder` テストを追記する（フロンティアセル選定・全既知探索済時 None）
- [x] T030 [US3] `src/simulation/engine.rs` に `Simulation::tick(delta_ms: u64)` を実装する（LRF スキャン→マップ更新→経路再生成→位置更新の 1 サイクル）
- [x] T031 [US3] `tests/simulation_test.rs` に `tick` の統合テストを追記する（到達判定・RobotState::Arrived 遷移タtick 後に位置が更新されていること）
- [x] T032 [P] [US3] `benches/planner_bench.rs` に A\* 経路生成性能ベンチマークを実装する（目標: 240×240 グリッドで 20 ms 以内）

---

## Phase 6: US4 ? UI によるシミュレーション可視化（Priority: P4）

**Goal**: egui で全情報を重畳表示し、「開始」「再起動」ボタンが正しく機能する

**Independent Test**: アプリを起動してボタン操作のみで探索→再起動サイクルが完結することを手動確認できる

- [x] T033 [US4] `src/ui/app.rs` に `eframe::App` を実装するスケルトンを作成し、`src/main.rs` から `eframe::run_native` で起動できるようにする
- [x] T034 [US4] `src/ui/app.rs` に迷路マップ描画（壁=黒・通路=白）を `egui::Painter` で実装する
- [x] T035 [P] [US4] `src/ui/app.rs` にロボット保有マップ描画（未知=グレー・既知通路=白・既知壁=黒、半透明重畳）を実装する
- [x] T036 [P] [US4] `src/ui/app.rs` に LRF 点群描画（淡い緑の点群）を実装する
- [x] T037 [P] [US4] `src/ui/app.rs` に生成経路描画（赤のポリライン）を実装する
- [x] T038 [P] [US4] `src/ui/app.rs` にロボット本体描画（0.5 m×0.5 m 矩形、到達前=淡い赤枚・到達後=淡い青枚）を実装する
- [x] T039 [P] [US4] `src/ui/app.rs` に目標位置マーカー（橙）と進行方向ベクトル矢印を実装する
- [x] T040 [US4] `src/ui/app.rs` に「開始」ボタンを実装する（`Idle` 状態でのみ有効、探索中はグレーアウト、クリックで `tick` ループ開始）
- [x] T041 [US4] `src/ui/app.rs` に「再起動」ボタンを実装する（常時有効、クリックで `Simulation::restart` 呼び出し・UI 初期化）
- [x] T042 [US4] `src/ui/app.rs` に `RobotState::Arrived` 時のロボット枚色変更（淡い赤→淡い青）と自動停止ロジックを実装する
- [x] T043 [US4] `src/simulation/engine.rs` の `tick` ループを `eframe` の更新サイクルに統合し、シミュレーション時間（`elapsed_ms`）と実時間（`delta_ms` = フレーム間隔）を同期させる

---

## Phase 7: Polish（最終仕上げ・クロスカッティング）

**Purpose**: CI 通過確認・audit・ドキュメント最終化

- [x] T044 `cargo fmt`・`cargo clippy -- -D warnings` を実行しすべての警告・エラーを解消する
- [ ] T045 `cargo audit` を実行し依存クレートに既知の脆弱性がないことを確認する
- [x] T046 `cargo test` を実行し全テストがグリーンであることを確認する
- [x] T047 [P] `cargo bench` を実行し LRF スキャン・A\* 経路生成が各 20 ms 以内であることを確認する
- [x] T048 [P] `cargo doc` を実行しドキュメントビルドが警告なしで完了することを確認する（公開 API に `///` コメント追加）
- [x] T049 [P] `README.md` を作成しプロジェクト概要・ビルド手順・シード指定方法を記述する

---

## 依存関係グラフ

```
Phase 1 (Setup)
    │
    ▼
Phase 2 (Foundational: 共通型)
    │
    ├─? Phase 3 (US1: 迷路生成)
    │       │
    │       ▼
    │   Phase 4 (US2: LRF + マップ更新)
    │       │
    │       ▼
    │   Phase 5 (US3: 経路生成 + 移動)
    │       │
    │       ▼
    │   Phase 6 (US4: UI 可視化)
    │
    └─? Phase 7 (Polish) ← Phase 6 完了後
```

US3 は US2（マップ更新）に依存する。US4 は US1?US3 すべてに依存する。

---

## 並列実行例

```
# US2 完了後、US3・ベンチマーク・US4 スケルトンを並列開始可能
T025 (US3 テスト記述)  ┐
T026 (A* 実装)         ├── 並列
T024 (LRF ベンチ)      ┘

# US3 完了後
T034 (迷路描画)   ┐
T035 (保有マップ) │
T036 (LRF 描画)  ├── 並列（独立ファイルに分離済みなら可）
T037 (経路描画)  │
T038 (ロボット)  ┘
```

---

## 実装戦略（MVP ファースト）

| スコープ | フェーズ | 検証方法 |
|---------|---------|---------|
| **MVP** | Phase 1?3 (T001?T019) | `cargo test maze` + `cargo run` で迷路表示を確認 |
| **コア機能** | + Phase 4?5 (T020?T032) | `cargo test` 全パス + ロボットが自律移動 |
| **完成版** | + Phase 6?7 (T033?T049) | UI 全表示 + CI グリーン |

---

## サマリー

| 項目 | 数 |
|------|----|
| 総タスク数 | 49 |
| Phase 1 (Setup) | 7 |
| Phase 2 (Foundational) | 5 |
| Phase 3 (US1) | 7 |
| Phase 4 (US2) | 5 |
| Phase 5 (US3) | 8 |
| Phase 6 (US4) | 11 |
| Phase 7 (Polish) | 6 |
| 並列実行可能タスク [P] | 20 |
