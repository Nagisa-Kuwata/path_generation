# Module Contracts: ロボットリアルタイム経路生成・迷路走破シミュレータ

**Branch**: `001-robot-maze-simulator` | **Date**: 2026-03-24

本ドキュメントはアプリケーションを構成する各モジュールの公開インターフェース契約を定義する。  
実装言語は Rust であり、型シグネチャは Rust の疑似コードで表現する。

---

## モジュール構成

```
src/
├── maze/          # 迷路生成・迷路データ管理
├── robot/         # ロボット状態・移動制御
├── sensor/        # LRF シミュレーション
├── planner/       # 経路計画（A* + フロンティア探索）
├── simulation/    # シミュレーションループ・状態管理
└── ui/            # egui 描画・ボタン制御
```

---

## Contract 1: `maze` モジュール

### `MazeGenerator::generate(seed: u64) -> Maze`

- **事前条件**: なし
- **事後条件**:
  - 返却された `Maze` の `grid` は 240×240 セルで構成される
  - `start` セルと `goal` セルは必ず `Passage`
  - `goal` は外周（行 0、行 239、列 0、列 239 のいずれか）の `Passage` セル
  - 開始位置から目標位置へのルートがちょうど 1 本存在する（スパニングツリー保証）
  - 同一 `seed` に対して常に同じ `Maze` を返す（決定論的）
- **エラー**: パニック不可。`seed` によらず常に有効な迷路を返す

### `Maze::is_passable_for_robot(pos: GridPos) -> bool`

- ロボット中心 `pos` にロボット（10×10 セル）を配置したとき、全セルが `Passage` であれば `true`

---

## Contract 2: `sensor` モジュール

### `Lrf::scan(robot_pos: WorldPos, maze: &Maze) -> LrfScan`

- **事前条件**: `robot_pos` は迷路内の有効な通路座標
- **事後条件**:
  - `points` の要素数 ? 720（360° / 0.5° = 720 方向）
  - 各点はロボット中心から壁境界まで、または 5 m 以内の最初の衝突点
  - 壁を貫通する点は含まない
  - `origin` == `robot_pos`
- **エラー**: パニック不可

---

## Contract 3: `planner` モジュール

### `Planner::plan(robot_pos: WorldPos, goal: GridPos, known_map: &KnownMap) -> Option<Path>`

- **事前条件**: `robot_pos` は `KnownMap` 上の `FreeSpace` 座標
- **事後条件**:
  - `Some(Path)` の場合:
    - `waypoints[0]` は `robot_pos` に一致（誤差 ? 0.05 m）
    - 全ウェイポイント間セグメントはロボット幅（0.5 m）分の clearance を持つ
    - 経路はすべて `FreeSpace` セル上を通過する
  - `None` の場合: 現在の既知マップ上でゴール（またはフロンティア）へ到達不可能
- **エラー**: パニック不可

### `FrontierFinder::nearest_frontier(robot_pos: WorldPos, known_map: &KnownMap) -> Option<WorldPos>`

- **事前条件**: なし
- **事後条件**:
  - `Some(pos)` の場合: `pos` は `FreeSpace` セルに隣接する `Unknown` セルの中心座標
  - `None` の場合: フロンティアセルが存在しない（全既知領域が探索済み）
- **エラー**: パニック不可

---

## Contract 4: `robot` モジュール

### `Robot::update(delta_ms: u64, path: &Option<Path>)`

- **事前条件**: `delta_ms` > 0
- **事後条件**:
  - `state` が `Arrived` 以外の場合、`position` を速度 1 m/s × `delta_ms` ms 分だけ更新する
  - `state` が `Arrived` の場合、`position` は変化しない
  - `|position - goal| ? 0.05` のとき `state` を `Arrived` へ遷移させる
- **エラー**: パニック不可

### `Robot::update_map(scan: &LrfScan)`

- **事前条件**: なし
- **事後条件**:
  - `scan.points` の各点をレイトレースし、レイ上のセルを `FreeSpace`、命中点のセルを `Wall` へ更新する
  - 既存の `FreeSpace` / `Wall` エントリは上書きしない（静的環境前提）
- **エラー**: パニック不可

---

## Contract 5: `simulation` モジュール

### `Simulation::tick(delta_ms: u64)`

1. `Lrf::scan` でスキャンを取得
2. `Robot::update_map(scan)` でマップ更新
3. `Planner::plan(...)` で経路再生成
4. `Robot::update(delta_ms, path)` で位置更新
5. `elapsed_ms += delta_ms`

- **事前条件**: `robot.state` が `Arrived` でないこと（到達後は呼び出し無効）
- **事後条件**: 上記 5 ステップがすべて完了している
- **エラー**: パニック不可

### `Simulation::restart(seed: Option<u64>) -> Simulation`

- **事前条件**: なし
- **事後条件**:
  - 新しい `Maze` が生成される（`seed` 指定時は決定論的、`None` 時はランダムシード）
  - `Robot` は初期状態（位置=迷路中央、既知マップ全`Unknown`、`state=Idle`）にリセット
  - `elapsed_ms` = 0
- **エラー**: パニック不可

---

## Contract 6: `ui` モジュール

### `AppState` のボタン制御規則

| ボタン | 有効条件 | 押下時の動作 |
|--------|---------|------------|
| 「開始」 | `robot.state == Idle` かつ `robot.state != Arrived` | `Simulation::tick` ループ開始 |
| 「再起動」 | 常時 | `Simulation::restart(seed)` 呼び出し → UI 初期化 |

### 描画順序（重畳表示）

1. 実際の迷路マップ（壁=黒、通路=白）
2. ロボット保有マップ（未知=グレー、既知通路=白、既知壁=黒）※半透明重畳
3. LRF 点群（淡い緑の点）
4. 生成経路（赤のライン）
5. ロボット本体（0.5 m×0.5 m の矩形、到達前=淡い赤枠、到達後=淡い青枠）
6. 目標位置マーカー（橙）
7. 進行方向ベクトル（ロボット中心からの矢印）
