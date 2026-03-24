# Data Model: ロボットリアルタイム経路生成・迷路走破シミュレータ

**Branch**: `001-robot-maze-simulator` | **Date**: 2026-03-24

---

## エンティティ一覧

### 1. `CellType` ? セル種別（列挙型）

| 値 | 説明 |
|----|------|
| `Wall` | 壁セル |
| `Passage` | 通路セル |

グリッド上の物理的な構造を表す。ロボットの保有マップでは後述の `KnownCell` を使用する。

---

### 2. `GridPos` ? グリッド座標（値型）

| フィールド | 型 | 説明 |
|-----------|-----|------|
| `col` | `u16` | 列インデックス（0 ? 239） |
| `row` | `u16` | 行インデックス（0 ? 239） |

- グリッドサイズ: 240 × 240 セル
- 物理座標との変換: `x = (col as f32 - 120.0) * 0.05`, `y = (120.0 - row as f32) * 0.05`
- Y 軸: 行インデックスが増加するほど物理 Y 値は減少（画面下方向）

---

### 3. `WorldPos` ? 物理座標（値型）

| フィールド | 型 | 説明 |
|-----------|-----|------|
| `x` | `f32` | X 座標 [m]（左右、右=正） |
| `y` | `f32` | Y 座標 [m]（上下、上=正） |

- 迷路中央を原点 (0.0, 0.0) とする
- 最小単位: 0.05 m（= 1 セル）

---

### 4. `Maze` ? 迷路（集約ルート）

| フィールド | 型 | 説明 |
|-----------|-----|------|
| `grid` | `[[CellType; 240]; 240]` | 迷路グリッド |
| `start` | `GridPos` | 開始位置（中央: (120, 120)） |
| `goal` | `GridPos` | 目標位置（外周セル） |
| `seed` | `u64` | 生成に使用した乱数シード |

**制約**:
- `grid[start.row][start.col]` は必ず `Passage`
- `grid[goal.row][goal.col]` は必ず `Passage`（外周の開口部）
- 開始位置から目標位置へのルートがちょうど 1 本存在する（スパニングツリー保証）

---

### 5. `KnownCell` ? 保有マップのセル種別（列挙型）

| 値 | 説明 |
|----|------|
| `Unknown` | 未観測（グレー表示） |
| `FreeSpace` | 通路として認識済み |
| `Wall` | 壁として認識済み |

---

### 6. `KnownMap` ? ロボット保有マップ

| フィールド | 型 | 説明 |
|-----------|-----|------|
| `cells` | `[[KnownCell; 240]; 240]` | ロボットが蓄積した環境情報 |

**制約**:
- 初期状態は全セル `Unknown`
- LRF スキャン後に `FreeSpace` または `Wall` へ更新される
- 一度確定したセル種別は変更しない（静的環境を前提）

---

### 7. `LrfScan` ? LRF スキャンデータ（スナップショット）

| フィールド | 型 | 説明 |
|-----------|-----|------|
| `origin` | `WorldPos` | スキャン時のロボット中心座標 |
| `points` | `Vec<WorldPos>` | 取得された点群（最大 720 点: 360° / 0.5°） |
| `timestamp_ms` | `u64` | シミュレーション内経過時間 [ms] |

**制約**:
- 各レイは最初に衝突した壁セルの境界点で終端する（壁貫通なし）
- 障害物がない方向のレイは検出範囲 5 m で打ち切られる

---

### 8. `Path` ? 生成経路

| フィールド | 型 | 説明 |
|-----------|-----|------|
| `waypoints` | `Vec<WorldPos>` | 現在位置から目標までのウェイポイント列 |
| `is_to_frontier` | `bool` | フロンティアへのサブゴール経路か、ゴール直行かのフラグ |

**制約**:
- `waypoints[0]` はロボットの現在位置
- 隣接ウェイポイント間のセグメントはロボット幅 (0.5 m) を考慮した通行可能空間を保証する
- LRF 更新ごとに再生成される（旧データは破棄）

---

### 9. `Robot` ? ロボット状態

| フィールド | 型 | 説明 |
|-----------|-----|------|
| `position` | `WorldPos` | 現在の中心座標 |
| `velocity` | `(f32, f32)` | 進行方向の正規化ベクトル × 速度 [m/s] |
| `known_map` | `KnownMap` | ロボットが保有する環境マップ |
| `current_path` | `Option<Path>` | 現在生成中の経路（なければ `None`） |
| `state` | `RobotState` | 動作状態 |

---

### 10. `RobotState` ? ロボット動作状態（列挙型）

| 値 | 説明 |
|----|------|
| `Idle` | 「開始」前・静止中 |
| `Exploring` | 未知フロンティアへ向かっている |
| `NavigatingToGoal` | ゴールが既知領域に入り、直行中 |
| `Arrived` | 目標到達済み（自動停止） |

---

### 11. `SimulationState` ? シミュレーション全体状態

| フィールド | 型 | 説明 |
|-----------|-----|------|
| `maze` | `Maze` | 現在の迷路 |
| `robot` | `Robot` | ロボット状態 |
| `lrf_scan` | `Option<LrfScan>` | 最新の LRF スキャン結果 |
| `elapsed_ms` | `u64` | シミュレーション経過時間 [ms] |
| `seed` | `u64` | 現在のシミュレーションシード |

---

## 状態遷移

```
SimulatorLaunched
      │
      ▼
  [MazeGenerated]  ←─── RestartButton
      │
  StartButton
      │
      ▼
  RobotState::Idle
      │  (開始ボタン)
      ▼
  RobotState::Exploring
      │  (ゴールが既知マップに入る)
      ▼
  RobotState::NavigatingToGoal
      │  (|pos - goal| ? 0.05 m)
      ▼
  RobotState::Arrived  → 自動停止・ロボット色変更（淡い赤→淡い青）
```

---

## エンティティ関係図

```
SimulationState
├── Maze
│   ├── grid: [[CellType; 240]; 240]
│   ├── start: GridPos
│   └── goal: GridPos
└── Robot
    ├── position: WorldPos
    ├── velocity: (f32, f32)
    ├── known_map: KnownMap
    │   └── cells: [[KnownCell; 240]; 240]
    ├── current_path: Option<Path>
    │   └── waypoints: Vec<WorldPos>
    └── state: RobotState

LrfScan (毎フレーム生成・破棄)
├── origin: WorldPos
└── points: Vec<WorldPos>
```
