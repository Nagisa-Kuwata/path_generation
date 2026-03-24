use std::cmp::Reverse;
use std::collections::{BinaryHeap, HashMap};

use ordered_float::OrderedFloat;

use crate::maze::{GRID_SIZE, GridPos, WorldPos};
use crate::robot::types::{KnownCell, KnownMap, Path};

/// A* path planner operating on the robot's [`KnownMap`].
///
/// Operates on **logical cells** (even grid indices only).
/// Each logical cell `(lc, lr)` maps to physical cell `(lc*2, lr*2)`.
/// Movement between two adjacent logical cells is allowed only when the
/// wall cell between them is not a confirmed Wall.
pub struct Planner;

// Number of logical cells per side (240 / 2 = 120).
const LCOLS: usize = GRID_SIZE / 2;
const LROWS: usize = GRID_SIZE / 2;

impl Planner {
    /// Compute the shortest path from `robot_pos` to `goal` on `known_map`.
    ///
    /// Returns `None` if the goal is confirmed unreachable.
    pub fn plan(robot_pos: WorldPos, goal: GridPos, known_map: &KnownMap) -> Option<Path> {
        // Convert physical grid pos to logical cell.
        let phys_start = GridPos::from(robot_pos).snap_even();
        // Already even after snap.
        let start_lc = (phys_start.col as usize / 2).min(LCOLS - 1);
        let start_lr = (phys_start.row as usize / 2).min(LROWS - 1);

        let goal_lc = ((goal.col as usize).min(GRID_SIZE - 1) / 2).min(LCOLS - 1);
        let goal_lr = ((goal.row as usize).min(GRID_SIZE - 1) / 2).min(LROWS - 1);

        if start_lc == goal_lc && start_lr == goal_lr {
            return Some(Path {
                waypoints: vec![robot_pos],
                is_to_frontier: false,
            });
        }

        let sk = (start_lc, start_lr);
        let gk = (goal_lc, goal_lr);

        let mut open: BinaryHeap<Reverse<(OrderedFloat<f32>, usize, usize)>> = BinaryHeap::new();
        let mut g_cost: HashMap<(usize, usize), f32> = HashMap::new();
        let mut came_from: HashMap<(usize, usize), (usize, usize)> = HashMap::new();

        g_cost.insert(sk, 0.0);
        open.push(Reverse((
            OrderedFloat(lheuristic(start_lc, start_lr, goal_lc, goal_lr)),
            start_lc,
            start_lr,
        )));

        while let Some(Reverse((_, lc, lr))) = open.pop() {
            if lc == goal_lc && lr == goal_lr {
                return Some(lreconstruct(sk, gk, &came_from, robot_pos));
            }
            let current_g = *g_cost.get(&(lc, lr)).unwrap_or(&f32::MAX);

            for (nlc, nlr) in logical_neighbors(lc, lr) {
                // The wall cell between (lc, lr) and (nlc, nlr).
                let wall_c = lc + nlc; // lc*2 + (nlc-lc) = wall physical col
                let wall_r = lr + nlr;
                if !is_passable_wall(wall_c, wall_r, known_map) {
                    continue;
                }
                let new_g = current_g + 1.0;
                let nk = (nlc, nlr);
                if new_g < *g_cost.get(&nk).unwrap_or(&f32::MAX) {
                    g_cost.insert(nk, new_g);
                    came_from.insert(nk, (lc, lr));
                    let f = new_g + lheuristic(nlc, nlr, goal_lc, goal_lr);
                    open.push(Reverse((OrderedFloat(f), nlc, nlr)));
                }
            }
        }

        None
    }
}

/// Euclidean heuristic in logical-cell space.
fn lheuristic(lc: usize, lr: usize, glc: usize, glr: usize) -> f32 {
    let dc = (lc as f32 - glc as f32).abs();
    let dr = (lr as f32 - glr as f32).abs();
    (dc * dc + dr * dr).sqrt()
}

/// 4-connected logical-cell neighbors.
fn logical_neighbors(lc: usize, lr: usize) -> Vec<(usize, usize)> {
    let mut result = Vec::with_capacity(4);
    if lc > 0 { result.push((lc - 1, lr)); }
    if lc + 1 < LCOLS { result.push((lc + 1, lr)); }
    if lr > 0 { result.push((lc, lr - 1)); }
    if lr + 1 < LROWS { result.push((lc, lr + 1)); }
    result
}

/// The wall cell between two adjacent logical cells.
/// `wall_c = lc + nlc` and `wall_r = lr + nlr` because the physical
/// coordinates are `lc*2` and `nlc*2`, so the dividing cell is
/// `lc*2 + 1 = lc + nlc` when `nlc = lc + 1`.
/// A cell is passable if it is not a confirmed Wall (Unknown = optimistic).
fn is_passable_wall(wall_c: usize, wall_r: usize, known_map: &KnownMap) -> bool {
    if wall_c >= GRID_SIZE || wall_r >= GRID_SIZE {
        return false;
    }
    known_map.cells[wall_r][wall_c] != KnownCell::Wall
}

/// Reconstruct path, converting logical cells back to physical WorldPos.
/// The first waypoint is the robot's current world position to avoid snapping.
/// Between each pair of consecutive logical cells, the connector (wall/passage)
/// cell at the odd index is inserted so the robot always travels axis-aligned
/// through that cell rather than cutting diagonally across an (odd,odd) corner.
fn lreconstruct(
    start: (usize, usize),
    goal: (usize, usize),
    came_from: &HashMap<(usize, usize), (usize, usize)>,
    robot_pos: WorldPos,
) -> Path {
    let mut cells = Vec::new();
    let mut cur = goal;
    loop {
        cells.push(cur);
        if cur == start {
            break;
        }
        match came_from.get(&cur) {
            Some(&prev) => cur = prev,
            None => break,
        }
    }
    cells.reverse();

    // Build waypoints from the start logical cell onward.
    // We intentionally do NOT insert robot_pos as the first waypoint,
    // because the robot may currently sit on a connector cell (odd,even) that
    // is within ARRIVAL_THRESHOLD of the planned connector/passage waypoints,
    // which would cause every waypoint to be skipped and the robot to freeze.
    // Instead, the first waypoint is the snapped even-cell (start logical cell),
    // ensuring the robot always moves to a passage cell before following the path.
    let _ = robot_pos; // kept in signature for API compatibility
    let mut waypoints = Vec::new();
    // First waypoint: the start even-passage cell itself (snap destination).
    if let Some(&(slc, slr)) = cells.first() {
        waypoints.push(WorldPos::from(GridPos {
            col: (slc * 2) as u16,
            row: (slr * 2) as u16,
        }));
    }
    for window in cells.windows(2) {
        let (lc0, lr0) = window[0];
        let (lc1, lr1) = window[1];
        // Connector cell sits at the average of the two physical coordinates.
        let conn_col = (lc0 + lc1) as u16; // = lc0*2 + (lc1 - lc0) step = lc0+lc1
        let conn_row = (lr0 + lr1) as u16;
        waypoints.push(WorldPos::from(GridPos {
            col: conn_col,
            row: conn_row,
        }));
        // Destination even cell.
        waypoints.push(WorldPos::from(GridPos {
            col: (lc1 * 2) as u16,
            row: (lr1 * 2) as u16,
        }));
    }

    Path {
        waypoints,
        is_to_frontier: false,
    }
}
