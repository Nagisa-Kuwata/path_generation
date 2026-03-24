use std::cmp::Reverse;
use std::collections::{BinaryHeap, HashMap};

use ordered_float::OrderedFloat;

use crate::maze::{GRID_SIZE, GridPos, WorldPos};
use crate::robot::types::{KnownCell, KnownMap, Path};

/// A* path planner operating on the robot's [`KnownMap`].
///
/// Unknown cells are treated optimistically as passable to allow exploration
/// into unseen areas. Uses 8-connectivity with Euclidean distance heuristic.
pub struct Planner;

impl Planner {
    /// Compute the shortest path from `robot_pos` to `goal` on `known_map`.
    ///
    /// Returns `None` if the goal is confirmed unreachable (surrounded by known walls).
    pub fn plan(robot_pos: WorldPos, goal: GridPos, known_map: &KnownMap) -> Option<Path> {
        let start = GridPos::from(robot_pos);

        if start.col == goal.col && start.row == goal.row {
            return Some(Path {
                waypoints: vec![robot_pos],
                is_to_frontier: false,
            });
        }

        // Min-heap keyed by f = g + h.
        let mut open: BinaryHeap<Reverse<(OrderedFloat<f32>, u16, u16)>> = BinaryHeap::new();
        // g_cost[(col, row)] = best cost from start so far.
        let mut g_cost: HashMap<(u16, u16), f32> = HashMap::new();
        let mut came_from: HashMap<(u16, u16), (u16, u16)> = HashMap::new();

        let sk = (start.col, start.row);
        g_cost.insert(sk, 0.0);
        open.push(Reverse((
            OrderedFloat(heuristic(start, goal)),
            start.col,
            start.row,
        )));

        while let Some(Reverse((_, col, row))) = open.pop() {
            if col == goal.col && row == goal.row {
                return Some(reconstruct(sk, (col, row), &came_from));
            }
            let current_g = *g_cost.get(&(col, row)).unwrap_or(&f32::MAX);

            for (nc, nr, cost) in neighbors(col, row) {
                if !is_passable(nc, nr, known_map) {
                    continue;
                }
                let new_g = current_g + cost;
                let nk = (nc, nr);
                if new_g < *g_cost.get(&nk).unwrap_or(&f32::MAX) {
                    g_cost.insert(nk, new_g);
                    came_from.insert(nk, (col, row));
                    let f = new_g + heuristic(GridPos { col: nc, row: nr }, goal);
                    open.push(Reverse((OrderedFloat(f), nc, nr)));
                }
            }
        }

        None
    }
}

/// Euclidean distance heuristic.
fn heuristic(pos: GridPos, goal: GridPos) -> f32 {
    let dc = (pos.col as f32 - goal.col as f32).abs();
    let dr = (pos.row as f32 - goal.row as f32).abs();
    (dc * dc + dr * dr).sqrt()
}

/// 8-connected neighbors with move cost (cardinal=1.0, diagonal=sqrt(2)).
fn neighbors(col: u16, row: u16) -> Vec<(u16, u16, f32)> {
    let mut result = Vec::with_capacity(8);
    let c = col as i32;
    let r = row as i32;
    for dc in -1..=1_i32 {
        for dr in -1..=1_i32 {
            if dc == 0 && dr == 0 {
                continue;
            }
            let nc = c + dc;
            let nr = r + dr;
            if nc >= 0 && nr >= 0 && nc < GRID_SIZE as i32 && nr < GRID_SIZE as i32 {
                let cost = if dc == 0 || dr == 0 { 1.0 } else { 1.414 };
                result.push((nc as u16, nr as u16, cost));
            }
        }
    }
    result
}

/// A cell is passable if it is not a known Wall.
/// Unknown cells are optimistically treated as passable (for exploration).
fn is_passable(col: u16, row: u16, known_map: &KnownMap) -> bool {
    known_map.cells[row as usize][col as usize] != KnownCell::Wall
}

/// Reconstruct the path from start to goal via `came_from` map.
fn reconstruct(
    start: (u16, u16),
    goal: (u16, u16),
    came_from: &HashMap<(u16, u16), (u16, u16)>,
) -> Path {
    let mut waypoints = Vec::new();
    let mut cur = goal;
    loop {
        waypoints.push(WorldPos::from(GridPos {
            col: cur.0,
            row: cur.1,
        }));
        if cur == start {
            break;
        }
        match came_from.get(&cur) {
            Some(&prev) => cur = prev,
            None => break,
        }
    }
    waypoints.reverse();
    Path {
        waypoints,
        is_to_frontier: false,
    }
}
