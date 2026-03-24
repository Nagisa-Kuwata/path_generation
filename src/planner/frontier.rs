use std::collections::VecDeque;

use crate::maze::{GRID_SIZE, GridPos, WorldPos};
use crate::robot::types::{KnownCell, KnownMap};

/// Frontier-based exploration helper.
///
/// A "frontier" is a known-free cell that is adjacent (cardinal direction) to
/// at least one unknown cell. Navigating to the nearest frontier directs the
/// robot toward unexplored areas of the maze.
pub struct FrontierFinder;

impl FrontierFinder {
    /// Find the nearest frontier cell reachable from `robot_pos` via BFS.
    ///
    /// Returns `None` when no frontier exists (all reachable cells are known).
    pub fn nearest_frontier(robot_pos: WorldPos, known_map: &KnownMap) -> Option<WorldPos> {
        let start = GridPos::from(robot_pos);
        let sc = start.col as usize;
        let sr = start.row as usize;

        if known_map.cells[sr][sc] != KnownCell::FreeSpace {
            return None;
        }

        let mut visited = vec![vec![false; GRID_SIZE]; GRID_SIZE];
        let mut queue: VecDeque<(usize, usize)> = VecDeque::new();

        visited[sr][sc] = true;
        queue.push_back((sc, sr));

        while let Some((c, r)) = queue.pop_front() {
            // Is this FreeSpace cell adjacent to Unknown? Then it is a frontier.
            if is_frontier(c, r, known_map) {
                return Some(WorldPos::from(GridPos {
                    col: c as u16,
                    row: r as u16,
                }));
            }

            // Expand to cardinal FreeSpace neighbors.
            for (nc, nr) in cardinal_neighbors(c, r) {
                if !visited[nr][nc] && known_map.cells[nr][nc] == KnownCell::FreeSpace {
                    visited[nr][nc] = true;
                    queue.push_back((nc, nr));
                }
            }
        }

        None
    }
}

fn is_frontier(c: usize, r: usize, known_map: &KnownMap) -> bool {
    cardinal_neighbors(c, r)
        .into_iter()
        .any(|(nc, nr)| known_map.cells[nr][nc] == KnownCell::Unknown)
}

fn cardinal_neighbors(c: usize, r: usize) -> Vec<(usize, usize)> {
    let mut result = Vec::with_capacity(4);
    if c > 0 {
        result.push((c - 1, r));
    }
    if c + 1 < GRID_SIZE {
        result.push((c + 1, r));
    }
    if r > 0 {
        result.push((c, r - 1));
    }
    if r + 1 < GRID_SIZE {
        result.push((c, r + 1));
    }
    result
}
