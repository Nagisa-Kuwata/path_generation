use std::collections::VecDeque;

use crate::maze::{GRID_SIZE, GridPos, WorldPos};
use crate::robot::types::{KnownCell, KnownMap};

// Logical cell counts (240 / 2 = 120).
const LCOLS: usize = GRID_SIZE / 2;
const LROWS: usize = GRID_SIZE / 2;

/// Frontier-based exploration helper.
///
/// Operates exclusively on **logical cells** (even physical indices).
/// A frontier logical cell is one whose physical cell is FreeSpace and that
/// has at least one Unknown neighbour (physical cardinal neighbour).
pub struct FrontierFinder;

impl FrontierFinder {
    /// Find the nearest frontier logical cell reachable from `robot_pos` via BFS.
    ///
    /// Returns `None` when no frontier exists (all reachable logical cells are fully known).
    pub fn nearest_frontier(robot_pos: WorldPos, known_map: &KnownMap) -> Option<WorldPos> {
        let phys = GridPos::from(robot_pos).snap_even();
        let start_lc = (phys.col as usize / 2).min(LCOLS - 1);
        let start_lr = (phys.row as usize / 2).min(LROWS - 1);

        let mut visited = vec![vec![false; LCOLS]; LROWS];
        let mut queue: VecDeque<(usize, usize)> = VecDeque::new();

        // Seed the BFS from the robot's snapped logical cell and its immediate
        // logical neighbours. This is necessary because the snapped cell may be
        // Unknown (the robot is traversing a connector cell between two passage
        // cells), and a single-cell seed would leave the BFS with nothing to expand.
        for dlc in -1i32..=1 {
            for dlr in -1i32..=1 {
                let lc = start_lc as i32 + dlc;
                let lr = start_lr as i32 + dlr;
                if lc < 0 || lr < 0 || lc >= LCOLS as i32 || lr >= LROWS as i32 {
                    continue;
                }
                let lc = lc as usize;
                let lr = lr as usize;
                if !visited[lr][lc] {
                    visited[lr][lc] = true;
                    queue.push_back((lc, lr));
                }
            }
        }

        while let Some((lc, lr)) = queue.pop_front() {
            let pc = lc * 2;
            let pr = lr * 2;

            // Only treat confirmed FreeSpace logical cells as potential frontiers.
            // Also require that the frontier is not the starting logical cell itself:
            // returning the start would make A* produce a zero-length path.
            if known_map.cells[pr][pc] == KnownCell::FreeSpace
                && is_logical_frontier(lc, lr, known_map)
                && (lc != start_lc || lr != start_lr)
            {
                return Some(WorldPos::from(GridPos {
                    col: pc as u16,
                    row: pr as u16,
                }));
            }

            // Expand to logical neighbours that can be reached (wall cell between
            // them must not be a confirmed Wall).
            for (nlc, nlr) in logical_neighbors(lc, lr) {
                if visited[nlr][nlc] {
                    continue;
                }
                // Physical coords of destination logical cell.
                let npc = nlc * 2;
                let npr = nlr * 2;
                // Skip if destination is a confirmed Wall.
                if known_map.cells[npr][npc] == KnownCell::Wall {
                    continue;
                }
                // Skip if the separating wall cell is a confirmed Wall.
                let wall_c = lc + nlc; // = pc + (npc - pc)/1 direction
                let wall_r = lr + nlr;
                if known_map.cells[wall_r][wall_c] == KnownCell::Wall {
                    continue;
                }
                visited[nlr][nlc] = true;
                queue.push_back((nlc, nlr));
            }
        }

        None
    }

    /// Find the nearest Unknown logical cell reachable from `robot_pos`,
    /// treating Unknown connector cells as passable (optimistic BFS).
    ///
    /// Used as a fallback in Blind mode when no frontier exists yet (e.g. the
    /// robot's first tick where the LRF has fully revealed its immediate area
    /// but not yet reached any passage boundary with Unknown neighbours).
    /// Returns `None` when all reachable logical cells are already mapped.
    pub fn nearest_unknown(robot_pos: WorldPos, known_map: &KnownMap) -> Option<WorldPos> {
        let phys = GridPos::from(robot_pos).snap_even();
        let start_lc = (phys.col as usize / 2).min(LCOLS - 1);
        let start_lr = (phys.row as usize / 2).min(LROWS - 1);

        let mut visited = vec![vec![false; LCOLS]; LROWS];
        let mut queue: VecDeque<(usize, usize)> = VecDeque::new();

        // Seed only the snapped cell itself, then expand via passable connectors.
        //
        // IMPORTANT: do NOT use a 3x3 neighbourhood seed here.  A 3x3 seed
        // adds logical cells (e.g. lc = LCOLS-1 border cells) without first
        // verifying that a passable connector links them to the robot's
        // position.  Those border cells are often Unknown (not yet scanned)
        // but completely unreachable by A*, so returning one as the wander
        // target causes A* to fail every tick and the robot to freeze.
        // Starting from a single snapped cell and expanding only through
        // verified-passable connectors guarantees that whatever cell the BFS
        // returns is also reachable by the A* planner.
        visited[start_lr][start_lc] = true;
        queue.push_back((start_lc, start_lr));

        while let Some((lc, lr)) = queue.pop_front() {
            let pc = lc * 2;
            let pr = lr * 2;
            // If this logical cell is Unknown it is unexplored territory:
            // navigate here to open up new frontiers.
            if known_map.cells[pr][pc] == KnownCell::Unknown {
                return Some(WorldPos::from(GridPos {
                    col: pc as u16,
                    row: pr as u16,
                }));
            }
            // Expand to logical neighbours through any non-confirmed-Wall connector.
            // Unknown connectors are treated as passable (optimistic).
            for (nlc, nlr) in logical_neighbors(lc, lr) {
                if visited[nlr][nlc] {
                    continue;
                }
                let wall_c = lc + nlc;
                let wall_r = lr + nlr;
                if known_map.cells[wall_r][wall_c] == KnownCell::Wall {
                    continue;
                }
                visited[nlr][nlc] = true;
                queue.push_back((nlc, nlr));
            }
        }

        None
    }

    /// Return all reachable FreeSpace logical cells in BFS visit order from
    /// `robot_pos`, skipping the starting cell itself.
    ///
    /// Used as a post-exploration coverage path in Blind mode: after all
    /// Unknown cells are exhausted the robot navigates through every returned
    /// cell, guaranteeing it physically passes through blind_goal and can
    /// trigger the arrival check.
    pub fn all_reachable_free(robot_pos: WorldPos, known_map: &KnownMap) -> Vec<GridPos> {
        let phys = GridPos::from(robot_pos).snap_even();
        let start_lc = (phys.col as usize / 2).min(LCOLS - 1);
        let start_lr = (phys.row as usize / 2).min(LROWS - 1);

        let mut visited = vec![vec![false; LCOLS]; LROWS];
        let mut queue: VecDeque<(usize, usize)> = VecDeque::new();
        let mut result: Vec<GridPos> = Vec::new();

        visited[start_lr][start_lc] = true;
        queue.push_back((start_lc, start_lr));

        while let Some((lc, lr)) = queue.pop_front() {
            let pc = lc * 2;
            let pr = lr * 2;
            // Collect every FreeSpace cell except the starting cell (robot is
            // already there, so navigating there would produce a zero-length path).
            if known_map.cells[pr][pc] == KnownCell::FreeSpace
                && (lc != start_lc || lr != start_lr)
            {
                result.push(GridPos { col: pc as u16, row: pr as u16 });
            }
            for (nlc, nlr) in logical_neighbors(lc, lr) {
                if visited[nlr][nlc] {
                    continue;
                }
                let wall_c = lc + nlc;
                let wall_r = lr + nlr;
                if known_map.cells[wall_r][wall_c] == KnownCell::Wall {
                    continue;
                }
                visited[nlr][nlc] = true;
                queue.push_back((nlc, nlr));
            }
        }

        result
    }
}

/// A logical cell is a frontier when any of the 4 physical cardinal neighbours
/// of its physical cell (pc, pr) is Unknown.
fn is_logical_frontier(lc: usize, lr: usize, known_map: &KnownMap) -> bool {
    let pc = lc * 2;
    let pr = lr * 2;
    // Check the immediate physical cardinal neighbours (these are the wall/passage
    // connector cells at odd indices and the adjacent even cells).
    for (dc, dr) in [(-1i32, 0i32), (1, 0), (0, -1), (0, 1)] {
        let nc = pc as i32 + dc;
        let nr = pr as i32 + dr;
        if nc >= 0 && nr >= 0 && nc < GRID_SIZE as i32 && nr < GRID_SIZE as i32 {
            if known_map.cells[nr as usize][nc as usize] == KnownCell::Unknown {
                return true;
            }
        }
    }
    false
}

fn logical_neighbors(lc: usize, lr: usize) -> Vec<(usize, usize)> {
    let mut result = Vec::with_capacity(4);
    if lc > 0 { result.push((lc - 1, lr)); }
    if lc + 1 < LCOLS { result.push((lc + 1, lr)); }
    if lr > 0 { result.push((lc, lr - 1)); }
    if lr + 1 < LROWS { result.push((lc, lr + 1)); }
    result
}
