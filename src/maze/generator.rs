use rand::SeedableRng;
use rand::seq::SliceRandom;
use rand_chacha::ChaCha8Rng;

use super::types::{CellType, GRID_SIZE, GridPos, Maze};

// Half-size of robot in cells (0.5m / 0.05m = 10, half = 5)
const ROBOT_HALF: usize = 5;

// Passage grid: 120x120 logical cells.
// Logical cell (pc, pr) maps to grid column pc*2, row pr*2.
// Walls between cells sit at odd indices.
// Center (120, 120) = logical cell (60, 60) -> col=120 is even -> Passage. OK
const PCOLS: usize = 120;
const PROWS: usize = 120;

/// Procedural maze generator using the Recursive Backtracking (DFS) algorithm.
///
/// Guarantees:
/// - Every cell is reachable from every other cell (perfect maze / spanning tree).
/// - The start position `GridPos { col: 120, row: 120 }` is always a `Passage`.
/// - The goal is a randomly selected perimeter `Passage` cell.
/// - Given the same seed the output is deterministic.
pub struct MazeGenerator;

impl MazeGenerator {
    /// Generate a 240x~240 maze from `seed`.
    ///
    /// Passage cells occupy even grid indices (`col = pc * 2`, `row = pr * 2`).
    /// Wall cells between passages occupy odd indices.
    pub fn generate(seed: u64) -> Maze {
        let mut rng = ChaCha8Rng::seed_from_u64(seed);
        // Start with all walls
        let mut grid = Box::new([[CellType::Wall; GRID_SIZE]; GRID_SIZE]);

        // Mark every logical passage cell (even col, even row)
        // DFS will carve connections between them.
        let mut visited = vec![vec![false; PCOLS]; PROWS];
        let mut stack: Vec<(usize, usize)> = Vec::new();

        // Start DFS from logical center cell (60, 60) -> grid (120, 120)
        let start_pc = PCOLS / 2;
        let start_pr = PROWS / 2;
        visited[start_pr][start_pc] = true;
        stack.push((start_pc, start_pr));

        while let Some(&(pc, pr)) = stack.last() {
            let neighbors = Self::unvisited_neighbors(pc, pr, &visited);
            if neighbors.is_empty() {
                stack.pop();
            } else {
                let &(nc, nr) = neighbors.choose(&mut rng).unwrap();
                visited[nr][nc] = true;
                // Wall cell sits at the odd index between two even passage cells.
                // nc > pc => move right: wall at col pc*2+1, row pr*2
                // nc < pc => move left:  wall at col pc*2-1, row pr*2
                // nr > pr => move down:  wall at col pc*2,   row pr*2+1
                // nr < pr => move up:    wall at col pc*2,   row pr*2-1
                let wc: usize = if nc > pc {
                    pc * 2 + 1
                } else if nc < pc {
                    pc * 2 - 1
                } else {
                    pc * 2
                };
                let wr: usize = if nr > pr {
                    pr * 2 + 1
                } else if nr < pr {
                    pr * 2 - 1
                } else {
                    pr * 2
                };
                if wc < GRID_SIZE && wr < GRID_SIZE {
                    grid[wr][wc] = CellType::Passage;
                }
                stack.push((nc, nr));
            }
        }

        // Mark all visited passage cells
        for (pr, visited_row) in visited.iter().enumerate() {
            for (pc, &vis) in visited_row.iter().enumerate() {
                if vis {
                    let col = pc * 2;
                    let row = pr * 2;
                    if col < GRID_SIZE && row < GRID_SIZE {
                        grid[row][col] = CellType::Passage;
                    }
                }
            }
        }

        let start = GridPos { col: 120, row: 120 };
        let goal = Self::pick_goal(&mut grid, &mut rng);
        let blind_goal = Self::pick_blind_goal(&grid, &mut rng, start);
        // Border cells (pc=0, pc=119, pr=0, pr=119) are never visited by the
        // DFS (see unvisited_neighbors), so they remain Wall. No further
        // sealing is needed; pick_goal carves the only exit opening.

        Maze {
            grid,
            start,
            goal,
            blind_goal,
            seed,
        }
    }

    /// Restrict DFS to inner logical cells only: pc = 1..=PCOLS-2, pr = 1..=PROWS-2.
    /// This ensures the physical border (row 0, row 239, col 0, col 239) can never
    /// become a Passage through DFS carving, so the outer wall is always solid.
    fn unvisited_neighbors(pc: usize, pr: usize, visited: &[Vec<bool>]) -> Vec<(usize, usize)> {
        let mut n = Vec::new();
        // pc > 1  ??  can move left  (destination pc-1 >= 1, within inner range)
        if pc > 1 && !visited[pr][pc - 1] { n.push((pc - 1, pr)); }
        // pc+1 < PCOLS-1  ??  can move right (destination pc+1 <= PCOLS-2 = 118)
        if pc + 1 < PCOLS - 1 && !visited[pr][pc + 1] { n.push((pc + 1, pr)); }
        if pr > 1 && !visited[pr - 1][pc] { n.push((pc, pr - 1)); }
        if pr + 1 < PROWS - 1 && !visited[pr + 1][pc] { n.push((pc, pr + 1)); }
        n
    }

    // Pick a random inner passage cell (even indices, not the start) for blind mode.
    fn pick_blind_goal(
        grid: &[[CellType; GRID_SIZE]; GRID_SIZE],
        rng: &mut ChaCha8Rng,
        start: GridPos,
    ) -> GridPos {
        let mut candidates: Vec<GridPos> = Vec::new();
        // Even indices only (logical passage cells), skip pc=0/119 and pr=0/119 border.
        for pr in 1..PROWS - 1 {
            for pc in 1..PCOLS - 1 {
                let col = (pc * 2) as u16;
                let row = (pr * 2) as u16;
                if grid[row as usize][col as usize] == CellType::Passage
                    && !(col == start.col && row == start.row)
                {
                    candidates.push(GridPos { col, row });
                }
            }
        }
        candidates
            .choose(rng)
            .copied()
            .unwrap_or(GridPos { col: 2, row: 2 })
    }

    // Pick the goal: choose a random inner border cell and carve an opening
    // through the outer wall so the goal is always reachable from the interior.
    fn pick_goal(grid: &mut [[CellType; GRID_SIZE]; GRID_SIZE], rng: &mut ChaCha8Rng) -> GridPos {
        let n = GRID_SIZE;
        let mut candidates: Vec<GridPos> = Vec::new();

        // Left-edge goals: inner cells at pc=1, pr=1..=PROWS-2.
        // Goal at physical (col=0, row=pr*2); connector at (col=1, row=pr*2).
        for pr in 1..PROWS - 1 {
            candidates.push(GridPos { col: 0, row: (pr * 2) as u16 });
        }
        // Top-edge goals: inner cells at pr=1, pc=1..=PCOLS-2.
        // Goal at physical (col=pc*2, row=0); connector at (col=pc*2, row=1).
        for pc in 1..PCOLS - 1 {
            candidates.push(GridPos { col: (pc * 2) as u16, row: 0 });
        }

        let &goal = candidates.choose(rng).unwrap();
        let gc = goal.col as usize;
        let gr = goal.row as usize;

        // Open the border cell (goal) and the connector one step inward.
        grid[gr][gc] = CellType::Passage;
        if gc == 0 && gc + 1 < n { grid[gr][1] = CellType::Passage; } // left edge
        if gr == 0 && gr + 1 < n { grid[1][gc] = CellType::Passage; } // top edge

        goal
    }
}

impl Maze {
    pub fn is_passable_for_robot(&self, pos: GridPos) -> bool {
        let c = pos.col as usize;
        let r = pos.row as usize;
        if c < ROBOT_HALF || r < ROBOT_HALF {
            return false;
        }
        let c0 = c - ROBOT_HALF;
        let r0 = r - ROBOT_HALF;
        let c1 = c + ROBOT_HALF;
        let r1 = r + ROBOT_HALF;
        if c1 >= GRID_SIZE || r1 >= GRID_SIZE {
            return false;
        }
        for row in r0..=r1 {
            for col in c0..=c1 {
                if self.grid[row][col] == CellType::Wall {
                    return false;
                }
            }
        }
        true
    }
}
