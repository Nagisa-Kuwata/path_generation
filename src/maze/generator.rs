use rand::SeedableRng;
use rand::seq::SliceRandom;
use rand_chacha::ChaCha8Rng;

use super::types::{CellType, GRID_SIZE, GridPos, Maze};

// Half-size of robot in cells (0.5m / 0.05m = 10, half = 5)
const ROBOT_HALF: usize = 5;

// Passage grid: 120x120 logical cells.
// Logical cell (pc, pr) maps to grid column pc*2, row pr*2.
// Walls between cells sit at odd indices.
// Center (120, 120) = logical cell (60, 60) -> col=120 is even -> Passage. ?
const PCOLS: usize = 120;
const PROWS: usize = 120;

pub struct MazeGenerator;

impl MazeGenerator {
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

        Maze {
            grid,
            start,
            goal,
            seed,
        }
    }

    fn unvisited_neighbors(pc: usize, pr: usize, visited: &[Vec<bool>]) -> Vec<(usize, usize)> {
        let mut n = Vec::new();
        if pc > 0 && !visited[pr][pc - 1] {
            n.push((pc - 1, pr));
        }
        if pc + 1 < PCOLS && !visited[pr][pc + 1] {
            n.push((pc + 1, pr));
        }
        if pr > 0 && !visited[pr - 1][pc] {
            n.push((pc, pr - 1));
        }
        if pr + 1 < PROWS && !visited[pr + 1][pc] {
            n.push((pc, pr + 1));
        }
        n
    }

    // Pick goal after grid is fully built; open a wall if no perimeter passage exists.
    fn pick_goal(grid: &mut [[CellType; GRID_SIZE]; GRID_SIZE], rng: &mut ChaCha8Rng) -> GridPos {
        let n = GRID_SIZE;
        let mut candidates: Vec<GridPos> = Vec::new();

        for (col, cell) in grid[0].iter().enumerate() {
            if *cell == CellType::Passage {
                candidates.push(GridPos { col: col as u16, row: 0 });
            }
        }
        for (col, cell) in grid[n - 1].iter().enumerate() {
            if *cell == CellType::Passage {
                candidates.push(GridPos {
                    col: col as u16,
                    row: (n - 1) as u16,
                });
            }
        }
        for (row, row_data) in grid.iter().enumerate().skip(1).take(n - 2) {
            if row_data[0] == CellType::Passage {
                candidates.push(GridPos {
                    col: 0,
                    row: row as u16,
                });
            }
            if row_data[n - 1] == CellType::Passage {
                candidates.push(GridPos {
                    col: (n - 1) as u16,
                    row: row as u16,
                });
            }
        }

        if candidates.is_empty() {
            // Open the nearest even passage column's top cell as an exit
            for pc in 0..PCOLS {
                let col = pc * 2;
                if col < n {
                    grid[0][col] = CellType::Passage;
                    return GridPos {
                        col: col as u16,
                        row: 0,
                    };
                }
            }
            return GridPos { col: 0, row: 0 };
        }

        *candidates.choose(rng).unwrap()
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
