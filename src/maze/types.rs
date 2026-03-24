// Grid cells per side: 12m / 0.05m = 240
pub const GRID_SIZE: usize = 240;
// Physical size of one cell in meters
pub const CELL_SIZE: f32 = 0.05;

// Cell type in the maze grid
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CellType {
    Wall,
    Passage,
}

// Grid coordinate (column, row index)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GridPos {
    pub col: u16,
    pub row: u16,
}

// Physical coordinate in meters (origin = maze center, Y-up positive)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WorldPos {
    pub x: f32,
    pub y: f32,
}

impl From<GridPos> for WorldPos {
    fn from(g: GridPos) -> Self {
        let half = (GRID_SIZE as f32) / 2.0;
        WorldPos {
            x: (g.col as f32 - half) * CELL_SIZE,
            y: (half - g.row as f32) * CELL_SIZE,
        }
    }
}

impl From<WorldPos> for GridPos {
    fn from(w: WorldPos) -> Self {
        let half = (GRID_SIZE as f32) / 2.0;
        let col = ((w.x / CELL_SIZE) + half).round() as u16;
        let row = (half - (w.y / CELL_SIZE)).round() as u16;
        GridPos { col, row }
    }
}

impl WorldPos {
    // Euclidean distance between two world positions
    pub fn distance(self, other: WorldPos) -> f32 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        (dx * dx + dy * dy).sqrt()
    }
}

// Maze data (owns the grid on the heap)
pub struct Maze {
    pub grid: Box<[[CellType; GRID_SIZE]; GRID_SIZE]>,
    pub start: GridPos,
    pub goal: GridPos,
    pub seed: u64,
}
