/// Number of grid cells per side: 12 m / 0.05 m = 240.
pub const GRID_SIZE: usize = 240;

/// Physical size of one cell in meters.
pub const CELL_SIZE: f32 = 0.05;

/// Cell type in the maze grid.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CellType {
    /// Solid wall ? impassable.
    Wall,
    /// Open passage ? passable.
    Passage,
}

/// Grid coordinate expressed as (column, row) zero-based indices.
///
/// Column 0 is on the left; row 0 is at the top.
/// The maze center is at `GridPos { col: 120, row: 120 }`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GridPos {
    /// Column index (0 ? 239).
    pub col: u16,
    /// Row index (0 ? 239).
    pub row: u16,
}

/// Physical coordinate in meters.
///
/// The origin `(0.0, 0.0)` is at the maze centre.
/// X increases to the right; Y increases upward.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WorldPos {
    /// Horizontal position in metres (right = positive).
    pub x: f32,
    /// Vertical position in metres (up = positive).
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

impl GridPos {
    /// Snap to the nearest logical cell (even col and row).
    ///
    /// The maze passage cells always sit at even indices. This prevents the
    /// robot, which may momentarily be positioned on an odd-indexed connector
    /// cell during movement, from being mis-classified as being on a wall cell.
    pub fn snap_even(self) -> GridPos {
        // Round each coordinate to the nearest even number.
        let col = ((self.col as usize + 1) / 2 * 2).min(GRID_SIZE - 2) as u16;
        let row = ((self.row as usize + 1) / 2 * 2).min(GRID_SIZE - 2) as u16;
        GridPos { col, row }
    }
}

impl WorldPos {
    /// Euclidean distance to another [`WorldPos`] in metres.
    pub fn distance(self, other: WorldPos) -> f32 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        (dx * dx + dy * dy).sqrt()
    }
}

/// Maze data (owns the 240?~240 grid on the heap).
///
/// The `start` position is always the maze centre `GridPos { col: 120, row: 120 }`.
/// The `goal` is a randomly selected perimeter cell (outer edge of the grid).
pub struct Maze {
    /// The full 240x240 cell grid.
    pub grid: Box<[[CellType; GRID_SIZE]; GRID_SIZE]>,
    /// Starting position of the robot (always the maze centre).
    pub start: GridPos,
    /// Target position for goal-aware mode (perimeter cell).
    pub goal: GridPos,
    /// Target position for blind exploration mode (random inner passage cell).
    pub blind_goal: GridPos,
    /// Random seed used to generate this maze.
    pub seed: u64,
}
