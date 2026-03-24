// T009, T013, T019: maze types and generation tests
use path_generation::maze::generator::MazeGenerator;
use path_generation::maze::types::{CellType, GRID_SIZE, GridPos, WorldPos};

#[test]
fn grid_to_world_origin() {
    // Center cell (120,120) -> physical (0.0, 0.0)
    let pos = GridPos { col: 120, row: 120 };
    let world = WorldPos::from(pos);
    assert!((world.x).abs() < 1e-5, "x should be 0: {}", world.x);
    assert!((world.y).abs() < 1e-5, "y should be 0: {}", world.y);
}

#[test]
fn grid_to_world_top_left() {
    // (0,0) -> (-6.0, +6.0)
    let pos = GridPos { col: 0, row: 0 };
    let world = WorldPos::from(pos);
    assert!((world.x - (-6.0_f32)).abs() < 1e-4, "x: {}", world.x);
    assert!((world.y - 6.0_f32).abs() < 1e-4, "y: {}", world.y);
}

#[test]
fn world_to_grid_roundtrip() {
    let original = GridPos { col: 57, row: 183 };
    let world = WorldPos::from(original);
    let back = GridPos::from(world);
    assert_eq!(back.col, original.col);
    assert_eq!(back.row, original.row);
}

#[test]
fn grid_size_is_240() {
    assert_eq!(GRID_SIZE, 240);
}

// T013: maze generation tests
#[test]
fn maze_start_is_passage() {
    let maze = MazeGenerator::generate(42);
    let start = maze.start;
    assert_eq!(
        maze.grid[start.row as usize][start.col as usize],
        CellType::Passage
    );
}

#[test]
fn maze_goal_is_passage() {
    let maze = MazeGenerator::generate(42);
    let goal = maze.goal;
    assert_eq!(
        maze.grid[goal.row as usize][goal.col as usize],
        CellType::Passage
    );
}

#[test]
fn maze_goal_is_on_perimeter() {
    let maze = MazeGenerator::generate(42);
    let g = maze.goal;
    let n = GRID_SIZE as u16;
    let on_edge = g.row == 0 || g.row == n - 1 || g.col == 0 || g.col == n - 1;
    assert!(on_edge, "goal {:?} is not on perimeter", g);
}

#[test]
fn maze_is_deterministic_with_same_seed() {
    let a = MazeGenerator::generate(12345);
    let b = MazeGenerator::generate(12345);
    // Compare a subset of cells for equality
    for r in 0..10 {
        for c in 0..10 {
            assert_eq!(a.grid[r][c], b.grid[r][c]);
        }
    }
    assert_eq!(a.goal.col, b.goal.col);
    assert_eq!(a.goal.row, b.goal.row);
}

#[test]
fn maze_differs_with_different_seeds() {
    let a = MazeGenerator::generate(1);
    let b = MazeGenerator::generate(2);
    // Very likely that at least one cell differs
    let any_diff = (0..240)
        .flat_map(|r| (0..240).map(move |c| (r, c)))
        .any(|(r, c)| a.grid[r][c] != b.grid[r][c]);
    assert!(any_diff, "mazes with different seeds should differ");
}

#[test]
fn maze_center_is_start() {
    let maze = MazeGenerator::generate(99);
    // Start must be at grid center (120, 120)
    assert_eq!(maze.start.col, 120);
    assert_eq!(maze.start.row, 120);
}

// T019: simulation restart tests (added when SimulationState is implemented)
