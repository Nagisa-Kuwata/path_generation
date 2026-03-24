// T025, T029: A* and FrontierFinder tests

use path_generation::maze::types::WorldPos;
use path_generation::maze::{CELL_SIZE, GridPos, MazeGenerator};
use path_generation::planner::{FrontierFinder, Planner};
use path_generation::robot::types::{KnownCell, KnownMap};

// --- Helpers ---

/// Build a KnownMap that mirrors the true maze (all cells marked FreeSpace or Wall).
fn full_known_map(maze: &path_generation::maze::Maze) -> KnownMap {
    let mut km = KnownMap::default();
    for row in 0..240 {
        for col in 0..240 {
            km.cells[row][col] = match maze.grid[row][col] {
                path_generation::maze::CellType::Passage => KnownCell::FreeSpace,
                path_generation::maze::CellType::Wall => KnownCell::Wall,
            };
        }
    }
    km
}

// --- T025: A* tests ---

#[test]
fn astar_returns_some_path_in_fully_known_maze() {
    let maze = MazeGenerator::generate(42);
    let km = full_known_map(&maze);
    let robot_pos = WorldPos::from(maze.start);
    let path = Planner::plan(robot_pos, maze.goal, &km);
    assert!(
        path.is_some(),
        "expected A* to find a path from start to goal"
    );
}

#[test]
fn astar_first_waypoint_is_robot_position() {
    let maze = MazeGenerator::generate(42);
    let km = full_known_map(&maze);
    let robot_pos = WorldPos::from(maze.start);
    let path = Planner::plan(robot_pos, maze.goal, &km).expect("path expected");
    let first = path
        .waypoints
        .first()
        .expect("waypoints should not be empty");
    let dx = (first.x - robot_pos.x).abs();
    let dy = (first.y - robot_pos.y).abs();
    assert!(
        dx <= CELL_SIZE && dy <= CELL_SIZE,
        "first waypoint should be at robot pos, got ({:.4},{:.4})",
        first.x,
        first.y
    );
}

#[test]
fn astar_path_does_not_pass_through_walls() {
    let maze = MazeGenerator::generate(42);
    let km = full_known_map(&maze);
    let robot_pos = WorldPos::from(maze.start);
    let path = Planner::plan(robot_pos, maze.goal, &km).expect("path expected");
    for wp in &path.waypoints {
        let gp = GridPos::from(*wp);
        assert_eq!(
            km.cells[gp.row as usize][gp.col as usize],
            KnownCell::FreeSpace,
            "waypoint ({},{}) passes through non-FreeSpace cell",
            gp.col,
            gp.row
        );
    }
}

#[test]
fn astar_returns_none_when_goal_is_walled_off() {
    let mut km = KnownMap::default();
    // Mark all cells as FreeSpace except a wall ring around the goal.
    for row in 0..240_usize {
        for col in 0..240_usize {
            km.cells[row][col] = KnownCell::FreeSpace;
        }
    }
    // Surround goal position (0,0) with walls so it is unreachable.
    let goal = GridPos { col: 0, row: 0 };
    km.cells[0][0] = KnownCell::Wall;
    km.cells[0][1] = KnownCell::Wall;
    km.cells[1][0] = KnownCell::Wall;
    km.cells[1][1] = KnownCell::Wall;

    let robot_pos = WorldPos { x: 1.0, y: -1.0 };
    let result = Planner::plan(robot_pos, goal, &km);
    assert!(result.is_none(), "expected None when goal is walled off");
}

#[test]
fn astar_returns_immediately_when_already_at_goal() {
    let km = KnownMap::default();
    let goal = GridPos { col: 120, row: 120 };
    let robot_pos = WorldPos { x: 0.0, y: 0.0 };
    let path = Planner::plan(robot_pos, goal, &km).expect("expected immediate path");
    assert_eq!(path.waypoints.len(), 1);
}

// --- T029: FrontierFinder tests ---

#[test]
fn frontier_returns_some_when_unknown_neighbors_exist() {
    let mut km = KnownMap::default();
    // Mark center cell and a small radius as FreeSpace.
    for dr in -2_i32..=2 {
        for dc in -2_i32..=2 {
            let r = (120 + dr) as usize;
            let c = (120 + dc) as usize;
            km.cells[r][c] = KnownCell::FreeSpace;
        }
    }
    // The FreeSpace region is surrounded by Unknown => frontier cells exist.
    let robot_pos = WorldPos { x: 0.0, y: 0.0 };
    let frontier = FrontierFinder::nearest_frontier(robot_pos, &km);
    assert!(
        frontier.is_some(),
        "expected a frontier when Unknown neighbors exist"
    );
}

#[test]
fn frontier_returns_none_when_all_reachable_cells_are_known() {
    let mut km = KnownMap::default();
    // Fill the entire map with FreeSpace or Wall (no Unknown at all).
    for row in 0..240_usize {
        for col in 0..240_usize {
            km.cells[row][col] = KnownCell::FreeSpace;
        }
    }
    // Now no cell is adjacent to Unknown => no frontier.
    let robot_pos = WorldPos { x: 0.0, y: 0.0 };
    let frontier = FrontierFinder::nearest_frontier(robot_pos, &km);
    assert!(frontier.is_none(), "expected None when all cells are known");
}

#[test]
fn frontier_returns_none_when_robot_cell_is_not_free() {
    // Robot is on an Unknown cell (not started exploring yet).
    let km = KnownMap::default(); // all Unknown
    let robot_pos = WorldPos { x: 0.0, y: 0.0 };
    let frontier = FrontierFinder::nearest_frontier(robot_pos, &km);
    assert!(frontier.is_none());
}
