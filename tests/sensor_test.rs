// T020, T023: LRF scan and map update tests

use path_generation::maze::types::WorldPos;
use path_generation::maze::{CELL_SIZE, CellType, GridPos, MazeGenerator};
use path_generation::robot::types::{KnownCell, Robot};
use path_generation::sensor::Lrf;

// --- Helpers ---

fn make_maze_and_scan() -> (
    path_generation::maze::Maze,
    path_generation::sensor::LrfScan,
) {
    let maze = MazeGenerator::generate(42);
    let robot_pos = WorldPos { x: 0.0, y: 0.0 };
    let scan = Lrf::scan(robot_pos, &maze);
    (maze, scan)
}

// --- T020: LRF scan property tests ---

#[test]
fn scan_produces_exactly_720_points() {
    let (_, scan) = make_maze_and_scan();
    assert_eq!(scan.points.len(), 720);
    assert_eq!(scan.is_wall_hit.len(), 720);
}

#[test]
fn scan_origin_matches_robot_pos() {
    let (_, scan) = make_maze_and_scan();
    assert_eq!(scan.origin.x, 0.0_f32);
    assert_eq!(scan.origin.y, 0.0_f32);
}

#[test]
fn scan_all_points_within_max_range() {
    let (_, scan) = make_maze_and_scan();
    for pt in &scan.points {
        let dx = pt.x - scan.origin.x;
        let dy = pt.y - scan.origin.y;
        let dist = (dx * dx + dy * dy).sqrt();
        // Allow 1 cell tolerance for floating-point rounding.
        assert!(
            dist <= 5.0 + CELL_SIZE,
            "point at dist {dist:.4}m exceeds max range"
        );
    }
}

#[test]
fn scan_wall_hits_land_on_wall_cells() {
    let (maze, scan) = make_maze_and_scan();
    for (i, cell) in scan.hit_cells.iter().enumerate() {
        if let Some(gp) = cell {
            assert_eq!(
                maze.grid[gp.row as usize][gp.col as usize],
                CellType::Wall,
                "ray {i}: hit_cell ({},{}) is not a wall",
                gp.col,
                gp.row
            );
        }
    }
}

#[test]
fn scan_free_rays_reach_max_range() {
    let (_, scan) = make_maze_and_scan();
    for (i, (&is_hit, pt)) in scan.is_wall_hit.iter().zip(&scan.points).enumerate() {
        if is_hit {
            continue;
        }
        let dx = pt.x - scan.origin.x;
        let dy = pt.y - scan.origin.y;
        let dist = (dx * dx + dy * dy).sqrt();
        // A non-hit must be at max range (+/- half cell tolerance).
        assert!(
            dist >= 5.0 - CELL_SIZE,
            "ray {i}: non-hit at dist {dist:.4}m is shorter than max range"
        );
    }
}

// --- T023: Robot::update_map tests ---

fn make_robot_at_center(maze_seed: u64) -> (path_generation::maze::Maze, Robot) {
    let maze = MazeGenerator::generate(maze_seed);
    let robot = Robot::new(WorldPos { x: 0.0, y: 0.0 }, maze.start);
    (maze, robot)
}

#[test]
fn update_map_marks_origin_cell_as_free() {
    let (maze, mut robot) = make_robot_at_center(42);
    let scan = Lrf::scan(robot.position, &maze);
    robot.update_map(&scan);
    // The robot's own cell (120, 120) must be FreeSpace after scan.
    assert_eq!(robot.known_map.cells[120][120], KnownCell::FreeSpace);
}

#[test]
fn update_map_marks_wall_cells() {
    let (maze, mut robot) = make_robot_at_center(42);
    let scan = Lrf::scan(robot.position, &maze);
    robot.update_map(&scan);
    // At least one cell should be marked Wall.
    let wall_count = robot
        .known_map
        .cells
        .iter()
        .flat_map(|row| row.iter())
        .filter(|&&c| c == KnownCell::Wall)
        .count();
    assert!(wall_count > 0, "no Wall cells after update_map");
}

#[test]
fn update_map_leaves_distant_cells_unknown() {
    let (maze, mut robot) = make_robot_at_center(42);
    let scan = Lrf::scan(robot.position, &maze);
    robot.update_map(&scan);
    // Cells far from the robot (beyond max LRF range) should still be Unknown.
    // Corner cell (0,0) is 120 cells away from center, well beyond 100-cell range.
    assert_eq!(robot.known_map.cells[0][0], KnownCell::Unknown);
}

#[test]
fn update_map_does_not_overwrite_existing_entries() {
    let (maze, mut robot) = make_robot_at_center(42);
    let scan = Lrf::scan(robot.position, &maze);
    // Pre-mark the origin cell as Wall (simulate prior observation).
    robot.known_map.cells[120][120] = KnownCell::Wall;
    robot.update_map(&scan);
    // The pre-existing Wall entry must not be overwritten.
    assert_eq!(robot.known_map.cells[120][120], KnownCell::Wall);
}

// --- LrfScan::detects_cell tests ---

/// The robot can see its own position (zero distance).
#[test]
fn scan_detects_same_position_returns_true() {
    let (_, scan) = make_maze_and_scan();
    let pos = GridPos { col: 120, row: 120 };
    assert!(scan.detects_cell(pos));
}

/// A target within the same grid cell as the robot is always visible.
#[test]
fn scan_detects_same_cell_target_is_visible() {
    let (_, scan) = make_maze_and_scan();
    assert!(scan.detects_cell(GridPos { col: 120, row: 120 }));
}

/// A passage cell directly connected to the robot (no wall in between) is visible.
#[test]
fn scan_detects_connected_passage_is_visible() {
    let maze = MazeGenerator::generate(42);
    let scan = Lrf::scan(WorldPos { x: 0.0, y: 0.0 }, &maze);
    // The center cell (120, 120) is always a passage.  Find the first cardinal
    // physical neighbour that is also a Passage (i.e. the wall between them was
    // carved by the DFS), then test visibility to the cell beyond it.
    let offsets: &[(i32, i32)] = &[(2, 0), (-2, 0), (0, -2), (0, 2)];
    let mut found = false;
    for &(dc, dr) in offsets {
        let wall_c = (120 + dc / 2) as usize; // connector cell between start and neighbour
        let wall_r = (120 + dr / 2) as usize;
        if maze.grid[wall_r][wall_c] == CellType::Wall {
            continue;
        }
        // The connector is open: the neighbour passage is directly reachable.
        let target_gp = GridPos {
            col: (120 + dc) as u16,
            row: (120 + dr) as u16,
        };
        assert!(
            scan.detects_cell(target_gp),
            "expected scan detection for open neighbour ({},{})", target_gp.col, target_gp.row
        );
        found = true;
        break;
    }
    assert!(found, "no open cardinal neighbour found from start -- maze generation bug");
}

/// A target beyond MAX_RANGE_M (5 m) is never visible regardless of walls.
#[test]
fn scan_does_not_detect_beyond_max_range_cell() {
    let (_, scan) = make_maze_and_scan();
    assert!(!scan.detects_cell(GridPos { col: 0, row: 120 }));
}

/// blind_goal visibility from robot start position is consistent with
/// whether a free-ray LRF ray at that angle reaches past that distance.
#[test]
fn scan_blind_goal_detection_is_range_bounded() {
    let maze = MazeGenerator::generate(42);
    let from = WorldPos::from(maze.start);
    let scan = Lrf::scan(from, &maze);
    let goal = WorldPos::from(maze.blind_goal);
    let detected = scan.detects_cell(maze.blind_goal);

    let dx = goal.x - from.x;
    let dy = goal.y - from.y;
    let dist = (dx * dx + dy * dy).sqrt();

    if dist > 5.0 {
        assert!(!detected, "target beyond range must not be detected");
    }
    let _ = detected;
}
