// T019, T031: simulation restart and tick integration tests
use path_generation::maze::types::{GRID_SIZE, GridPos};
use path_generation::robot::types::{KnownCell, RobotState};
use path_generation::simulation::SimulationState;

// T019: restart tests
#[test]
fn restart_robot_state_is_idle() {
    let sim = SimulationState::restart(Some(42));
    assert_eq!(sim.robot.state, RobotState::Idle);
}

#[test]
fn restart_elapsed_is_zero() {
    let sim = SimulationState::restart(Some(42));
    assert_eq!(sim.elapsed_ms, 0);
}

#[test]
fn restart_robot_at_maze_center() {
    let sim = SimulationState::restart(Some(42));
    let center: GridPos = sim.robot.position.into();
    assert_eq!(center.col, 120);
    assert_eq!(center.row, 120);
}

#[test]
fn restart_known_map_all_unknown() {
    use path_generation::robot::types::KnownCell;
    let sim = SimulationState::restart(Some(42));
    let all_unknown = (0..GRID_SIZE)
        .flat_map(|r| (0..GRID_SIZE).map(move |c| (r, c)))
        .all(|(r, c)| sim.robot.known_map.cells[r][c] == KnownCell::Unknown);
    assert!(all_unknown);
}

#[test]
fn restart_deterministic_with_same_seed() {
    let a = SimulationState::restart(Some(777));
    let b = SimulationState::restart(Some(777));
    assert_eq!(a.seed, b.seed);
    assert_eq!(a.maze.goal.col, b.maze.goal.col);
    assert_eq!(a.maze.goal.row, b.maze.goal.row);
}

// T031: tick tests -- added in Phase 5

#[test]
fn tick_does_nothing_when_idle() {
    let mut sim = SimulationState::restart(Some(42));
    let pos_before = (sim.robot.position.x, sim.robot.position.y);
    sim.tick(20);
    assert_eq!(sim.elapsed_ms, 0, "elapsed_ms must not change while Idle");
    assert_eq!(sim.robot.position.x, pos_before.0);
    assert_eq!(sim.robot.position.y, pos_before.1);
}

#[test]
fn tick_advances_elapsed_time_after_start() {
    let mut sim = SimulationState::restart(Some(42));
    sim.start();
    sim.tick(20);
    assert_eq!(sim.elapsed_ms, 20);
    sim.tick(20);
    assert_eq!(sim.elapsed_ms, 40);
}

#[test]
fn tick_moves_robot_after_start() {
    let mut sim = SimulationState::restart(Some(42));
    sim.start();
    let pos_before = (sim.robot.position.x, sim.robot.position.y);
    // Tick with 1000 ms: robot should move at 1 m/s => moves 1 m max.
    sim.tick(1000);
    let dx = sim.robot.position.x - pos_before.0;
    let dy = sim.robot.position.y - pos_before.1;
    let dist = (dx * dx + dy * dy).sqrt();
    assert!(dist > 0.0, "robot should have moved after tick");
}

// Diagnostic: track exploration progress over a long run to see if it ever finishes.
// Run with: cargo test --release --test simulation_test diag_exploration_progress -- --nocapture
#[test]
fn diag_exploration_progress() {
    use path_generation::maze::types::GRID_SIZE;
    for seed in [0u64, 42, 999] {
        let mut sim = SimulationState::restart(Some(seed));
        sim.start();
        let goal = sim.maze.goal;
        let total_cells = (GRID_SIZE * GRID_SIZE) as usize;
        let mut arrived_tick = None;

        for tick in 0..30_000usize {
            sim.tick(20);

            if sim.robot.state == RobotState::Arrived {
                arrived_tick = Some(tick);
                break;
            }

            // Print progress every 5000 ticks.
            if tick % 5000 == 4999 {
                let free_count = sim.robot.known_map.cells.iter()
                    .flat_map(|r| r.iter())
                    .filter(|&&c| c == KnownCell::FreeSpace)
                    .count();
                let goal_known = sim.robot.known_map.cells[goal.row as usize][goal.col as usize];
                let pos = sim.robot.position;
                let gp: GridPos = pos.into();
                println!(
                    "seed={seed} tick={} free={}/{} ({:.1}%) goal_cell={:?} state={:?} pos=({},{})@({:.2},{:.2})",
                    tick + 1, free_count, total_cells,
                    100.0 * free_count as f32 / total_cells as f32,
                    goal_known, sim.robot.state,
                    gp.col, gp.row, pos.x, pos.y
                );
            }
        }

        let free_count = sim.robot.known_map.cells.iter()
            .flat_map(|r| r.iter())
            .filter(|&&c| c == KnownCell::FreeSpace)
            .count();
        let goal_known = sim.robot.known_map.cells[goal.row as usize][goal.col as usize];
        if let Some(t) = arrived_tick {
            println!("seed={seed} ARRIVED at tick={t} goal=({},{}) final_free={free_count}", goal.col, goal.row);
        } else {
            println!(
                "seed={seed} NOT ARRIVED after 30000 ticks. free={free_count}/{total_cells} goal_cell={:?} goal=({},{})",
                goal_known, goal.col, goal.row
            );
        }
    }
}

// Diagnostic: run N ticks on several seeds; print state when robot gets stuck.
// This is not an assertion test ? it uses `-- --nocapture` to show debug info.
#[test]
fn diag_stuck_detection() {
    for seed in [0u64, 1, 2, 42, 100, 999, 12345, 7777, 31337] {
        let mut sim = SimulationState::restart(Some(seed));
        sim.start();
        let mut last_pos = sim.robot.position;
        let mut stuck_ticks = 0u32;
        let mut total_ticks = 0u32;

        for _ in 0..3000 {
            sim.tick(20);
            total_ticks += 1;
            let dx = sim.robot.position.x - last_pos.x;
            let dy = sim.robot.position.y - last_pos.y;
            if (dx * dx + dy * dy).sqrt() < 0.001 {
                stuck_ticks += 1;
            } else {
                stuck_ticks = 0;
            }
            last_pos = sim.robot.position;

            if sim.robot.state == RobotState::Arrived {
                println!("seed={seed} ARRIVED at tick={total_ticks}");
                break;
            }

            if stuck_ticks >= 10 {
                let gp: GridPos = sim.robot.position.into();
                let known_at_robot = sim.robot.known_map.cells[gp.row as usize][gp.col as usize];
                let maze_at_robot = sim.maze.grid[gp.row as usize][gp.col as usize];
                let lc = gp.col as usize / 2;
                let lr = gp.row as usize / 2;

                let neighbors: Vec<(i32, i32, KnownCell)> = [(-1i32, 0i32), (1, 0), (0, -1), (0, 1)]
                    .iter()
                    .filter_map(|&(dc, dr)| {
                        let c = gp.col as i32 + dc;
                        let r = gp.row as i32 + dr;
                        if c >= 0 && r >= 0 && c < GRID_SIZE as i32 && r < GRID_SIZE as i32 {
                            Some((c, r, sim.robot.known_map.cells[r as usize][c as usize]))
                        } else {
                            None
                        }
                    })
                    .collect();

                println!(
                    "seed={seed} STUCK tick={total_ticks} pos=({:.3},{:.3}) phys=({},{}) logical=({lc},{lr}) state={:?} known={:?} maze={:?}",
                    sim.robot.position.x, sim.robot.position.y,
                    gp.col, gp.row, sim.robot.state,
                    known_at_robot, maze_at_robot
                );
                println!("  cardinal known neighbours: {:?}", neighbors);

                // Show FreeSpace cells nearby
                let mut free_cells: Vec<String> = Vec::new();
                for r in (gp.row as usize).saturating_sub(3)..=(gp.row as usize + 3).min(GRID_SIZE-1) {
                    for c in (gp.col as usize).saturating_sub(3)..=(gp.col as usize + 3).min(GRID_SIZE-1) {
                        if sim.robot.known_map.cells[r][c] == KnownCell::FreeSpace {
                            free_cells.push(format!("p({c},{r})l({}/{})", c/2, r/2));
                        }
                    }
                }
                println!("  FreeSpace within 3 cells: {:?}", free_cells);

                match &sim.robot.current_path {
                    None => println!("  current_path: None"),
                    Some(p) => {
                        println!("  current_path waypoints: {}", p.waypoints.len());
                        for (i, wp) in p.waypoints.iter().take(6).enumerate() {
                            let wgp: GridPos = (*wp).into();
                            let wk = sim.robot.known_map.cells[wgp.row as usize][wgp.col as usize];
                            let wm = sim.maze.grid[wgp.row as usize][wgp.col as usize];
                            println!(
                                "    wp[{i}] ({:.3},{:.3}) phys=({},{}) logical=({}/{}) known={:?} maze={:?}",
                                wp.x, wp.y, wgp.col, wgp.row, wgp.col/2, wgp.row/2, wk, wm
                            );
                        }
                    }
                }
                break;
            }
        }
        if sim.robot.state != RobotState::Arrived && stuck_ticks < 10 {
            println!("seed={seed} still running at tick={total_ticks} state={:?}", sim.robot.state);
        }
    }
}

#[test]
fn tick_updates_known_map_after_start() {
    use path_generation::robot::types::KnownCell;
    let mut sim = SimulationState::restart(Some(42));
    sim.start();
    sim.tick(20);
    // At least one cell should be marked FreeSpace after a scan.
    let any_free = sim
        .robot
        .known_map
        .cells
        .iter()
        .flat_map(|row| row.iter())
        .any(|&c| c == KnownCell::FreeSpace);
    assert!(any_free, "known map should have FreeSpace cells after tick");
}

#[test]
fn start_transitions_state_to_exploring() {
    let mut sim = SimulationState::restart(Some(42));
    assert_eq!(sim.robot.state, RobotState::Idle);
    sim.start();
    assert_eq!(sim.robot.state, RobotState::Exploring);
}
