// T019, T031: simulation restart and tick integration tests
use path_generation::maze::types::{GRID_SIZE, GridPos};
use path_generation::robot::types::RobotState;
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
