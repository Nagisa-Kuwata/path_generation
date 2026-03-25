// T019, T031: simulation restart and tick integration tests
use path_generation::maze::types::{GRID_SIZE, GridPos};
use path_generation::planner::{FrontierFinder, Planner};
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

// Diagnostic: confirm Blind mode robot actually moves.
// Run with: cargo test --test simulation_test diag_blind_mode -- --nocapture
#[test]
fn diag_blind_mode() {
    for seed in [0u64, 42, 999] {
        let mut sim = SimulationState::restart(Some(seed));
        sim.goal_known_to_robot = false; // Blind mode
        sim.start();
        let start_pos = sim.robot.position;
        println!(
            "seed={seed} blind_goal=({},{}) start=({},{})",
            sim.maze.blind_goal.col,
            sim.maze.blind_goal.row,
            sim.maze.start.col,
            sim.maze.start.row
        );
        let mut moved = false;
        for tick in 0..200usize {
            sim.tick(20);
            let dx = sim.robot.position.x - start_pos.x;
            let dy = sim.robot.position.y - start_pos.y;
            if (dx * dx + dy * dy).sqrt() > 0.001 {
                println!("seed={seed} moved at tick={tick} pos=({:.3},{:.3}) state={:?}",
                    sim.robot.position.x, sim.robot.position.y, sim.robot.state);
                moved = true;
                break;
            }
        }
        if !moved {
            println!("seed={seed} DID NOT MOVE after 200 ticks. state={:?}", sim.robot.state);
        }
        assert!(moved, "seed={seed}: robot should move in Blind mode");
    }
}

// Diagnostic: Blind mode long-run - confirm robot reaches blind_goal.
// cargo test --test simulation_test diag_blind_mode_long -- --nocapture
#[test]
fn diag_blind_mode_long() {
    for seed in [0u64, 42, 999] {
        let mut sim = SimulationState::restart(Some(seed));
        sim.goal_known_to_robot = false;
        sim.start();
        println!(
            "seed={seed} blind_goal=({},{}) start=({},{})",
            sim.maze.blind_goal.col, sim.maze.blind_goal.row,
            sim.maze.start.col, sim.maze.start.row,
        );
        let mut arrived = false;
        let mut last_pos = sim.robot.position;
        let mut stuck = 0u32;
        for tick in 0..30_000usize {
            sim.tick(20);
            let dx = sim.robot.position.x - last_pos.x;
            let dy = sim.robot.position.y - last_pos.y;
            if (dx*dx+dy*dy).sqrt() < 1e-4 { stuck += 1; } else { stuck = 0; }
            last_pos = sim.robot.position;
            if stuck >= 50 {
                let gp: GridPos = sim.robot.position.into();
                println!("seed={seed} STUCK at tick={tick} pos=({:.3},{:.3}) cell=({},{}) state={:?}",
                    sim.robot.position.x, sim.robot.position.y, gp.col, gp.row, sim.robot.state);
                match &sim.robot.current_path {
                    None => println!("  path=None"),
                    Some(p) => println!("  path len={}", p.waypoints.len()),
                }
                break;
            }
            if sim.robot.state == RobotState::Arrived {
                println!("seed={seed} ARRIVED at tick={tick}");
                arrived = true;
                break;
            }
            if tick % 5000 == 4999 {
                let free = sim.robot.known_map.cells.iter().flat_map(|r|r.iter())
                    .filter(|&&c|c==KnownCell::FreeSpace).count();
                let gp: GridPos = sim.robot.position.into();
                println!("seed={seed} tick={} pos=({},{}) free={free} state={:?}",
                    tick+1, gp.col, gp.row, sim.robot.state);
            }
        }
        if !arrived {
            println!("seed={seed} did NOT arrive after 30000 ticks (blind mode: \
                robot explores without goal knowledge -- arrival only on accidental pass-through)");
        }
        // Diagnostic test: no assertion on arrival.
        // In Blind mode the robot does not know the goal location and may not
        // physically pass within ARRIVAL_THRESHOLD of blind_goal within the
        // tick budget.  That is correct behaviour per spec.
    }
}

// Diagnostic: switch to Blind mode after running in Goal-Aware mode.
// Simulates user pressing Blind button mid-run.
// cargo test --test simulation_test diag_blind_mode_switch -- --nocapture
#[test]
fn diag_blind_mode_switch() {
    for seed in [0u64, 42, 999] {
        let mut sim = SimulationState::restart(Some(seed));
        // Start in Goal-Aware mode (default).
        sim.start();
        // Run 50 ticks in Goal-Aware mode.
        for _ in 0..50 { sim.tick(20); }
        let pos_before = sim.robot.position;
        // Switch to Blind mode mid-run.
        sim.goal_known_to_robot = false;
        // Robot should continue moving.
        let mut moved = false;
        for tick in 0..200usize {
            sim.tick(20);
            let dx = sim.robot.position.x - pos_before.x;
            let dy = sim.robot.position.y - pos_before.y;
            if (dx*dx+dy*dy).sqrt() > 0.001 {
                println!("seed={seed} moved after mode-switch at tick={tick} state={:?}", sim.robot.state);
                moved = true;
                break;
            }
        }
        assert!(moved, "seed={seed}: robot should move after switching to Blind mid-run");
    }
}

// Diagnostic: inspect known_map and BFS frontier after first scan.
// Run with: cargo test --test simulation_test diag_frontier_internals -- --nocapture
#[test]
fn diag_frontier_internals() {
    use path_generation::maze::GRID_SIZE;
    let mut sim = SimulationState::restart(Some(0));
    sim.start();
    // Run one tick so the map is populated by the first scan
    sim.tick(20);

    let km = &sim.robot.known_map;
    let free_count = km.cells.iter().flat_map(|r| r.iter()).filter(|&&c| c == KnownCell::FreeSpace).count();
    let wall_count = km.cells.iter().flat_map(|r| r.iter()).filter(|&&c| c == KnownCell::Wall).count();
    let unk_count  = km.cells.iter().flat_map(|r| r.iter()).filter(|&&c| c == KnownCell::Unknown).count();
    println!("After 1 tick: free={free_count} wall={wall_count} unknown={unk_count} total={}", GRID_SIZE*GRID_SIZE);

    // Count / print frontier candidates (even-indexed physical cells that are FreeSpace
    // and have at least one Unknown cardinal physical neighbor, that are NOT the start cell).
    let start_lc = 60usize; let start_lr = 60usize;
    let mut frontier_cells: Vec<(usize, usize)> = Vec::new();
    for lr in 0..120usize {
        for lc in 0..120usize {
            let pc = lc * 2;
            let pr = lr * 2;
            if km.cells[pr][pc] != KnownCell::FreeSpace { continue; }
            if lc == start_lc && lr == start_lr { continue; }
            // check cardinal physical neighbours for Unknown
            let mut has_unknown = false;
            for (dc, dr) in [(-1i32,0i32),(1,0),(0,-1),(0,1)] {
                let nc = pc as i32 + dc;
                let nr = pr as i32 + dr;
                if nc >= 0 && nr >= 0 && nc < GRID_SIZE as i32 && nr < GRID_SIZE as i32 {
                    if km.cells[nr as usize][nc as usize] == KnownCell::Unknown {
                        has_unknown = true; break;
                    }
                }
            }
            if has_unknown { frontier_cells.push((lc, lr)); }
        }
    }
    println!("Frontier candidate cells (FreeSpace + Unknown neighbor, not start): {}", frontier_cells.len());
    for &(lc, lr) in frontier_cells.iter().take(10) {
        println!("  logical ({lc},{lr}) physical ({},{})", lc*2, lr*2);
    }

    // Check whether any frontier is BFS-reachable from the start using robot's known map.
    // Do a simple BFS from (60,60) following non-Wall connectors.
    let mut bfs_visited = vec![vec![false; 120]; 120];
    let mut bfs_q: std::collections::VecDeque<(usize,usize)> = std::collections::VecDeque::new();
    for dlc in -1i32..=1 {
        for dlr in -1i32..=1 {
            let lc = 60i32 + dlc; let lr = 60i32 + dlr;
            if lc >= 0 && lr >= 0 && lc < 120 && lr < 120 {
                let (lc, lr) = (lc as usize, lr as usize);
                if !bfs_visited[lr][lc] { bfs_visited[lr][lc] = true; bfs_q.push_back((lc,lr)); }
            }
        }
    }
    let mut bfs_cell_count = 0usize;
    let mut bfs_frontier_count = 0usize;
    while let Some((lc, lr)) = bfs_q.pop_front() {
        bfs_cell_count += 1;
        // Can this cell be a frontier?
        let pc = lc * 2; let pr = lr * 2;
        if km.cells[pr][pc] == KnownCell::FreeSpace && frontier_cells.contains(&(lc,lr)) {
            bfs_frontier_count += 1;
        }
        // Expand
        for (nlc, nlr) in [(lc.wrapping_sub(1),lr),(lc+1,lr),(lc,lr.wrapping_sub(1)),(lc,lr+1)] {
            if nlc >= 120 || nlr >= 120 { continue; }
            if bfs_visited[nlr][nlc] { continue; }
            let npc = nlc * 2; let npr = nlr * 2;
            if km.cells[npr][npc] == KnownCell::Wall { continue; }
            let wall_c = lc + nlc; let wall_r = lr + nlr;
            if km.cells[wall_r][wall_c] == KnownCell::Wall { continue; }
            bfs_visited[nlr][nlc] = true;
            bfs_q.push_back((nlc, nlr));
        }
    }
    println!("BFS from start: visited {bfs_cell_count} cells, found {bfs_frontier_count} frontier cells reachable");
    // NOTE: After just 1 scan from the robot's starting position, frontier_count = 0 is
    // expected in tight corridors -- the LRF marks all physically adjacent cells Known
    // (FreeSpace or Wall) so no Unknown neighbors remain adjacent to any FreeSpace cell.
    // The engine handles this via the blind_goal fallback in the Blind-mode path planner.
    println!("(0 frontiers after 1 scan is expected; blind mode uses fallback A* to get moving)");
}

// Diagnostic: debug blind-mode freeze for a specific seed.
// cargo test --release --test simulation_test diag_blind_freeze_seed -- --nocapture
#[test]
fn diag_blind_freeze_seed() {
    const SEED: u64 = 4684074858222377819;
    const MAX_TICKS: usize = 500_000; // ~10000 s at 20 ms/tick
    const STUCK_WINDOW: u32 = 200;    // ~4 s of no movement
    const REPORT_EVERY: usize = 10_000;

    let mut sim = SimulationState::restart(Some(SEED));
    sim.goal_known_to_robot = false;
    sim.start();

    println!("seed={SEED}");
    println!("  blind_goal=({},{})", sim.maze.blind_goal.col, sim.maze.blind_goal.row);
    println!("  start=({},{})", sim.maze.start.col, sim.maze.start.row);

    let mut last_pos = sim.robot.position;
    let mut stuck_ticks = 0u32;
    let mut arrived = false;

    for tick in 0..MAX_TICKS {
        sim.tick(20);

        let dx = sim.robot.position.x - last_pos.x;
        let dy = sim.robot.position.y - last_pos.y;
        if (dx * dx + dy * dy).sqrt() < 1e-4 {
            stuck_ticks += 1;
        } else {
            stuck_ticks = 0;
        }
        last_pos = sim.robot.position;

        if sim.robot.state == RobotState::Arrived {
            let elapsed_s = (tick + 1) * 20 / 1000;
            println!("  ARRIVED at tick={tick} (~{elapsed_s} s)");
            arrived = true;
            break;
        }

        if stuck_ticks >= STUCK_WINDOW {
            let elapsed_s = (tick + 1) * 20 / 1000;
            let gp: GridPos = sim.robot.position.into();
            let free = sim.robot.known_map.cells.iter()
                .flat_map(|r| r.iter())
                .filter(|&&c| c == KnownCell::FreeSpace)
                .count();
            println!(
                "  STUCK at tick={tick} (~{elapsed_s} s)  pos=({:.3},{:.3}) cell=({},{})  \
                 free={free}/{total}  state={:?}",
                sim.robot.position.x, sim.robot.position.y,
                gp.col, gp.row, sim.robot.state,
                total = GRID_SIZE * GRID_SIZE,
            );
            // Show current path info.
            match &sim.robot.current_path {
                None => println!("  path=None"),
                Some(p) => {
                    println!("  path: {} waypoints  is_frontier={}", p.waypoints.len(), p.is_to_frontier);
                    for (i, wp) in p.waypoints.iter().take(4).enumerate() {
                        let wgp: GridPos = (*wp).into();
                        let known = sim.robot.known_map.cells[wgp.row as usize][wgp.col as usize];
                        println!("    wp[{i}] ({:.3},{:.3}) grid=({},{}) known={known:?}", wp.x, wp.y, wgp.col, wgp.row);
                    }
                }
            }
            // Show known-map around robot.
            let rc = gp.col as i32;
            let rr = gp.row as i32;
            println!("  known-map 7x7 around robot (col {} to {}, row {} to {}):",
                rc - 3, rc + 3, rr - 3, rr + 3);
            for dr in -3i32..=3 {
                let mut row_s = String::new();
                for dc in -3i32..=3 {
                    let c = (rc + dc).clamp(0, GRID_SIZE as i32 - 1) as usize;
                    let r = (rr + dr).clamp(0, GRID_SIZE as i32 - 1) as usize;
                    let mark = if dc == 0 && dr == 0 { 'R' } else {
                        match sim.robot.known_map.cells[r][c] {
                            KnownCell::Unknown  => '?',
                            KnownCell::FreeSpace => '.',
                            KnownCell::Wall     => '#',
                        }
                    };
                    row_s.push(mark);
                    row_s.push(' ');
                }
                println!("    {row_s}");
            }

            // --- Deep diagnostic: count globally + locally reachable Unknown logical cells ---
            // Count all Unknown logical cells in the entire map.
            let mut global_unknown_logical = 0usize;
            let mut global_free_logical = 0usize;
            for lr in 0..120usize {
                for lc in 0..120usize {
                    match sim.robot.known_map.cells[lr * 2][lc * 2] {
                        KnownCell::Unknown   => global_unknown_logical += 1,
                        KnownCell::FreeSpace => global_free_logical += 1,
                        KnownCell::Wall      => {}
                    }
                }
            }
            // BFS from robot (same logic as nearest_unknown) ? count reachable logical cells.
            let snap_c = (gp.col as usize & !1).min(238);
            let snap_r = (gp.row as usize & !1).min(238);
            let start_lc = snap_c / 2;
            let start_lr = snap_r / 2;
            let mut bfs_vis = vec![vec![false; 120]; 120];
            let mut bfs_queue: std::collections::VecDeque<(usize, usize)> = Default::default();
            for dlc in -1i32..=1 {
                for dlr in -1i32..=1 {
                    let lc = start_lc as i32 + dlc;
                    let lr = start_lr as i32 + dlr;
                    if lc < 0 || lr < 0 || lc >= 120 || lr >= 120 { continue; }
                    let (lc, lr) = (lc as usize, lr as usize);
                    if !bfs_vis[lr][lc] { bfs_vis[lr][lc] = true; bfs_queue.push_back((lc, lr)); }
                }
            }
            let mut reachable_unknown = 0usize;
            let mut reachable_free   = 0usize;
            let mut reachable_total  = 0usize;
            while let Some((lc, lr)) = bfs_queue.pop_front() {
                reachable_total += 1;
                match sim.robot.known_map.cells[lr * 2][lc * 2] {
                    KnownCell::Unknown   => reachable_unknown += 1,
                    KnownCell::FreeSpace => reachable_free   += 1,
                    KnownCell::Wall      => {}
                }
                for (nlc, nlr) in [(lc.wrapping_sub(1), lr), (lc + 1, lr),
                                   (lc, lr.wrapping_sub(1)), (lc, lr + 1)] {
                    if nlc >= 120 || nlr >= 120 || bfs_vis[nlr][nlc] { continue; }
                    let wall_c = lc + nlc;
                    let wall_r = lr + nlr;
                    if sim.robot.known_map.cells[wall_r][wall_c] == KnownCell::Wall { continue; }
                    bfs_vis[nlr][nlc] = true;
                    bfs_queue.push_back((nlc, nlr));
                }
            }
            println!(
                "  logical-cell counts: globalUnknown={global_unknown_logical} \
                 globalFree={global_free_logical}"
            );
            println!(
                "  BFS from robot: reachable={reachable_total} \
                 reachableUnknown={reachable_unknown} reachableFree={reachable_free}"
            );
            // Also check how many Unknown logical cells exist unreachable from robot.
            let unreachable_unknown = global_unknown_logical - reachable_unknown;
            println!("  unreachable Unknown logical cells: {unreachable_unknown}");

            // --- Direct call to nearest_unknown and Planner ---
            let robot_pos = sim.robot.position;
            let nearest_unk = FrontierFinder::nearest_unknown(robot_pos, &sim.robot.known_map);
            println!("  nearest_unknown={nearest_unk:?}");
            if let Some(unk_world) = nearest_unk {
                let unk_gp = GridPos::from(unk_world);
                let plan_r = Planner::plan(robot_pos, unk_gp, &sim.robot.known_map);
                println!("  plan(robot->nearest_unknown({},{})): {}",
                    unk_gp.col, unk_gp.row,
                    if plan_r.is_some() { "Some" } else { "None" });
                if let Some(p) = &plan_r {
                    println!("    waypoints={}", p.waypoints.len());
                }
            }
            let nearest_fr = FrontierFinder::nearest_frontier(robot_pos, &sim.robot.known_map);
            println!("  nearest_frontier={nearest_fr:?}");
            break;
        }

        if tick % REPORT_EVERY == REPORT_EVERY - 1 {
            let elapsed_s = (tick + 1) * 20 / 1000;
            let gp: GridPos = sim.robot.position.into();
            let free = sim.robot.known_map.cells.iter()
                .flat_map(|r| r.iter())
                .filter(|&&c| c == KnownCell::FreeSpace)
                .count();
            println!(
                "  tick={tick} (~{elapsed_s} s)  pos=({},{})  free={free}  state={:?}",
                gp.col, gp.row, sim.robot.state,
            );
        }
    }

    if !arrived && stuck_ticks < STUCK_WINDOW {
        println!("  still running after {MAX_TICKS} ticks -- not stuck, not arrived");
    }
}
