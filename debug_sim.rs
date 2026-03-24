use path_generation::maze::types::GridPos;
use path_generation::robot::types::{KnownCell, RobotState};
use path_generation::simulation::SimulationState;

fn main() {
    // Try several seeds and see how many ticks until stop
    for seed in [0u64, 1, 2, 42, 100, 999, 12345] {
        let mut sim = SimulationState::restart(Some(seed));
        sim.start();
        let mut last_pos = sim.robot.position;
        let mut stuck_ticks = 0;
        let mut total_ticks = 0;
        for _ in 0..5000 {
            sim.tick(20);
            total_ticks += 1;
            let dx = sim.robot.position.x - last_pos.x;
            let dy = sim.robot.position.y - last_pos.y;
            if (dx*dx+dy*dy).sqrt() < 0.001 {
                stuck_ticks += 1;
            } else {
                stuck_ticks = 0;
            }
            last_pos = sim.robot.position;
            if stuck_ticks >= 5 {
                // Print state
                let gp: GridPos = sim.robot.position.into();
                let cell = sim.robot.known_map.cells[gp.row as usize][gp.col as usize];
                let adj: Vec<_> = [(-1i32,0),(1,0),(0,-1i32),(0,1)].iter().map(|(dc,dr)| {
                    let c = (gp.col as i32 + dc) as usize;
                    let r = (gp.row as i32 + dr) as usize;
                    sim.robot.known_map.cells[r][c]
                }).collect();
                println!("seed={seed} stuck at tick={total_ticks} pos=({:.3},{:.3}) gp=({},{}) state={:?} cell={:?} neighbors={:?}",
                    sim.robot.position.x, sim.robot.position.y,
                    gp.col, gp.row, sim.robot.state, cell, adj);
                // Check maze around that position
                let mc = sim.maze.grid[gp.row as usize][gp.col as usize];
                println!("  maze cell at robot pos: {:?}", mc);
                // Check path
                if let Some(ref p) = sim.robot.current_path {
                    println!("  path waypoints: {}", p.waypoints.len());
                    for (i, wp) in p.waypoints.iter().take(5).enumerate() {
                        let wgp: GridPos = (*wp).into();
                        let wc = sim.robot.known_map.cells[wgp.row as usize][wgp.col as usize];
                        println!("    wp[{}] ({:.3},{:.3}) gp=({},{}) known={:?}", i, wp.x, wp.y, wgp.col, wgp.row, wc);
                    }
                } else {
                    println!("  path: None");
                }
                break;
            }
            if sim.robot.state == RobotState::Arrived {
                println!("seed={seed} ARRIVED at tick={total_ticks}");
                break;
            }
        }
        if stuck_ticks < 5 && sim.robot.state != RobotState::Arrived {
            println!("seed={seed} still moving after {total_ticks} ticks, state={:?}", sim.robot.state);
        }
    }
}
