use rand::SeedableRng;
use rand_chacha::ChaCha8Rng;

use crate::maze::generator::MazeGenerator;
use crate::maze::types::{GridPos, Maze, WorldPos};
use crate::planner::{FrontierFinder, Planner};
use crate::robot::types::{KnownCell, Robot, RobotState};
use crate::sensor::lrf::{Lrf, LrfScan};

/// Full simulation state.
pub struct SimulationState {
    pub maze: Maze,
    pub robot: Robot,
    pub lrf_scan: Option<LrfScan>,
    pub elapsed_ms: u64,
    pub seed: u64,
}

impl SimulationState {
    /// Create or reset simulation with an optional seed.
    /// None = random seed, Some(s) = deterministic.
    pub fn restart(seed: Option<u64>) -> Self {
        let actual_seed = match seed {
            Some(s) => s,
            None => {
                let mut rng = ChaCha8Rng::from_entropy();
                rand::RngCore::next_u64(&mut rng)
            }
        };
        let maze = MazeGenerator::generate(actual_seed);
        let start_world = WorldPos::from(maze.start);
        let goal = maze.goal;
        let mut robot = Robot::new(start_world, goal);
        robot.state = RobotState::Idle;

        SimulationState {
            maze,
            robot,
            lrf_scan: None,
            elapsed_ms: 0,
            seed: actual_seed,
        }
    }

    /// Transition from Idle to Exploring, enabling tick() to run.
    pub fn start(&mut self) {
        if self.robot.state == RobotState::Idle {
            self.robot.state = RobotState::Exploring;
        }
    }

    /// Run one simulation cycle: scan -> map update -> replan -> move.
    ///
    /// Does nothing when robot is Idle or Arrived.
    pub fn tick(&mut self, delta_ms: u64) {
        if self.robot.state == RobotState::Idle || self.robot.state == RobotState::Arrived {
            return;
        }

        // 1. LRF scan.
        let scan = Lrf::scan(self.robot.position, &self.maze);

        // 2. Update known map.
        self.robot.update_map(&scan);
        self.lrf_scan = Some(scan);

        // 3. Compute path for this tick.
        let goal_world = WorldPos::from(self.maze.goal);
        let dgx = goal_world.x - self.robot.position.x;
        let dgy = goal_world.y - self.robot.position.y;
        let dist_to_goal = (dgx * dgx + dgy * dgy).sqrt();

        let path = if dist_to_goal <= 0.05 {
            None
        } else {
            let goal_cell = self.robot.known_map.cells[self.maze.goal.row as usize]
                [self.maze.goal.col as usize];

            let direct = if goal_cell != KnownCell::Unknown {
                Planner::plan(self.robot.position, self.maze.goal, &self.robot.known_map)
            } else {
                None
            };

            if let Some(p) = direct {
                self.robot.state = RobotState::NavigatingToGoal;
                Some(p)
            } else {
                FrontierFinder::nearest_frontier(self.robot.position, &self.robot.known_map)
                    .and_then(|frontier| {
                        self.robot.state = RobotState::Exploring;
                        let gp = GridPos::from(frontier);
                        Planner::plan(self.robot.position, gp, &self.robot.known_map)
                    })
                    .or_else(|| {
                        Planner::plan(self.robot.position, self.maze.goal, &self.robot.known_map)
                    })
            }
        };

        // 4. Move robot.
        self.robot.update(delta_ms, &path);
        self.robot.current_path = path;

        // 5. Advance elapsed time.
        self.elapsed_ms += delta_ms;
    }
}
