use rand::SeedableRng;
use rand_chacha::ChaCha8Rng;

use crate::maze::generator::MazeGenerator;
use crate::maze::types::{GridPos, Maze, WorldPos};
use crate::planner::{FrontierFinder, Planner};
use crate::robot::types::{KnownCell, Robot, RobotState};
use crate::sensor::lrf::{Lrf, LrfScan};

/// Complete runtime state of the simulation.
pub struct SimulationState {
    /// The procedurally generated maze.
    pub maze: Maze,
    /// The simulated robot.
    pub robot: Robot,
    /// Most-recent LRF scan snapshot, available after the first `tick()`.
    pub lrf_scan: Option<LrfScan>,
    /// Total simulated time elapsed in milliseconds.
    pub elapsed_ms: u64,
    /// Seed used to generate the current maze.
    pub seed: u64,
    /// When `true` (goal-aware mode) the robot knows the goal location from
    /// the start and will navigate directly once the goal cell is in the map.
    /// When `false` (blind mode) the robot uses frontier exploration only and
    /// reaches the goal only if it happens to pass through it.
    pub goal_known_to_robot: bool,
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
            goal_known_to_robot: true, // default: goal-aware mode
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
        } else if self.goal_known_to_robot {
            // --- Goal-aware mode ---
            // Drive directly to the goal as soon as its cell is in the map;
            // fall back to frontier exploration while the goal is unknown.
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
        } else {
            // --- Blind mode ---
            // The robot does not know where the goal is. It explores using
            // frontier BFS only. `Arrived` is triggered by controller.rs when
            // the robot physically reaches the goal position.
            self.robot.state = RobotState::Exploring;
            FrontierFinder::nearest_frontier(self.robot.position, &self.robot.known_map)
                .and_then(|frontier| {
                    let gp = GridPos::from(frontier);
                    Planner::plan(self.robot.position, gp, &self.robot.known_map)
                })
        };

        // 4. Move robot.
        self.robot.update(delta_ms, &path);
        self.robot.current_path = path;

        // 5. Advance elapsed time.
        self.elapsed_ms += delta_ms;
    }
}
