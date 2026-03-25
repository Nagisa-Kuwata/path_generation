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
    /// Persistent exploration target used in Blind mode when no frontier is
    /// reachable.  Held across ticks until the robot arrives (avoids
    /// oscillation from picking a new nearest-Unknown cell every tick).
    wander_target: Option<GridPos>,
    /// Set to `true` the first time `nearest_unknown` returns `None` in
    /// Blind mode.  Once all reachable Unknown passage cells are gone the
    /// robot switches to a direct post-exploration path to `blind_goal`
    /// rather than continuing to chase permanent "frontiers" caused by
    /// Unknown connector cells that the LRF never managed to reach.
    exploration_complete: bool,
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
        // Always use maze.goal for the Robot, regardless of mode; in blind mode
        // the engine overrides which target it compares against at tick time.
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
            wander_target: None,
            exploration_complete: false,
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
        // In blind mode the effective goal is maze.blind_goal; in goal-aware
        // mode it is maze.goal.  Both share the same arrival threshold.
        let effective_goal = if self.goal_known_to_robot {
            self.maze.goal
        } else {
            self.maze.blind_goal
        };
        let goal_world = WorldPos::from(effective_goal);
        let dgx = goal_world.x - self.robot.position.x;
        let dgy = goal_world.y - self.robot.position.y;
        let dist_to_goal = (dgx * dgx + dgy * dgy).sqrt();

        // Sync robot.goal so that controller.rs arrival check uses the right target.
        self.robot.goal = effective_goal;

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
                        Planner::plan(self.robot.position, effective_goal, &self.robot.known_map)
                    })
            }
        } else {
            // --- Blind mode ---
            // The robot does not know where the goal is in advance.
            //
            // Priority 1 ? LRF visibility: if blind_goal is within the LRF
            // range (?5 m) and the straight-line ray reaches it without hitting
            // a wall, the robot has "spotted" the goal with its sensor and
            // navigates directly to it, ending the exploration phase early.
            //
            // Priority 2 ? Post-exploration: once all reachable Unknown passage
            // cells are exhausted (exploration_complete flag), navigate directly
            // to blind_goal.  This handles the case where the goal was mapped
            // remotely via LRF without the robot physically passing through it.
            //
            // Priority 3 ? Frontier exploration: normal BFS-based exploration
            // with wander_target fallback.
            if self.lrf_scan.as_ref().is_some_and(|scan| scan.detects_cell(self.maze.blind_goal)) {
                // Blind_goal is directly visible via LRF: navigate to it now.
                let direct = Planner::plan(
                    self.robot.position,
                    self.maze.blind_goal,
                    &self.robot.known_map,
                );
                if direct.is_some() {
                    self.robot.state = RobotState::NavigatingToGoal;
                }
                direct
            } else if self.exploration_complete {
                // Post-exploration: all reachable Unknown passage cells are
                // gone.  Navigate directly to blind_goal so the arrival check
                // fires.  Some FreeSpace cells were mapped remotely via LRF
                // without the robot physically passing through them; this step
                // closes that gap.  The exploration phase was entirely
                // goal-unaware; this is a completion step only.
                let direct = Planner::plan(
                    self.robot.position,
                    self.maze.blind_goal,
                    &self.robot.known_map,
                );
                if direct.is_some() {
                    self.robot.state = RobotState::NavigatingToGoal;
                }
                direct
            } else {
                self.robot.state = RobotState::Exploring;
                FrontierFinder::nearest_frontier(self.robot.position, &self.robot.known_map)
                    .and_then(|frontier| {
                        let gp = GridPos::from(frontier);
                        Planner::plan(self.robot.position, gp, &self.robot.known_map)
                    })
                .or_else(|| {
                    // Decide whether to refresh wander_target:
                    //  (a) None yet
                    //  (b) Robot arrived within 0.15 m of target
                    //  (c) Target cell is no longer Unknown (already explored)
                    let needs_refresh = match self.wander_target {
                        None => true,
                        Some(t) => {
                            // (b) proximity
                            let tw = WorldPos::from(t);
                            let dx = tw.x - self.robot.position.x;
                            let dy = tw.y - self.robot.position.y;
                            let close = (dx * dx + dy * dy).sqrt() < 0.15;
                            // (c) target already mapped
                            let already_known =
                                self.robot.known_map.cells[t.row as usize][t.col as usize]
                                    != KnownCell::Unknown;
                            close || already_known
                        }
                    };
                    if needs_refresh {
                        let unk = FrontierFinder::nearest_unknown(
                            self.robot.position,
                            &self.robot.known_map,
                        );
                        if unk.is_none() {
                            // No reachable Unknown passage cells remain.
                            // Mark exploration complete so the next tick
                            // switches to direct blind_goal navigation.
                            self.exploration_complete = true;
                        }
                        self.wander_target = unk.map(|w| GridPos::from(w));
                    }
                    let path_opt = self.wander_target.and_then(|target| {
                        Planner::plan(self.robot.position, target, &self.robot.known_map)
                    });
                    // (d) If A* returns None for the current target (confirmed
                    //     unreachable), invalidate it immediately so the next tick
                    //     picks a fresh nearest-Unknown instead of looping forever.
                    if path_opt.is_none() {
                        self.wander_target = None;
                    }
                    path_opt
                })
                .or_else(|| {
                    // Post-exploration fallback: all reachable Unknown cells are
                    // exhausted (frontier BFS and nearest_unknown both returned
                    // None).  The robot has mapped the entire accessible maze via
                    // LRF but may not have physically passed through blind_goal
                    // (long-range scans reveal cells without travelling to them).
                    // Navigate directly to blind_goal so the arrival check can
                    // fire.  The exploration phase was entirely goal-unaware;
                    // this is purely a completion step.
                    let direct = Planner::plan(
                        self.robot.position,
                        self.maze.blind_goal,
                        &self.robot.known_map,
                    );
                    if direct.is_some() {
                        self.robot.state = RobotState::NavigatingToGoal;
                    }
                    direct
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
