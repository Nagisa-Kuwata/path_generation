use crate::maze::types::{GRID_SIZE, GridPos, WorldPos};

/// Cell state in the robot's known map.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum KnownCell {
    /// Not yet observed by the LRF.
    Unknown,
    /// Confirmed open space.
    FreeSpace,
    /// Confirmed wall.
    Wall,
}

/// The robot's internal map built incrementally from LRF scan observations.
pub struct KnownMap {
    /// Per-cell observation state; indexed as `cells[row][col]`.
    pub cells: Box<[[KnownCell; GRID_SIZE]; GRID_SIZE]>,
}

impl Default for KnownMap {
    fn default() -> Self {
        KnownMap {
            cells: Box::new([[KnownCell::Unknown; GRID_SIZE]; GRID_SIZE]),
        }
    }
}

/// High-level operational state of the robot.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RobotState {
    /// Waiting to start; `tick()` is a no-op in this state.
    Idle,
    /// Actively exploring unknown regions of the maze.
    Exploring,
    /// A path to the goal has been found; robot is following it.
    NavigatingToGoal,
    /// Robot has reached the goal (within the 0.05 m arrival threshold).
    Arrived,
}

/// A planned path expressed as an ordered sequence of waypoints.
#[derive(Debug, Clone)]
pub struct Path {
    /// Waypoints from the robot's current position toward the target.
    pub waypoints: Vec<WorldPos>,
    /// `true` when the path leads to an exploration frontier rather than the goal.
    pub is_to_frontier: bool,
}

/// Full robot entity: position, velocity, map, path, and state.
pub struct Robot {
    /// Current world position in metres.
    pub position: WorldPos,
    /// Current velocity vector `(vx, vy)` in m/s.
    pub velocity: (f32, f32),
    /// Incrementally built map from LRF observations.
    pub known_map: KnownMap,
    /// Active planned path, if any.
    pub current_path: Option<Path>,
    /// Current operational state.
    pub state: RobotState,
    /// Maze goal the robot is trying to reach.
    pub goal: GridPos,
}

impl Robot {
    /// Create a new robot at `start` heading toward `goal`, with all state reset.
    pub fn new(start: WorldPos, goal: GridPos) -> Self {
        Robot {
            position: start,
            velocity: (0.0, 0.0),
            known_map: KnownMap::default(),
            current_path: None,
            state: RobotState::Idle,
            goal,
        }
    }
}
