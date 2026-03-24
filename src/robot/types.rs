use crate::maze::types::{GRID_SIZE, GridPos, WorldPos};

// Cell state in the robot's known map
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum KnownCell {
    Unknown,
    FreeSpace,
    Wall,
}

// Robot's internal map built from LRF observations
pub struct KnownMap {
    pub cells: Box<[[KnownCell; GRID_SIZE]; GRID_SIZE]>,
}

impl Default for KnownMap {
    fn default() -> Self {
        KnownMap {
            cells: Box::new([[KnownCell::Unknown; GRID_SIZE]; GRID_SIZE]),
        }
    }
}

// Robot's operational state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RobotState {
    Idle,
    Exploring,
    NavigatingToGoal,
    Arrived,
}

// Planned path as a sequence of waypoints
#[derive(Debug, Clone)]
pub struct Path {
    pub waypoints: Vec<WorldPos>,
    // true = heading toward exploration frontier, false = heading to goal directly
    pub is_to_frontier: bool,
}

// Robot state
pub struct Robot {
    pub position: WorldPos,
    // Velocity vector (normalised direction * speed m/s)
    pub velocity: (f32, f32),
    pub known_map: KnownMap,
    pub current_path: Option<Path>,
    pub state: RobotState,
    pub goal: GridPos,
}

impl Robot {
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
