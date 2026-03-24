use std::f32::consts::PI;

use crate::maze::types::{GridPos, WorldPos};
use crate::maze::{CELL_SIZE, CellType, GRID_SIZE, Maze};

const NUM_RAYS: usize = 720;
// 360 deg / 720 rays = 0.5 deg step
const ANGLE_STEP: f32 = 2.0 * PI / NUM_RAYS as f32;
const MAX_RANGE_M: f32 = 5.0;
const MAX_RANGE_CELLS: f32 = MAX_RANGE_M / CELL_SIZE; // 100.0 cells

/// Point cloud from one LRF scan (720 rays: 360 deg / 0.5 deg step, 5 m max range).
pub struct LrfScan {
    pub origin: WorldPos,
    /// One point per ray. Either at the wall surface (entry boundary) or at max range.
    pub points: Vec<WorldPos>,
    /// Parallel to `points`. `true` = ray hit a wall; `false` = reached max range.
    pub is_wall_hit: Vec<bool>,
    /// Parallel to `points`. `Some(gp)` = the grid cell that was hit (wall); `None` = max range.
    pub hit_cells: Vec<Option<GridPos>>,
    pub timestamp_ms: u64,
}

pub struct Lrf;

impl Lrf {
    /// Fire 720 rays from `robot_pos` using Digital Differential Analyzer raycasting.
    ///
    /// Coordinate notes:
    ///   Grid-space fractional position: rx = x/CELL_SIZE + 120.5
    ///   (the +0.5 shift places the world center (0,0) at rx=120.5 = center of cell 120)
    ///   Grid row direction is Y-down, so grid_dy = -world_dy.
    pub fn scan(robot_pos: WorldPos, maze: &Maze) -> LrfScan {
        let mut points = Vec::with_capacity(NUM_RAYS);
        let mut is_wall_hit = Vec::with_capacity(NUM_RAYS);
        let mut hit_cells: Vec<Option<GridPos>> = Vec::with_capacity(NUM_RAYS);

        let rx = robot_pos.x / CELL_SIZE + 120.5;
        let ry = 120.5 - robot_pos.y / CELL_SIZE;

        for i in 0..NUM_RAYS {
            let angle = i as f32 * ANGLE_STEP;
            // World-space unit direction (Y-up).
            let wdx = angle.cos();
            let wdy = angle.sin();
            // Grid-space direction: col follows X (same), row follows -Y (inverted).
            let gdx = wdx;
            let gdy = -wdy;

            let step_x: i32 = if gdx >= 0.0 { 1 } else { -1 };
            let step_y: i32 = if gdy >= 0.0 { 1 } else { -1 };

            // Per-cell step length along the ray.
            let inv_x = if gdx.abs() < 1e-7 {
                f32::MAX
            } else {
                1.0 / gdx.abs()
            };
            let inv_y = if gdy.abs() < 1e-7 {
                f32::MAX
            } else {
                1.0 / gdy.abs()
            };

            // Distance along ray to first boundary in each axis.
            let mut tmax_x = if gdx.abs() < 1e-7 {
                f32::MAX
            } else if gdx > 0.0 {
                (rx.floor() + 1.0 - rx) * inv_x
            } else {
                (rx - rx.floor()) * inv_x
            };
            let mut tmax_y = if gdy.abs() < 1e-7 {
                f32::MAX
            } else if gdy > 0.0 {
                (ry.floor() + 1.0 - ry) * inv_y
            } else {
                (ry - ry.floor()) * inv_y
            };

            let mut map_x = rx.floor() as i32;
            let mut map_y = ry.floor() as i32;
            // t = distance (in cells) at which we entered the current cell.
            let mut t = 0.0_f32;

            loop {
                // Out-of-grid: treat as max-range miss.
                if map_x < 0 || map_y < 0 || map_x >= GRID_SIZE as i32 || map_y >= GRID_SIZE as i32
                {
                    points.push(WorldPos {
                        x: robot_pos.x + wdx * t * CELL_SIZE,
                        y: robot_pos.y + wdy * t * CELL_SIZE,
                    });
                    is_wall_hit.push(false);
                    hit_cells.push(None);
                    break;
                }

                // Wall hit: record point at the cell-entry boundary.
                if maze.grid[map_y as usize][map_x as usize] == CellType::Wall {
                    points.push(WorldPos {
                        x: robot_pos.x + wdx * t * CELL_SIZE,
                        y: robot_pos.y + wdy * t * CELL_SIZE,
                    });
                    is_wall_hit.push(true);
                    hit_cells.push(Some(GridPos {
                        col: map_x as u16,
                        row: map_y as u16,
                    }));
                    break;
                }

                // Check max range before advancing.
                let t_next = tmax_x.min(tmax_y);
                if t_next >= MAX_RANGE_CELLS {
                    points.push(WorldPos {
                        x: robot_pos.x + wdx * MAX_RANGE_CELLS * CELL_SIZE,
                        y: robot_pos.y + wdy * MAX_RANGE_CELLS * CELL_SIZE,
                    });
                    is_wall_hit.push(false);
                    hit_cells.push(None);
                    break;
                }

                // Advance DDA to next cell.
                if tmax_x <= tmax_y {
                    t = tmax_x;
                    tmax_x += inv_x;
                    map_x += step_x;
                } else {
                    t = tmax_y;
                    tmax_y += inv_y;
                    map_y += step_y;
                }
            }
        }

        LrfScan {
            origin: robot_pos,
            points,
            is_wall_hit,
            hit_cells,
            timestamp_ms: 0,
        }
    }
}
