use crate::maze::{CELL_SIZE, GRID_SIZE, WorldPos};
use crate::robot::types::{KnownCell, Path, Robot, RobotState};
use crate::sensor::LrfScan;

const ARRIVAL_THRESHOLD: f32 = 0.05; // meters

impl Robot {
    /// Update the robot's known map from one LRF scan.
    ///
    /// For each ray, all cells between the scan origin and the endpoint are
    /// marked FreeSpace. If the ray hit a wall, that wall cell is marked Wall.
    /// Cells that are already non-Unknown are not overwritten (static environment).
    pub fn update_map(&mut self, scan: &LrfScan) {
        for (i, pt) in scan.points.iter().enumerate() {
            let dx = pt.x - scan.origin.x;
            let dy = pt.y - scan.origin.y;
            let dist_world = (dx * dx + dy * dy).sqrt();
            if dist_world < 1e-7 {
                continue;
            }
            let wdx = dx / dist_world;
            let wdy = dy / dist_world;
            let total_cells = dist_world / CELL_SIZE;

            let rx = scan.origin.x / CELL_SIZE + 120.5;
            let ry = 120.5 - scan.origin.y / CELL_SIZE;

            // Grid direction (row is Y-down).
            let gdx = wdx;
            let gdy = -wdy;

            let step_x: i32 = if gdx >= 0.0 { 1 } else { -1 };
            let step_y: i32 = if gdy >= 0.0 { 1 } else { -1 };

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

            loop {
                if map_x >= 0 && map_y >= 0 && map_x < GRID_SIZE as i32 && map_y < GRID_SIZE as i32
                {
                    let cell = &mut self.known_map.cells[map_y as usize][map_x as usize];
                    if *cell == KnownCell::Unknown {
                        *cell = KnownCell::FreeSpace;
                    }
                }

                let t_next = tmax_x.min(tmax_y);
                if t_next >= total_cells {
                    if scan.is_wall_hit[i] {
                        let (wall_x, wall_y) = if tmax_x <= tmax_y {
                            (map_x + step_x, map_y)
                        } else {
                            (map_x, map_y + step_y)
                        };
                        if wall_x >= 0
                            && wall_y >= 0
                            && wall_x < GRID_SIZE as i32
                            && wall_y < GRID_SIZE as i32
                        {
                            let cell = &mut self.known_map.cells[wall_y as usize][wall_x as usize];
                            if *cell == KnownCell::Unknown {
                                *cell = KnownCell::Wall;
                            }
                        }
                    }
                    break;
                }

                if tmax_x <= tmax_y {
                    tmax_x += inv_x;
                    map_x += step_x;
                } else {
                    tmax_y += inv_y;
                    map_y += step_y;
                }
            }
        }
    }

    /// Move the robot along `path` for `delta_ms` milliseconds at 1 m/s.
    ///
    /// Transitions to Arrived when within ARRIVAL_THRESHOLD of the goal.
    /// Does nothing when state is Idle or Arrived.
    pub fn update(&mut self, delta_ms: u64, path: &Option<Path>) {
        if self.state == RobotState::Idle || self.state == RobotState::Arrived {
            return;
        }
        let Some(p) = path else { return };
        if p.waypoints.is_empty() {
            return;
        }

        let speed_mps = 1.0_f32;
        let dt_s = delta_ms as f32 / 1000.0;
        let mut remaining = speed_mps * dt_s;

        for &wp in &p.waypoints {
            let dx = wp.x - self.position.x;
            let dy = wp.y - self.position.y;
            let dist = (dx * dx + dy * dy).sqrt();
            if dist <= ARRIVAL_THRESHOLD {
                continue;
            }
            let ux = dx / dist;
            let uy = dy / dist;
            let step = remaining.min(dist);
            self.position.x += ux * step;
            self.position.y += uy * step;
            self.velocity = (ux * speed_mps, uy * speed_mps);
            remaining -= step;
            if remaining <= 0.0 {
                break;
            }
        }

        // Check goal arrival.
        let gw = WorldPos::from(self.goal);
        let gdx = gw.x - self.position.x;
        let gdy = gw.y - self.position.y;
        if (gdx * gdx + gdy * gdy).sqrt() <= ARRIVAL_THRESHOLD {
            self.state = RobotState::Arrived;
        }
    }
}
