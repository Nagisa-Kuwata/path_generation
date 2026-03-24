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
        // Grid-space fractional position of the scan origin (same formula as lrf.rs).
        let rx0 = scan.origin.x / CELL_SIZE + 120.5;
        let ry0 = 120.5 - scan.origin.y / CELL_SIZE;

        for i in 0..scan.points.len() {
            // --- ray direction: use the actual LRF ray, not origin??target-center ---
            // This ensures the DDA visits exactly the same cells the LRF did,
            // avoiding mismatches between this loop and what hit_cells records.
            let pt = scan.points[i];
            let world_dx = pt.x - scan.origin.x;
            let world_dy = pt.y - scan.origin.y;
            let world_dist = (world_dx * world_dx + world_dy * world_dy).sqrt();
            if world_dist < 1e-7 {
                continue;
            }
            // Grid-space direction (row is Y-down).
            let gdx = world_dx / world_dist;
            let gdy = -(world_dy / world_dist);

            let step_x: i32 = if gdx >= 0.0 { 1 } else { -1 };
            let step_y: i32 = if gdy >= 0.0 { 1 } else { -1 };

            let inv_x = if gdx.abs() < 1e-7 { f32::MAX } else { 1.0 / gdx.abs() };
            let inv_y = if gdy.abs() < 1e-7 { f32::MAX } else { 1.0 / gdy.abs() };

            let mut tmax_x = if gdx.abs() < 1e-7 {
                f32::MAX
            } else if gdx > 0.0 {
                (rx0.floor() + 1.0 - rx0) * inv_x
            } else {
                (rx0 - rx0.floor()) * inv_x
            };
            let mut tmax_y = if gdy.abs() < 1e-7 {
                f32::MAX
            } else if gdy > 0.0 {
                (ry0.floor() + 1.0 - ry0) * inv_y
            } else {
                (ry0 - ry0.floor()) * inv_y
            };

            let mut map_x = rx0.floor() as i32;
            let mut map_y = ry0.floor() as i32;

            // For wall-hit rays the terminal cell is known exactly from hit_cells.
            // For max-range rays use a distance limit instead.
            let total_cells = world_dist / CELL_SIZE;

            loop {
                if map_x < 0 || map_y < 0
                    || map_x >= GRID_SIZE as i32 || map_y >= GRID_SIZE as i32
                {
                    break;
                }

                // Have we reached the wall cell recorded by the LRF?
                if scan.is_wall_hit[i] {
                    if let Some(hit) = scan.hit_cells[i] {
                        if map_x == hit.col as i32 && map_y == hit.row as i32 {
                            // Mark this cell Wall (only if not yet observed).
                            let cell = &mut self.known_map.cells[map_y as usize][map_x as usize];
                            if *cell == KnownCell::Unknown {
                                *cell = KnownCell::Wall;
                            }
                            break;
                        }
                    }
                }

                // Safety: DDA has gone past the endpoint distance ? stop.
                let t_next = tmax_x.min(tmax_y);
                if t_next > total_cells + 0.5 {
                    break;
                }

                // Current cell is before the wall ? it is free space.
                let cell = &mut self.known_map.cells[map_y as usize][map_x as usize];
                if *cell == KnownCell::Unknown {
                    *cell = KnownCell::FreeSpace;
                }

                // Advance DDA.
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

        // Check goal arrival before testing path availability: when the engine
        // suppresses the path (dist_to_goal <= threshold), the robot must still
        // transition to Arrived so the simulation does not freeze.
        let gw = WorldPos::from(self.goal);
        let gdx = gw.x - self.position.x;
        let gdy = gw.y - self.position.y;
        if (gdx * gdx + gdy * gdy).sqrt() <= ARRIVAL_THRESHOLD {
            self.state = RobotState::Arrived;
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

            // Guard: if we ended up at an (odd, odd) grid cell ? always a wall ?
            // snap back to whichever even axis is closer. This can happen due to
            // float rounding when the robot sits exactly on a cell boundary and
            // then a new path redirects it by a tiny amount in both axes.
            {
                use crate::maze::CELL_SIZE;
                let half = (crate::maze::GRID_SIZE as f32) / 2.0;
                let raw_col = (self.position.x / CELL_SIZE + half).round() as i32;
                let raw_row = (half - self.position.y / CELL_SIZE).round() as i32;
                if raw_col % 2 != 0 && raw_row % 2 != 0 {
                    // Both axes are odd ? snap the axis that requires less movement.
                    let col_to_even_dist = {
                        let even_col = ((raw_col + 1) / 2 * 2) as f32;
                        ((even_col - half) * CELL_SIZE - self.position.x).abs()
                    };
                    let row_to_even_dist = {
                        let even_row = ((raw_row + 1) / 2 * 2) as f32;
                        (self.position.y - (half - even_row) * CELL_SIZE).abs()
                    };
                    if col_to_even_dist <= row_to_even_dist {
                        let even_col = ((raw_col + 1) / 2 * 2) as f32;
                        self.position.x = (even_col - half) * CELL_SIZE;
                    } else {
                        let even_row = ((raw_row + 1) / 2 * 2) as f32;
                        self.position.y = (half - even_row) * CELL_SIZE;
                    }
                }
            }

            if remaining <= 0.0 {
                break;
            }
        }

    }
}
