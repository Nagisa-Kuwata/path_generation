use std::time::Instant;

use eframe::{App, Frame, egui};
use egui::{Color32, Pos2, Rect, Stroke, StrokeKind, Vec2};

use crate::maze::{CellType, GRID_SIZE, GridPos};
use crate::robot::types::{KnownCell, RobotState};
use crate::simulation::SimulationState;

// Pixels per grid cell. 240 cells * 3 px = 720 px square maze area.
const SCALE: f32 = 3.0;
// Robot half-extent in pixels (ROBOT_HALF=5 cells).
const ROBOT_HALF_PX: f32 = 5.0 * SCALE;

pub struct SimApp {
    sim: SimulationState,
    last_tick: Option<Instant>,
    maze_tex: Option<egui::TextureHandle>,
    map_tex: Option<egui::TextureHandle>,
    /// Simulation speed multiplier (1x ? 10x).
    speed_multiplier: u8,    /// Exploration mode persisted across Restart/Regenerate.
    goal_known_to_robot: bool,}

impl SimApp {
    pub fn new(seed: Option<u64>) -> Self {
        SimApp {
            sim: SimulationState::restart(seed),
            last_tick: None,
            maze_tex: None,
            map_tex: None,
            speed_multiplier: 1,
            goal_known_to_robot: true,
        }
    }

    fn build_maze_image(&self) -> egui::ColorImage {
        let mut bytes = vec![255u8; GRID_SIZE * GRID_SIZE * 4];
        for row in 0..GRID_SIZE {
            for col in 0..GRID_SIZE {
                let base = (row * GRID_SIZE + col) * 4;
                let (r, g, b, a) = match self.sim.maze.grid[row][col] {
                    CellType::Wall => (0u8, 0, 0, 255u8),
                    CellType::Passage => (255, 255, 255, 255),
                };
                bytes[base] = r;
                bytes[base + 1] = g;
                bytes[base + 2] = b;
                bytes[base + 3] = a;
            }
        }
        egui::ColorImage::from_rgba_unmultiplied([GRID_SIZE, GRID_SIZE], &bytes)
    }

    fn build_known_map_image(&self) -> egui::ColorImage {
        let mut bytes = vec![0u8; GRID_SIZE * GRID_SIZE * 4];
        for row in 0..GRID_SIZE {
            for col in 0..GRID_SIZE {
                let base = (row * GRID_SIZE + col) * 4;
                let (r, g, b, a) = match self.sim.robot.known_map.cells[row][col] {
                    KnownCell::Unknown => (128u8, 128, 128, 160u8),
                    KnownCell::FreeSpace => (0, 0, 0, 0), // transparent
                    KnownCell::Wall => (30, 30, 30, 180),
                };
                bytes[base] = r;
                bytes[base + 1] = g;
                bytes[base + 2] = b;
                bytes[base + 3] = a;
            }
        }
        egui::ColorImage::from_rgba_unmultiplied([GRID_SIZE, GRID_SIZE], &bytes)
    }

    fn grid_pos_to_screen(origin: Pos2, col: u16, row: u16) -> Pos2 {
        Pos2::new(origin.x + col as f32 * SCALE, origin.y + row as f32 * SCALE)
    }
}

impl App for SimApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut Frame) {
        // Advance simulation if active.
        let is_active =
            self.sim.robot.state != RobotState::Idle && self.sim.robot.state != RobotState::Arrived;

        if is_active {
            let now = Instant::now();
            let delta_ms = self
                .last_tick
                .map(|t| t.elapsed().as_millis() as u64)
                .unwrap_or(20)
                .min(100); // cap at 100 ms to avoid large jumps
            self.last_tick = Some(now);

            // Run `speed_multiplier` sub-ticks per frame, each with the same
            // delta_ms.  Total simulated time per frame = delta_ms * n, giving
            // a true Nx wall-clock speed-up while keeping each step small
            // enough that the robot does not skip waypoints.
            let n = self.speed_multiplier as u64;
            for _ in 0..n {
                if self.sim.robot.state != RobotState::Idle
                    && self.sim.robot.state != RobotState::Arrived
                {
                    self.sim.tick(delta_ms);
                }
            }
            ctx.request_repaint();
        }

        // --- Left control panel ---
        egui::SidePanel::left("controls")
            .min_width(130.0)
            .show(ctx, |ui| {
                ui.heading("Maze Robot");
                ui.separator();

                ui.label(format!("State: {:?}", self.sim.robot.state));
                ui.label(format!(
                    "Time: {:.1} s",
                    self.sim.elapsed_ms as f64 / 1000.0
                ));
                ui.label(format!(
                    "Pos:  ({:.2}, {:.2})",
                    self.sim.robot.position.x, self.sim.robot.position.y
                ));
                ui.separator();

                // Start button (enabled only when Idle).
                let can_start = self.sim.robot.state == RobotState::Idle;
                if ui
                    .add_enabled(can_start, egui::Button::new("Start"))
                    .clicked()
                {
                    self.sim.start();
                    self.last_tick = Some(Instant::now());
                }

                // Restart button (always enabled).
                if ui.button("Restart").clicked() {
                    let seed = self.sim.seed;
                    self.sim = SimulationState::restart(Some(seed));
                    self.sim.goal_known_to_robot = self.goal_known_to_robot;
                    self.last_tick = None;
                    self.maze_tex = None;
                }

                // Regenerate button: restart with a new random seed.
                if ui.button("Regenerate").clicked() {
                    self.sim = SimulationState::restart(None);
                    self.sim.goal_known_to_robot = self.goal_known_to_robot;
                    self.last_tick = None;
                    self.maze_tex = None;
                }

                ui.separator();
                ui.label("Mode:");
                // Mode toggle buttons ? highlighted when active.
                let goal_aware_btn = egui::Button::new("Goal-Aware")
                    .fill(if self.goal_known_to_robot {
                        egui::Color32::from_rgb(60, 130, 60)
                    } else {
                        egui::Color32::from_gray(55)
                    });
                if ui.add(goal_aware_btn).clicked() {
                    self.goal_known_to_robot = true;
                    self.sim.goal_known_to_robot = true;
                }
                let blind_btn = egui::Button::new("Blind")
                    .fill(if !self.goal_known_to_robot {
                        egui::Color32::from_rgb(130, 60, 60)
                    } else {
                        egui::Color32::from_gray(55)
                    });
                if ui.add(blind_btn).clicked() {
                    self.goal_known_to_robot = false;
                    self.sim.goal_known_to_robot = false;
                }

                ui.separator();
                ui.label("Speed:");
                ui.add(
                    egui::Slider::new(&mut self.speed_multiplier, 1u8..=10)
                        .suffix("x")
                        .integer()
                );

                ui.separator();
                ui.label(format!("Seed: {}", self.sim.seed));

                if self.sim.robot.state == RobotState::Arrived {
                    ui.separator();
                    ui.label("Goal reached!");
                }
            });

        // --- Main drawing panel ---
        egui::CentralPanel::default().show(ctx, |ui| {
            let grid_px = GRID_SIZE as f32 * SCALE;

            // Allocate a fixed-size area for the maze.
            let (resp, painter) =
                ui.allocate_painter(Vec2::new(grid_px, grid_px), egui::Sense::hover());
            let origin = resp.rect.min;
            let display_rect = resp.rect;
            let uv = Rect::from_min_max(Pos2::ZERO, Pos2::new(1.0, 1.0));

            // Layer 1: Maze (static texture; rebuild only once or after restart).
            let opts = egui::TextureOptions::NEAREST;
            if self.maze_tex.is_none() {
                let img = self.build_maze_image();
                self.maze_tex = Some(ctx.load_texture("maze", img, opts));
            }
            painter.image(
                self.maze_tex.as_ref().unwrap().id(),
                display_rect,
                uv,
                Color32::WHITE,
            );

            // Layer 2: Robot known-map overlay (updated each tick).
            let map_image = self.build_known_map_image();
            if let Some(t) = self.map_tex.as_mut() {
                t.set(map_image, opts);
            } else {
                self.map_tex = Some(ctx.load_texture("known_map", map_image, opts));
            }
            painter.image(
                self.map_tex.as_ref().unwrap().id(),
                display_rect,
                uv,
                Color32::WHITE,
            );

            // Layer 3: Start position marker (persistent red mark).
            {
                let sc = Self::grid_pos_to_screen(
                    origin,
                    self.sim.maze.start.col,
                    self.sim.maze.start.row,
                );
                let arm = SCALE * 2.0;
                let start_red = Color32::from_rgb(200, 30, 30);
                // Draw a filled small square.
                painter.rect_filled(
                    Rect::from_center_size(sc, Vec2::new(arm * 2.0, arm * 2.0)),
                    0.0,
                    start_red,
                );
            }

            // Layer 4: LRF point cloud (light green dots).
            if let Some(scan) = &self.sim.lrf_scan {
                for pt in &scan.points {
                    let gp = GridPos::from(*pt);
                    let pos = Self::grid_pos_to_screen(origin, gp.col, gp.row);
                    painter.circle_filled(pos, 1.0, Color32::from_rgb(80, 200, 80));
                }
            }

            // Layer 4: Planned path (red polyline).
            if let Some(path) = &self.sim.robot.current_path {
                let pts: Vec<Pos2> = path
                    .waypoints
                    .iter()
                    .map(|&wp| {
                        let gp = GridPos::from(wp);
                        Self::grid_pos_to_screen(origin, gp.col, gp.row)
                    })
                    .collect();
                if pts.len() >= 2 {
                    painter.add(egui::Shape::line(
                        pts,
                        Stroke::new(1.5, Color32::from_rgb(220, 40, 40)),
                    ));
                }
            }

            // Layer 5: Goal markers.
            // Goal-aware goal: orange X (always shown).
            {
                let gc = Self::grid_pos_to_screen(
                    origin,
                    self.sim.maze.goal.col,
                    self.sim.maze.goal.row,
                );
                let arm = SCALE * 2.0;
                let orange = Color32::from_rgb(255, 140, 0);
                painter.line_segment(
                    [
                        Pos2::new(gc.x - arm, gc.y - arm),
                        Pos2::new(gc.x + arm, gc.y + arm),
                    ],
                    Stroke::new(2.0, orange),
                );
                painter.line_segment(
                    [
                        Pos2::new(gc.x + arm, gc.y - arm),
                        Pos2::new(gc.x - arm, gc.y + arm),
                    ],
                    Stroke::new(2.0, orange),
                );
            }
            // Blind-mode goal: cyan diamond (always shown so the user can see it).
            {
                let bg = Self::grid_pos_to_screen(
                    origin,
                    self.sim.maze.blind_goal.col,
                    self.sim.maze.blind_goal.row,
                );
                let arm = SCALE * 2.0;
                let cyan = Color32::from_rgb(0, 220, 220);
                let pts = vec![
                    Pos2::new(bg.x, bg.y - arm),
                    Pos2::new(bg.x + arm, bg.y),
                    Pos2::new(bg.x, bg.y + arm),
                    Pos2::new(bg.x - arm, bg.y),
                    Pos2::new(bg.x, bg.y - arm),
                ];
                painter.add(egui::Shape::line(pts, Stroke::new(2.0, cyan)));
            }

            // Layer 6: Robot body rectangle.
            {
                let gp = GridPos::from(self.sim.robot.position);
                let center = Self::grid_pos_to_screen(origin, gp.col, gp.row);
                let body_color = if self.sim.robot.state == RobotState::Arrived {
                    Color32::from_rgb(80, 130, 255) // light blue on arrival
                } else {
                    Color32::from_rgb(255, 90, 90) // light red otherwise
                };
                painter.rect_stroke(
                    Rect::from_center_size(
                        center,
                        Vec2::new(ROBOT_HALF_PX * 2.0, ROBOT_HALF_PX * 2.0),
                    ),
                    0.0,
                    Stroke::new(2.0, body_color),
                    StrokeKind::Outside,
                );

                // Velocity indicator (white line in direction of travel).
                let (vx, vy) = self.sim.robot.velocity;
                let v_len = (vx * vx + vy * vy).sqrt();
                if v_len > 0.01 {
                    let arrow_len = ROBOT_HALF_PX * 1.5;
                    let end = Pos2::new(
                        center.x + (vx / v_len) * arrow_len,
                        center.y - (vy / v_len) * arrow_len, // screen Y inverted
                    );
                    painter.line_segment([center, end], Stroke::new(2.0, Color32::WHITE));
                }
            }
        });
    }
}
