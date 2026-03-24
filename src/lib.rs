//! # path_generation
//!
//! Real-time robot maze exploration and path-generation simulator.
//!
//! ## Modules
//! - [`maze`] ? grid representation and procedural maze generation
//! - [`sensor`] ? LRF (Laser Range Finder) simulation via DDA raycasting
//! - [`robot`] ? robot state, known map, and movement controller
//! - [`planner`] ? A* path planner and frontier-based exploration
//! - [`simulation`] ? top-level simulation loop (`restart` / `start` / `tick`)
//! - [`ui`] ? egui/eframe application for interactive visualization

// expose modules for integration tests and library use
pub mod maze;
pub mod planner;
pub mod robot;
pub mod sensor;
pub mod simulation;
pub mod ui;

