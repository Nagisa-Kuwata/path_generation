# path_generation

Real-time robot maze exploration and path-generation simulator written in Rust.

A robot is placed in a procedurally generated 240x240 maze and autonomously
explores it using LRF (Laser Range Finder) sensing, A* path planning, and
frontier-based navigation.

---

## Requirements

- [Rust](https://www.rust-lang.org/tools/install) stable channel, MSRV 1.85+

Verify your toolchain:

```sh
rustc --version   # >= 1.85.0
cargo --version
```

---

## Build

```sh
# debug
cargo build

# release (recommended for interactive use)
cargo build --release
```

---

## Run

```sh
# run with a random seed
cargo run --release

# run with a fixed seed for reproducible mazes
cargo run --release -- --seed 42
```

### `--seed` option

| Argument | Type | Description |
|----------|------|-------------|
| `--seed <N>` | `u64` | Seed value for the maze PRNG. Omitting the flag picks a random seed. |

Using the same seed always generates the identical maze, which is useful for
repeatable demonstrations and debugging.

---

## Test

```sh
cargo test
```

All 37 integration tests and unit tests must pass on every commit.

---

## Benchmark

```sh
cargo bench
```

Key results (reference hardware):

| Benchmark | Measured | Target |
|-----------|----------|--------|
| LRF scan 240x240 | ~30 us | < 20 ms |
| A* plan 240x240 | ~2 ms | < 20 ms |

---

## Documentation

```sh
cargo doc --no-deps --open
```

All public APIs carry `///` doc comments; the generated docs are hosted under
`target/doc/path_generation/index.html`.

---

## Project Structure

```
src/
  maze/          # grid types and recursive-backtracker generator
  sensor/        # LRF simulation via DDA raycasting
  robot/         # robot state, known map, movement controller
  planner/       # A* path planner and frontier finder
  simulation/    # top-level simulation engine (restart / tick)
  ui/            # egui/eframe interactive visualizer
  lib.rs         # crate root
  main.rs        # binary entry point
benches/         # criterion benchmarks
tests/           # integration tests
specs/           # project specifications
```

---

## License

This project is private and not yet published under an open-source license.
