use criterion::{Criterion, criterion_group, criterion_main};
use path_generation::maze::types::WorldPos;
use path_generation::maze::{CellType, MazeGenerator};
use path_generation::planner::Planner;
use path_generation::robot::types::{KnownCell, KnownMap};

fn planner_benchmark(c: &mut Criterion) {
    let maze = MazeGenerator::generate(42);

    // Build a fully-known map mirroring the true maze.
    let mut km = KnownMap::default();
    for row in 0..240 {
        for col in 0..240 {
            km.cells[row][col] = match maze.grid[row][col] {
                CellType::Passage => KnownCell::FreeSpace,
                CellType::Wall => KnownCell::Wall,
            };
        }
    }

    let robot_pos = WorldPos::from(maze.start);
    let goal = maze.goal;

    c.bench_function("astar_plan_240x240", |b| {
        b.iter(|| {
            let _ = Planner::plan(robot_pos, goal, &km);
        });
    });
}

criterion_group!(benches, planner_benchmark);
criterion_main!(benches);
