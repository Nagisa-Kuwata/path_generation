use criterion::{Criterion, criterion_group, criterion_main};
use path_generation::maze::{MazeGenerator, types::WorldPos};
use path_generation::sensor::Lrf;

fn lrf_benchmark(c: &mut Criterion) {
    let maze = MazeGenerator::generate(42);
    let robot_pos = WorldPos { x: 0.0, y: 0.0 };

    c.bench_function("lrf_scan_240x240", |b| {
        b.iter(|| {
            let _ = Lrf::scan(robot_pos, &maze);
        });
    });
}

criterion_group!(benches, lrf_benchmark);
criterion_main!(benches);
