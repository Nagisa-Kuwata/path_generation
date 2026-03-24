fn parse_seed() -> Option<u64> {
    let args: Vec<String> = std::env::args().collect();
    for i in 1..args.len() {
        if args[i] == "--seed" {
            if let Some(val) = args.get(i + 1) {
                return val.parse::<u64>().ok();
            }
        }
    }
    None
}

fn main() {
    let seed = parse_seed();
    let native_options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([900.0, 760.0])
            .with_title("Robot Maze Simulator"),
        ..Default::default()
    };
    eframe::run_native(
        "Robot Maze Simulator",
        native_options,
        Box::new(move |_cc| Ok(Box::new(path_generation::ui::SimApp::new(seed)))),
    )
    .expect("eframe failed to start");
}
