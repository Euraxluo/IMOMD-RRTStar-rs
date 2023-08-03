use clap::Parser;
use log::info;
use std::env;
use IMOMD_RRTStar::prelude::Command;

fn main() {
    let args = Command::parse();
    Command::init_log(args.verbose);

    if let Some(config_path) = args.config.as_deref() {
        println!("Value for config: {}", config_path.display());
    }

    let cargo_path = env!("CARGO_MANIFEST_DIR");
    let exe_path = env::current_exe().unwrap();
    let exe_dir = exe_path.parent().unwrap();
    let work_dir = env::current_dir().unwrap();
    info!("cargo_path: {:?}", cargo_path);
    info!("exe_path: {:?}", exe_path);
    info!("exe_dir: {:?}", exe_dir);
    info!("work_dir: {:?}", work_dir);
}
