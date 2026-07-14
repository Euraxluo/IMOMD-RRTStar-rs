use clap::Parser;
use log::info;
use std::process::ExitCode;

use IMOMD_RRTStar::error::PlannerError;
use IMOMD_RRTStar::prelude::{Command, PlanningSystem};
use IMOMD_RRTStar::types::PlanningResult;

fn main() -> ExitCode {
    let args = Command::parse();
    Command::init_log(args.verbose);

    let config_path = args
        .config
        .unwrap_or_else(|| std::path::PathBuf::from("config/algorithm_config.yaml"));

    info!("loading config from {}", config_path.display());

    match run_planner(&config_path) {
        Ok((result, print_path)) => {
            println!("Elapsed time[s]: {:>10.4}", result.elapsed_secs);
            println!("Path cost[m]:    {:>10.4}", result.cost);
            println!("Tree size:       {:>10}", result.explored_nodes);
            if print_path && !result.path.is_empty() {
                print!("Path: ");
                for node in &result.path {
                    print!("{node} -> ");
                }
                println!("#");
            }
            ExitCode::SUCCESS
        }
        Err(PlannerError::Disconnected(a, b)) => {
            eprintln!("planning failed: graph disconnected between {a} and {b}");
            ExitCode::from(2)
        }
        Err(e) => {
            eprintln!("planning failed: {e}");
            ExitCode::FAILURE
        }
    }
}

fn run_planner(config_path: &std::path::Path) -> Result<(PlanningResult, bool), PlannerError> {
    let mut system = PlanningSystem::from_yaml(config_path)?;
    let print_path = system.print_path_enabled();
    system.run().map(|result| (result, print_path))
}
