use clap::Parser;
use env_logger::Env;
use log::Level;
use std::path::PathBuf;
use std::str::FromStr;

/// Information Multi-Objective and Multi-Directional RRT* System for Path Planning.\n
/// This work reimplemented an anytime iterative system to concurrently solve the multi-objective path planning problem \n
/// and determine the visiting order of destinations using rust-lang
#[derive(Parser, Debug)]
#[command(author, bin_name = "IMOMD_RRTStar", version, about)]
pub struct Command {
    /// Path of the config file. eg:config.yaml
    #[arg(short, long, value_name = "FILE")]
    pub config: Option<PathBuf>,
    /// log level
    #[arg(short, long,action = clap::ArgAction::Count)]
    pub verbose: u8,
}

impl Command {
    pub fn init_log(verbose: u8) {
        // The logging level is set through the environment variable RUST_LOG,
        // which defaults to the info level
        env_logger::Builder::from_env(Env::default().default_filter_or({
            let level_env = match std::env::var("RUST_LOG") {
                Ok(val) => val,
                Err(_) => "info".to_string(),
            };
            println!("RUST_ENV_LOG: {}", level_env);
            let level_verbose = match verbose {
                0 => "ERROR",
                1 => "INFO",
                2 => "Debug",
                _ => "Trace",
            };
            println!("RUST_VERBOSE_LOG: {}", level_verbose);
            // Converts a string to the corresponding log-level enumeration
            let level = if let (Ok(level1), Ok(level2)) =
                (Level::from_str(&level_env), Level::from_str(&level_verbose))
            {
                // Use the cmp method to compare the sizes of the two log levels
                match level1.cmp(&level2) {
                    std::cmp::Ordering::Less => level2.to_string(),
                    std::cmp::Ordering::Equal => level2.to_string(),
                    std::cmp::Ordering::Greater => level1.to_string(),
                }
            } else {
                level_env
            };
            println!("RUST_LOG: {}", level);
            level
        }))
        .init();
    }
}
