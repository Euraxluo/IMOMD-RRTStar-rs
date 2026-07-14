use std::fs::{self, File};
use std::io::{BufRead, BufReader, Write};
use std::path::{Path, PathBuf};

use crate::error::{PlannerError, Result};

/// One row of C++ experiment CSV (`CPU_time`, `path_cost`, `tree_size`).
#[derive(Debug, Clone, PartialEq)]
pub struct ExperimentRecord {
    pub cpu_time: f64,
    pub path_cost: f64,
    pub tree_size: f64,
}

/// In-memory experiment log matching C++ `logData_` CSV output.
#[derive(Debug, Default)]
pub struct ExperimentLog {
    records: Vec<ExperimentRecord>,
    file: Option<File>,
}

impl ExperimentLog {
    pub fn new(output_path: Option<&Path>) -> Result<Self> {
        let file = if let Some(path) = output_path {
            if let Some(parent) = path.parent() {
                fs::create_dir_all(parent).map_err(|e| PlannerError::Config(e.to_string()))?;
            }
            Some(
                File::create(path)
                    .map_err(|e| PlannerError::Config(format!("failed to create log file: {e}")))?,
            )
        } else {
            None
        };

        Ok(Self {
            records: Vec::new(),
            file,
        })
    }

    pub fn from_enabled(enabled: bool, output_path: Option<PathBuf>) -> Result<Option<Self>> {
        if enabled {
            Ok(Some(Self::new(output_path.as_deref())?))
        } else {
            Ok(None)
        }
    }

    pub fn record(&mut self, cpu_time: f64, path_cost: f64, tree_size: usize) {
        let row = ExperimentRecord {
            cpu_time,
            path_cost,
            tree_size: tree_size as f64,
        };
        self.records.push(row.clone());

        println!(
            "Elapsed time[s]: {:>10.4} | Path Cost[m]: {:>10.4} | Tree Size: {:>10}",
            cpu_time, path_cost, tree_size
        );

        if let Some(file) = &mut self.file {
            if self.records.len() == 1 {
                let _ = writeln!(file, "\"CPU_time\";\"path_cost\";\"tree_size\";");
            }
            let _ = writeln!(
                file,
                "\"{:.4}\";\"{:.4}\";\"{:.4}\";",
                row.cpu_time, row.path_cost, row.tree_size
            );
        }
    }

    pub fn records(&self) -> &[ExperimentRecord] {
        &self.records
    }
}

/// Parse a C++ experiment CSV (semicolon-separated, quoted fields).
pub fn parse_cpp_csv(path: &Path) -> Result<Vec<ExperimentRecord>> {
    let file = File::open(path)
        .map_err(|e| PlannerError::Config(format!("failed to open csv {path:?}: {e}")))?;
    let reader = BufReader::new(file);
    let mut records = Vec::new();

    for (line_no, line) in reader.lines().enumerate() {
        let line = line.map_err(|e| PlannerError::Config(e.to_string()))?;
        let line = line.trim();
        if line.is_empty() || line_no == 0 {
            continue;
        }

        let fields: Vec<&str> = line.split(';').filter(|s| !s.is_empty()).collect();
        if fields.len() < 3 {
            continue;
        }

        let parse_field = |s: &str| -> Result<f64> {
            s.trim_matches('"')
                .parse::<f64>()
                .map_err(|e| PlannerError::Config(format!("invalid csv number: {e}")))
        };

        records.push(ExperimentRecord {
            cpu_time: parse_field(fields[0])?,
            path_cost: parse_field(fields[1])?,
            tree_size: parse_field(fields[2])?,
        });
    }

    Ok(records)
}

/// Returns true when finite path costs never increase across logged improvements.
pub fn costs_are_non_increasing(records: &[ExperimentRecord]) -> bool {
    let mut last_finite = None;
    for row in records {
        if !row.path_cost.is_finite() {
            continue;
        }
        if let Some(prev) = last_finite {
            if row.path_cost > prev + 1e-6 {
                return false;
            }
        }
        last_finite = Some(row.path_cost);
    }
    true
}

/// Final finite path cost in a C++ reference log, if any.
pub fn final_finite_cost(records: &[ExperimentRecord]) -> Option<f64> {
    records
        .iter()
        .rev()
        .find(|r| r.path_cost.is_finite())
        .map(|r| r.path_cost)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_bugtrap_imomd_csv() {
        let path = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("tmp/imomd-cpp/experiments/bugtrap/sanfrancisco/imomd.csv");
        if !path.exists() {
            return;
        }
        let records = parse_cpp_csv(&path).unwrap();
        assert!(records.len() > 5);
        assert!(costs_are_non_increasing(&records));
        assert!(final_finite_cost(&records).unwrap() < 200_000.0);
    }
}
