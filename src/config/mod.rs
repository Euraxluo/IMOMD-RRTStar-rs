use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct AlgorithmConfig {
    pub general: GeneralConfig,
    pub rrt_params: RrtParams,
    pub destinations: DestinationsConfig,
    pub map: MapConfig,
    pub rtsp_settings: RtspSettings,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct GeneralConfig {
    pub system: u8,
    pub pseudo: u8,
    pub log_data: u8,
    pub print_path: u8,
    pub max_iter: usize,
    pub max_time: u64,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct RrtParams {
    pub goal_bias: f64,
    pub random_seed: u8,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct DestinationsConfig {
    pub source_id: usize,
    pub objective_ids: Vec<usize>,
    pub target_id: usize,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct MapConfig {
    pub r#type: i32,
    pub path: String,
    pub name: String,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct RtspSettings {
    pub shortcut: u8,
    pub swapping: u8,
    pub genetic: u8,
    pub ga: GaSettings,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct GaSettings {
    pub random_seed: u8,
    pub mutation_iter: usize,
    pub population: usize,
    pub generation: usize,
}

impl AlgorithmConfig {
    pub fn from_yaml_str(yaml: &str) -> crate::error::Result<Self> {
        serde_yaml::from_str(yaml)
            .map_err(|e| crate::error::PlannerError::Config(e.to_string()))
    }

    pub fn from_yaml_file(path: &std::path::Path) -> crate::error::Result<Self> {
        let contents = std::fs::read_to_string(path)
            .map_err(|e| crate::error::PlannerError::Config(e.to_string()))?;
        Self::from_yaml_str(&contents)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SAMPLE: &str = r#"
general:
  system: 0
  pseudo: 0
  log_data: 1
  print_path: 0
  max_iter: 1000000
  max_time: 60
rrt_params:
  goal_bias: 1.0
  random_seed: 0
destinations:
  source_id: 0
  objective_ids: [1]
  target_id: 2
map:
  type: -1
  path: ""
  name: ""
rtsp_settings:
  shortcut: 1
  swapping: 1
  genetic: 1
  ga:
    random_seed: 0
    mutation_iter: 10000
    population: 1000
    generation: 5
"#;

    #[test]
    fn parse_sample_config() {
        let cfg = AlgorithmConfig::from_yaml_str(SAMPLE).unwrap();
        assert_eq!(cfg.general.system, 0);
        assert_eq!(cfg.destinations.objective_ids, vec![1]);
        assert_eq!(cfg.map.r#type, -1);
    }
}
