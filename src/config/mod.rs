use std::collections::HashSet;
use std::path::Path;

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct OsmWayConfig {
    pub osm_show_ways: u8,
    pub osm_all: OsmKeyFilter,
    pub osm_cars: OsmValueFilter,
    pub osm_walkers: OsmWalkersFilter,
    pub osm_cyclists: OsmCyclistsFilter,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct OsmKeyFilter {
    pub key: Vec<String>,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct OsmValueFilter {
    pub value: Vec<String>,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct OsmWalkersFilter {
    pub key: Vec<String>,
    pub value: Vec<String>,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct OsmCyclistsFilter {
    pub key: Vec<String>,
    pub value: Vec<String>,
}

/// Filter properties for OSM way parsing (maps to C++ `map_properties_t`).
#[derive(Debug, Clone, Default)]
pub struct OsmFilterProperties {
    pub keys: HashSet<String>,
    pub values: HashSet<String>,
}

impl OsmWayConfig {
    pub fn from_yaml_file(path: &Path) -> crate::error::Result<Self> {
        let contents = std::fs::read_to_string(path)
            .map_err(|e| crate::error::PlannerError::Config(e.to_string()))?;
        serde_yaml::from_str(&contents)
            .map_err(|e| crate::error::PlannerError::Config(e.to_string()))
    }

    pub fn filter_properties(&self) -> crate::error::Result<OsmFilterProperties> {
        let mut props = OsmFilterProperties::default();
        match self.osm_show_ways {
            0 => props.keys.extend(self.osm_all.key.iter().cloned()),
            1 => props.values.extend(self.osm_cars.value.iter().cloned()),
            2 => {
                props.keys.extend(self.osm_walkers.key.iter().cloned());
                props.values.extend(self.osm_walkers.value.iter().cloned());
            }
            3 => {
                props.keys.extend(self.osm_cyclists.key.iter().cloned());
                props.values.extend(self.osm_cyclists.value.iter().cloned());
            }
            other => {
                return Err(crate::error::PlannerError::Config(format!(
                    "invalid osm_show_ways: {other}"
                )));
            }
        }
        Ok(props)
    }
}

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
        let config: Self = serde_yaml::from_str(yaml)
            .map_err(|e| crate::error::PlannerError::Config(e.to_string()))?;
        config.validate()?;
        Ok(config)
    }

    pub fn from_yaml_file(path: &std::path::Path) -> crate::error::Result<Self> {
        let contents = std::fs::read_to_string(path)
            .map_err(|e| crate::error::PlannerError::Config(e.to_string()))?;
        Self::from_yaml_str(&contents)
    }

    pub fn to_yaml_string(&self) -> crate::error::Result<String> {
        serde_yaml::to_string(self).map_err(|e| crate::error::PlannerError::Config(e.to_string()))
    }

    pub fn validate(&self) -> crate::error::Result<()> {
        use crate::error::PlannerError;

        if crate::types::PlannerSystem::from_u8(self.general.system).is_none() {
            return Err(PlannerError::Config(format!(
                "invalid system id: {}",
                self.general.system
            )));
        }
        for (name, flag) in [
            ("general.pseudo", self.general.pseudo),
            ("general.log_data", self.general.log_data),
            ("general.print_path", self.general.print_path),
            ("rrt_params.random_seed", self.rrt_params.random_seed),
            ("rtsp_settings.shortcut", self.rtsp_settings.shortcut),
            ("rtsp_settings.swapping", self.rtsp_settings.swapping),
            ("rtsp_settings.genetic", self.rtsp_settings.genetic),
            (
                "rtsp_settings.ga.random_seed",
                self.rtsp_settings.ga.random_seed,
            ),
        ] {
            if flag > 1 {
                return Err(PlannerError::Config(format!(
                    "{name} must be 0 or 1, got {flag}"
                )));
            }
        }
        if self.general.max_iter == 0 {
            return Err(PlannerError::Config(
                "general.max_iter must be positive".into(),
            ));
        }
        if self.general.max_time == 0 {
            return Err(PlannerError::Config(
                "general.max_time must be positive".into(),
            ));
        }
        if !self.rrt_params.goal_bias.is_finite()
            || !(0.0..=1.0).contains(&self.rrt_params.goal_bias)
        {
            return Err(PlannerError::Config(format!(
                "rrt_params.goal_bias must be within [0, 1], got {}",
                self.rrt_params.goal_bias
            )));
        }
        if self.destinations.source_id == self.destinations.target_id {
            return Err(PlannerError::Config(
                "source_id and target_id must be different".into(),
            ));
        }
        let mut unique = std::collections::HashSet::new();
        unique.insert(self.destinations.source_id);
        unique.insert(self.destinations.target_id);
        for &objective in &self.destinations.objective_ids {
            if !unique.insert(objective) {
                return Err(PlannerError::Config(format!(
                    "duplicate destination node: {objective}"
                )));
            }
        }
        if self.rtsp_settings.genetic != 0
            && (self.rtsp_settings.ga.mutation_iter == 0
                || self.rtsp_settings.ga.population == 0
                || self.rtsp_settings.ga.generation == 0)
        {
            return Err(PlannerError::Config(
                "GA mutation_iter, population and generation must be positive when genetic=1"
                    .into(),
            ));
        }
        Ok(())
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

    #[test]
    fn config_round_trips_and_rejects_invalid_goal_bias() {
        let cfg = AlgorithmConfig::from_yaml_str(SAMPLE).unwrap();
        let encoded = cfg.to_yaml_string().unwrap();
        let round_trip = AlgorithmConfig::from_yaml_str(&encoded).unwrap();
        assert_eq!(round_trip.destinations.objective_ids, vec![1]);

        let invalid = SAMPLE.replace("goal_bias: 1.0", "goal_bias: 1.5");
        assert!(AlgorithmConfig::from_yaml_str(&invalid).is_err());
    }
}
