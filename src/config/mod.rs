use serde::{Deserialize, Serialize};

#[derive(Debug, Deserialize, Serialize)]
pub struct Config {
    pub name: String,
}

impl Config {

}



// // parse config from yaml
// fn parse_config() -> Config {
//     None
// }


