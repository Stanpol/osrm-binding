use serde::{Deserialize, Serialize};

#[derive(Debug, Deserialize, Serialize)]
pub struct Waypoint {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub hint: Option<String>,
    pub location: [f64; 2],
    pub name: String,
    pub distance: f64,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub nodes: Option<Vec<u64>>,
}
