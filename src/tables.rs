use derive_builder::Builder;
use serde::{Deserialize, Serialize};

#[derive(Debug, Deserialize, Serialize)]
#[allow(dead_code)]
pub struct TableResponse {
    pub code: String,
    pub destinations: Vec<TableLocationEntry>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub durations: Option<Vec<Vec<Option<f64>>>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub distances: Option<Vec<Vec<Option<f64>>>>,
    sources: Vec<TableLocationEntry>,
}

#[derive(Debug, Deserialize, Serialize)]
#[allow(dead_code)]
pub struct TableLocationEntry {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    hint: Option<String>,
    location: [f64; 2],
    name: String,
    distance: f64,
}

#[derive(Debug, Builder, Clone)]
pub struct TableRequest{
    pub coordinates: Vec<(f64, f64)>,
    #[builder(default = "true")]
    pub include_duration: bool,
    #[builder(default = "true")]
    pub include_distance: bool,
    #[builder(default)]
    pub bearings: Option<Vec<Option<(i16, i16)>>>,
    #[builder(default)]
    pub radiuses: Option<Vec<Option<f64>>>,
    #[builder(default)]
    pub hints: Option<Vec<Option<String>>>,
    #[builder(default = "true")]
    pub generate_hints: bool,
    #[builder(default)]
    pub sources_indices: Option<Vec<usize>>,
    #[builder(default)]
    pub destinations_indices: Option<Vec<usize>>,
    #[builder(default)]
    pub approaches: Option<Vec<Option<String>>>,
    #[builder(default)]
    pub fallback_speed: Option<f64>,
    #[builder(default)]
    pub fallback_coordinate: Option<String>,
    #[builder(default)]
    pub scale_factor: Option<f64>,
    #[builder(default)]
    pub snapping: Option<String>,
}
