use derive_builder::Builder;
use crate::point::Point;
use serde::{Deserialize, Serialize};
use serde_json::Value;
use crate::waypoints::Waypoint;

#[derive(Debug, Builder, Clone, Default)]
#[builder(setter(into, strip_option), default)]
pub struct RouteRequest {
    #[builder(default)]
    pub points: Vec<Point>,
    #[builder(default)]
    pub bearings: Option<Vec<Option<(i16, i16)>>>,
    #[builder(default)]
    pub radiuses: Option<Vec<Option<f64>>>,
    #[builder(default)]
    pub hints: Option<Vec<Option<String>>>,
    #[builder(default = "true")]
    pub generate_hints: bool,
    #[builder(default)]
    pub approaches: Option<Vec<Option<String>>>,
    #[builder(default)]
    pub snapping: Option<String>,
    #[builder(default = "false")]
    pub steps: bool,
    #[builder(default)]
    pub alternatives: Option<i32>,
    #[builder(default)]
    pub annotations: Option<Vec<String>>,
    #[builder(default)]
    pub geometries: Option<String>,
    #[builder(default)]
    pub overview: Option<String>,
    #[builder(default = "false")]
    pub continue_straight: bool,
    #[builder(default)]
    pub exclude: Option<Vec<String>>,
    #[builder(default)]
    pub waypoints: Option<Vec<usize>>,
}

#[derive(Debug, Deserialize, Serialize)]
#[allow(dead_code)]
pub struct SimpleRouteResponse {
    pub code: String,
    pub durations: f64,
    pub distance: f64,
}

#[derive(Debug, Deserialize, Serialize)]
#[allow(dead_code)]
pub struct RouteResponse {
    pub code: String,
    pub routes: Vec<Route>,
    pub waypoints: Vec<Waypoint>,
}

#[derive(Debug, Deserialize, Serialize)]
pub struct OsrmResponse {
    pub code: String,
    pub routes: Vec<Route>,
    pub waypoints: Vec<Waypoint>,
}

#[derive(Debug, Deserialize, Serialize)]
pub struct Route {
    pub legs: Vec<Leg>,
    pub weight_name: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub geometry: Option<Value>,  // Can be String (polyline) or Object (GeoJSON), None when overview=false
    pub weight: f64,
    pub duration: f64,
    pub distance: f64,
}

#[derive(Debug, Deserialize, Serialize)]
pub struct Leg {
    #[serde(default)]
    pub steps: Vec<Step>,
    pub weight: f64,
    pub summary: String,
    pub duration: f64,
    pub distance: f64,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub annotation: Option<Annotation>,
}

#[derive(Debug, Deserialize, Serialize)]
pub struct Annotation {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub duration: Option<Vec<f64>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub distance: Option<Vec<f64>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub speed: Option<Vec<f64>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub weight: Option<Vec<f64>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub datasources: Option<Vec<u32>>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub nodes: Option<Vec<i64>>,  // Use i64 for large OSM node IDs
}

#[derive(Debug, Deserialize, Serialize)]
pub struct Step {
    #[serde(skip_serializing_if = "Option::is_none")]
    pub distance: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub duration: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub geometry: Option<Value>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub name: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub mode: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub maneuver: Option<Value>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub weight: Option<f64>,
}
