use derive_builder::Builder;
use crate::point::Point;
use serde::{Deserialize, Serialize};
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
    pub geometry: String,
    pub weight: f64,
    pub duration: f64,
    pub distance: f64,
}

#[derive(Debug, Deserialize, Serialize)]
pub struct Leg {
    pub steps: Vec<Step>,
    pub weight: f64,
    pub summary: String,
    pub duration: f64,
    pub distance: f64,
}

#[derive(Debug, Deserialize, Serialize)]
pub struct Step {
}
