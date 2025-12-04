use derive_builder::Builder;
use serde::{Deserialize, Serialize};
use crate::point::Point;
use crate::route::Route;
use crate::waypoints::Waypoint;

#[derive(Debug, Builder, Default)]
#[builder(setter(into, strip_option), default)]
pub struct TripRequest {
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
    #[builder(default = "true")]
    pub roundtrip: bool,
    #[builder(default)]
    pub source: Option<String>,
    #[builder(default)]
    pub destination: Option<String>,
    #[builder(default = "false")]
    pub steps: bool,
    #[builder(default)]
    pub annotations: Option<Vec<String>>,
    #[builder(default)]
    pub geometries: Option<String>,
    #[builder(default)]
    pub overview: Option<String>,
    #[builder(default)]
    pub exclude: Option<Vec<String>>,
}

#[derive(Debug, Deserialize, Serialize)]
#[allow(dead_code)]
pub struct TripResponse {
    pub code: String,
    pub trips: Vec<Route>,
    pub waypoints: Vec<Waypoint>,
}
