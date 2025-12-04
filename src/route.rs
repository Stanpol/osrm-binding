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
    #[builder(default = "false")]
    pub skip_waypoints: bool,
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
    #[serde(skip_serializing_if = "Option::is_none", default)]
    pub waypoints: Option<Vec<Waypoint>>,
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
    #[serde(skip_serializing_if = "Option::is_none", default, deserialize_with = "deserialize_nodes_optional")]
    pub nodes: Option<Vec<i64>>,  // Use i64 for large OSM node IDs
}

fn deserialize_nodes_optional<'de, D>(deserializer: D) -> Result<Option<Vec<i64>>, D::Error>
where
    D: serde::Deserializer<'de>,
{
    use serde::de::{Deserialize, Error};
    
    let opt: Option<Vec<serde_json::Value>> = Option::deserialize(deserializer)?;
    match opt {
        Some(values) => {
            let nodes: Result<Vec<i64>, _> = values.iter().map(|v| {
                match v {
                    serde_json::Value::Number(n) => {
                        // Try to get as i64 first
                        if let Some(i) = n.as_i64() {
                            Ok(i)
                        } else if let Some(u) = n.as_u64() {
                            // If it's a u64, try to convert to i64
                            i64::try_from(u).map_err(|_| {
                                Error::custom(format!("Node ID {} too large for i64", u))
                            })
                        } else if let Some(f) = n.as_f64() {
                            // If it's stored as float (like 10985247230.0), convert to i64
                            Ok(f as i64)
                        } else {
                            Err(Error::custom("Invalid number type for node ID"))
                        }
                    }
                    _ => Err(Error::custom("Node ID must be a number")),
                }
            }).collect();
            Ok(Some(nodes?))
        }
        None => Ok(None),
    }
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
